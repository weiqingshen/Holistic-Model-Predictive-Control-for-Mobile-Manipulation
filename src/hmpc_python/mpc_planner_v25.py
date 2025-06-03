from distutils.util import execute

import casadi as ca
import numpy as np
from fk_generator import FKGenerator  # 依然基于之前写好的





class MPCPlannerV25:
    def __init__(self, logger, urdf_path,valid_steps):
        self.logger = logger
        self.logger.info("初始化 MPC 规划器 v25 完整版")

        self.n_states = 9  # base 3 + arm 6
        self.n_controls = 9
        self.N = 30
        self.step_horizon = 0.1
        self.valid_steps = valid_steps
        #初始化碰撞代价
        self.collision_cost_fn=0
        self.collision_constraints=[]

        # 权重矩阵：位置 + 姿态（rpy）
        # 终端代价（可以设更高）
        # self.Q_terminal = 10000*ca.diagcat(50, 50, 50, 10, 10, 10, 10)  # 强化终端误差优化
        self.Q_terminal = ca.diagcat(500000000, 500000000, 500000000, 100000000, 100000000, 100000000, 100000000)  # 强化终端误差优化
        #self.Q_execute=ca.diagcat(50000, 50000, 50000, 10000, 10000, 10000, 10000)  # 强化终端误差优化
        self.Q_execute=ca.diagcat(500000, 500000, 500000, 100000, 100000, 100000, 100000)  # 强化终端误差优化
        self.Q_task =  ca.diagcat(50, 50, 50, 5, 5, 5, 5)
        # self.Q_task =  ca.diagcat(0, 0, 0, 0, 0, 0, 0)
        # 优先移动机械臂
        # self.R = ca.diagcat(100, 100, 100, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
        self.R = ca.diagcat(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
        # self.R = ca.diagcat(50, 50, 50, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
        # self.R = ca.diagcat(1, 1, 1, 1, 1, 1, 1, 1, 1)
        #优先移动底盘
        #self.R = ca.diagcat(0.1, 0.1, 0.1, 10, 10, 10, 10, 10, 10)

        self.v_min = -0.2
        self.v_max = 0.2
        self.v_arm_min = -0.5
        self.v_arm_max = 0.5

        #这里是用来试验用
        # self.v_min = -0.2
        # self.v_max = 0.2
        # self.v_arm_min = -0.2
        # self.v_arm_max = 0.2

        self.fk_generator = FKGenerator(urdf_path)
        self.fk_func = self.fk_generator.fk_func_with_base_quat()  # 正确的

        self.create_symbolic_variables()
        self.build_cost_function()
        self.set_limit()
        self.set_optimize_option()

    def create_symbolic_variables(self):
        self.logger.info("创建符号变量")
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)
        self.U = ca.SX.sym('U', self.n_controls, self.N)
        self.P = ca.SX.sym('P', 7 + self.n_states)  # xyz + rpy + 当前状态

        x_t = ca.SX.sym('x_t', self.n_states)
        u_t = ca.SX.sym('u_t', self.n_controls)

        theta_t = x_t[2]
        vx_t = u_t[0]
        vy_t = u_t[1]
        omega_t = u_t[2]
        q_dot_t = u_t[3:]

        f_x = vx_t * ca.cos(theta_t) - vy_t * ca.sin(theta_t)
        f_y = vx_t * ca.sin(theta_t) + vy_t * ca.cos(theta_t)
        f_theta = omega_t

        self.f = ca.Function('f', [x_t, u_t], [ca.vertcat(f_x, f_y, f_theta, q_dot_t)])

    def build_cost_function(self):
        self.logger.info("构建代价函数和约束（位置 + 姿态优化）")
        self.cost_fn = 0  # 初始化代价函数
        constraints = [self.X[:, 0] - self.P[7:]]  # 当前状态约束

        target_ee_pose = self.P[0:7]  # 目标位姿 (xyz + rpy)

        # 位置、姿态优化和控制代价
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]

            q_total = st[0:9]  # base 3 + arm 6
            ee_pose = self.fk_func(q_total)

            # 姿态误差
            pose_error = ee_pose - target_ee_pose  # xyz + rpy
            self.cost_fn += ca.mtimes(pose_error.T, self.Q_task @ pose_error)
            self.cost_fn += ca.mtimes(con.T, self.R @ con)

            # 状态转移约束
            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_pred = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

            constraints.append(st_next - st_next_pred)

        # ✅ 将碰撞代价作为目标函数的一部分
        # print(f"⚡ 正在加入碰撞惩罚项: {self.collision_cost_fn}")

        # 将碰撞代价加入目标函数
        self.cost_fn += self.collision_cost_fn

        # 计算最终预测步（第 N+1 步）的关节状态
        terminal_state = self.X[:, -1]  # 最后一步的关节状态
        # terminal_state = self.X[:, self.valid_steps]  # 最后一步的关节状态

        q_total_terminal = terminal_state[:9]  # 提取 base 3 + arm 6
        ee_pose_terminal = self.fk_func(q_total_terminal)  # FK 计算末端位姿

        # 计算末端执行器与目标的误差
        terminal_error = ee_pose_terminal - target_ee_pose  # xyz + rpy 误差

        # 施加更高权重，让 MPC 终点更精确
        self.cost_fn += ca.mtimes(terminal_error.T, self.Q_terminal @ terminal_error)   # 放大权重

        execute_state = self.X[:, self.valid_steps]  # 最后一步的关节状态
        q_execute = execute_state[:9]  # 提取 base 3 + arm 6
        ee_q_execute = self.fk_func(q_execute)  # FK 计算末端位姿
        execute_error = ee_q_execute - target_ee_pose  # xyz + rpy 误差
        self.cost_fn += ca.mtimes(execute_error.T, self.Q_execute @ execute_error)   # 放大权重

        # 这里只添加状态约束
        constraints.extend(self.collision_constraints)  # 已经不再需要这个步骤，如果不再使用碰撞约束作为硬性约束可以移除

        print(f"\n📌 constraints 添加前=后总数: {len(constraints)}")
        self.g = ca.vertcat(*constraints)  # 只有状态约束和目标函数计算的约束，碰撞不再作为硬约束存在

    def set_optimize_option(self):
        self.logger.info("设置优化器参数")
        OPT_variables = ca.vertcat(self.X.reshape((-1, 1)), self.U.reshape((-1, 1)))

        nlp_prob = {
            'f': self.cost_fn,
            'x': OPT_variables,
            'g': self.g,
            'p': self.P
        }

        opts = {
            'ipopt': {
                'max_iter': 100,
                'print_level': 1,
                'acceptable_tol': 1e-9,
                'acceptable_obj_change_tol': 1e-9,
                'max_cpu_time': 5
            },
            'print_time': 0
        }
        # 打印约束 g
        # print(f"📌 当前约束 g 的形状: {self.g.shape}")
        # print(f"📌 当前约束 g 的内容: {self.g}")

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def set_limit(self):
        self.logger.info("设置变量约束")
        num_constraints = self.g.shape[0]
        num_vars = self.n_states * (self.N + 1) + self.n_controls * self.N

        lbx = -ca.inf * ca.DM.ones((num_vars, 1))
        ubx = ca.inf * ca.DM.ones((num_vars, 1))

        u_start = self.n_states * (self.N + 1)

        for k in range(self.N):
            base_idx = u_start + k * self.n_controls

            for i in range(3):
                lbx[base_idx + i] = self.v_min
                ubx[base_idx + i] = self.v_max

            for i in range(3, 9):
                lbx[base_idx + i] = self.v_arm_min
                ubx[base_idx + i] = self.v_arm_max

        self.args = {
            'lbg': -1e-5 * ca.DM.ones(num_constraints, 1),
            'ubg': 1e-5 * ca.DM.ones(num_constraints, 1),
            'lbx': lbx,
            'ubx': ubx
        }

    def set_reference(self, target_ee_pose, current_state):
        self.logger.info("设定目标末端位姿 + 当前状态")
        self.args['p'] = ca.vertcat(target_ee_pose, current_state)

    def set_x0(self, X0, u0):
        self.logger.info("设置优化初始值")
        self.args['x0'] = ca.vertcat(
            ca.reshape(X0, self.n_states * (self.N + 1), 1),
            ca.reshape(u0, self.n_controls * self.N, 1)
        )

    def get_states_and_control(self):
        self.logger.info("开始求解 MPC")
        try:
            sol = self.solver(
                x0=self.args['x0'],
                lbx=self.args['lbx'],
                ubx=self.args['ubx'],
                lbg=self.args['lbg'],
                ubg=self.args['ubg'],
                p=self.args['p']
            )
            self.logger.info("MPC 求解完成")
            X_opt = ca.reshape(sol['x'][:self.n_states * (self.N + 1)], self.n_states, self.N + 1)
            return X_opt
        except Exception as e:
            self.logger.error(f"MPC 求解失败: {e}")
            return None


    def set_collision_constraints(self, link_pairs, gradient, joint_distances):
        """
        设定碰撞约束，作为优化目标的一部分
        """
        self.logger.info("设定碰撞约束")

        # 确保 gradient 是 NumPy 数组
        if isinstance(gradient, list):
            gradient = np.array(gradient)

        self.link_pairs = link_pairs
        self.joint_distances = joint_distances

        self.collision_cost_fn = 0  # 碰撞代价，用于目标函数优化

        print("\n=== 🔍 碰撞检测信息 ===")
        for j, link_pair in enumerate(self.link_pairs):
            # 获取当前的距离 d
            d_initial = self.joint_distances[j]
            print(f"🛠 关节对 {link_pair} | 初始距离: {float(d_initial):.6f}")

            if d_initial < 0.05:  # 设置碰撞阈值
                # 计算关节变化后的近似碰撞距离 d_new
                grad_filtered = np.delete(gradient[:, j], 6, axis=0)  # 去除夹爪
                grad = ca.vertcat(*grad_filtered)  # 变成 (9,1)

                # 使用 CasADi 方法获取控制输入的符号（而不是 NumPy 数组）
                joint_change = self.U[:, j]  # 使用 CasADi 的符号变量

                # 计算关节变化量的影响：d_new = d_initial + (grad * joint_change)
                d_new = d_initial + ca.mtimes(grad.T, joint_change)  # 使用 CasADi 的矩阵乘法

                # 现在我们不将 d_new 直接转换为数值，而是将其作为目标函数的一部分
                # 计算惩罚项：惩罚项 = 1e4 * (0.05 - d_new) ** 2
                penalty = 1e2 * (0.05 - d_new) ** 2  # 直接把 d_new 看作符号变量

                # 将这个惩罚项累加到目标函数中
                self.collision_cost_fn += penalty  # 作为代价项加入目标函数

                print(f"⚠️ 碰撞风险！关节对: {link_pair} | 新距离: {d_new} | 惩罚: {penalty}")

                # 打印累积碰撞代价
                print(f"📌 碰撞代价累计：{self.collision_cost_fn}")

        # print(f"🔴 当前碰撞代价: {self.collision_cost_fn}")

        self.logger.info(f"✅ 碰撞代价已成功加入目标函数")

    # def adjust_R_matrix(self, current_ee_pose, target_ee_pose):
    #     # 使用 CasADi 的函数计算欧氏距离
    #     distance = ca.norm_2(current_ee_pose[:3] - target_ee_pose[:3])  # 只考虑位置的欧氏距离
    #
    #     # 将 CasADi 的 DM 对象转换为 NumPy 数组并格式化输出
    #     distance_value = distance.full().flatten()[0]  # 获取距离的数值
    #
    #     # 设置距离阈值（根据实际情况调整）
    #     distance_threshold = 0.05  # 假设0.5是一个合适的阈值
    #
    #     # 使用 CasADi 的比较操作符来代替 Python 中的 `if`
    #     distance_is_greater = ca.gt(distance, distance_threshold)  # distance > distance_threshold
    #
    #     # 根据符号表达式判断 R 矩阵
    #     self.R = ca.if_else(distance_is_greater,
    #                         ca.diagcat(0.1, 0.1, 0.1, 10, 10, 10, 10, 10, 10),  # 远离目标时优先控制底盘
    #                         ca.diagcat(10, 10, 10, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1))  # 靠近目标时优先控制机械臂
    #
    #     print(f"当前末端执行器与目标位置的距离: {distance_value:.2f}，使用的R矩阵为:")
    #     print(self.R)
    def quaternion_distance(self, q1, q2):
        # 计算四元数 q1 和 q2 之间的角度误差
        dot_product = ca.mtimes(ca.transpose(q1), q2)  # 四元数的点积
        # 计算旋转角度误差
        rotation_error = ca.acos(ca.fmin(ca.fabs(dot_product), 1.0))  # 限制内积的值在 -1 到 1 之间
        return rotation_error

    def adjust_Q_execute(self, current_ee_pose, target_ee_pose):
        # 计算位置误差和旋转误差（欧氏距离和四元数误差）
        distance = ca.norm_2(current_ee_pose[:3] - target_ee_pose[:3])  # 位置误差
        current_q = current_ee_pose[3:]  # 提取四元数
        target_q = target_ee_pose[3:]
        rotation_error = self.quaternion_distance(current_q, target_q)  # 旋转误差

        # 将 CasADi 的 DM 对象转换为数值（方便后续处理）
        distance_value = distance.full().flatten()[0]
        rotation_error_value = rotation_error.full().flatten()[0]
        total_error=0.9*distance_value+0.1*rotation_error
        # 设置误差阈值（当距离小于该值时增加 Q_execute）
        error_threshold = 0.1  # 你可以根据需求调整该阈值
        if(total_error<error_threshold):
            adjust1=100000/(total_error+0.0001)
            adjust2=adjust1/5
        else:
            adjust1=1000000
            adjust2=adjust1/5
        self.Q_execute=ca.diagcat(
            adjust1,adjust1,adjust1,adjust2,adjust2,adjust2,adjust2
        )
        # 根据与目标的距离动态调整 Q_execute


        # 调试输出（可选）
        print(f"Q_execute 已根据距离进行调整: {self.Q_execute}")

    # def set_symbolic_collision_constraints(self, symbolic_result, danger_threshold, a, b):
    #     """
    #     使用字符串转 CasADi 表达式方式，避免 lambdify 失败，适配复杂嵌套符号结构。
    #     """
    #     import sympy as sp
    #
    #     if not symbolic_result:
    #         self.logger.info("✅ 当前无危险碰撞点，跳过符号碰撞约束设置。")
    #         return
    #
    #     self.logger.info("设置符号碰撞约束（字符串解析）")
    #     self.collision_cost_fn = 0
    #
    #     # 定义变量顺序
    #     sp_vars = sp.symbols('x y theta q1 q2 q3 q4 q5 q6 g')
    #     casadi_syms = [f'x{i}' for i in range(10)]  # 临时占位变量名
    #
    #     print(symbolic_result)
    #
    #     for pair, dist_val, dist_expr in symbolic_result:
    #         if dist_expr is None or str(dist_expr).lower() == 'nan':
    #             self.logger.warn(f"⚠️ 表达式非法，跳过 {pair}")
    #             continue
    #
    #         try:
    #             # 将表达式中的符号变量替换为 x0...x9
    #             replacements = {sym: sp.Symbol(name) for sym, name in zip(sp_vars, casadi_syms)}
    #             expr_safe = dist_expr.subs(replacements)
    #             expr_str = str(expr_safe)
    #
    #             # 解析为 CasADi 表达式
    #             for k in range(self.N):
    #                 casadi_vars = [self.X[i, k] for i in range(self.n_states)] + [0.001]  # g 固定值
    #                 env = {f'x{i}': casadi_vars[i] for i in range(10)}
    #                 dist_expr_casadi = eval(expr_str, {"sin": ca.sin, "cos": ca.cos, "sqrt": ca.sqrt, **env})
    #
    #                 penalty = a * ca.exp(b * (danger_threshold-dist_expr_casadi))
    #                 self.collision_cost_fn += penalty
    #         except Exception as e:
    #             self.logger.warn(f"⚠️ 表达式解析失败：{e}，跳过 {pair}")
    #             continue
    #
    #     self.logger.info("✅ 符号碰撞惩罚构建完成（字符串方式）")

    # def set_symbolic_collision_constraints(self, symbolic_result, danger_threshold, a, b):
    #     import sympy as sp
    #
    #     if not symbolic_result:
    #         self.logger.info("✅ 当前无危险碰撞点，跳过符号碰撞约束设置。")
    #         return
    #
    #     # 清洗非法数据结构
    #     symbolic_result = [
    #         item for item in symbolic_result
    #         if isinstance(item, (list, tuple)) and len(item) == 3
    #     ]
    #
    #     self.logger.info("设置符号碰撞约束（字符串解析）")
    #     self.collision_cost_fn = 0
    #
    #     sp_vars = sp.symbols('x y theta q1 q2 q3 q4 q5 q6 g')
    #     casadi_syms = [f'x{i}' for i in range(10)]
    #
    #     for pair, dist_val, dist_expr in symbolic_result:
    #         if dist_expr is None or str(dist_expr).lower() == 'nan':
    #             self.logger.warn(f"⚠️ 表达式非法，跳过 {pair}")
    #             continue
    #
    #         try:
    #             # 替换符号变量名为 x0 ~ x9
    #             replacements = {sym: sp.Symbol(name) for sym, name in zip(sp_vars, casadi_syms)}
    #             expr_safe = dist_expr.subs(replacements)
    #             expr_str = str(expr_safe)
    #
    #             for k in range(self.N):  # N 是时域长度
    #                 casadi_vars = [self.X[i, k] for i in range(self.n_states)] + [0.001]  # g固定
    #                 env = {f'x{i}': casadi_vars[i] for i in range(10)}
    #                 dist_expr_casadi = eval(expr_str, {"sin": ca.sin, "cos": ca.cos, "sqrt": ca.sqrt, **env})
    #                 print(dist_expr_casadi)
    #                 penalty = a * ca.exp(b * (danger_threshold - dist_expr_casadi))
    #                 self.collision_cost_fn += penalty
    #         except Exception as e:
    #             self.logger.warn(f"⚠️ 表达式解析失败：{e}，跳过 {pair}")
    #             continue
    #
    #     self.logger.info("✅ 符号碰撞惩罚构建完成（字符串方式）")

    def set_symbolic_collision_constraints(self, symbolic_result, danger_threshold, a, b, custom_params=None):
        if not symbolic_result:
            self.logger.info("✅ 当前无危险碰撞点，跳过符号碰撞约束设置。")
            return

        # 清洗非法数据结构，确保每个元素是四元组
        symbolic_result = [
            item for item in symbolic_result
            if isinstance(item, (list, tuple)) and len(item) == 4
        ]

        self.logger.info("设置符号碰撞约束（CasADi 变量）")
        self.collision_cost_fn = 0

        # 使用 custom_params 作为可选自定义参数
        if custom_params is None:
            custom_params = {}

        # 遍历每一对碰撞点
        for item in symbolic_result:
            pair = tuple(item[0])  # 确保pair是元组
            dist_val, dist_expr, t_idx = item[1], item[2], item[3]

            if dist_expr is None or str(dist_expr).lower() == 'nan':
                self.logger.warn(f"⚠️ 表达式非法，跳过 {pair}")
                continue

            # 检查该 pair 是否有自定义的危险阈值参数
            if pair in custom_params:
                custom_danger_threshold = custom_params[pair].get('danger_threshold', danger_threshold)
                custom_a = custom_params[pair].get('a', a)
                custom_b = custom_params[pair].get('b', b)
            else:
                # 使用默认值
                custom_danger_threshold = danger_threshold
                custom_a = a
                custom_b = b

            try:
                # dist_expr 已经是符号表达式，不需要再次创建 Function
                dist_expr_casadi_val = dist_expr  # 直接使用符号表达式进行计算

                # 如果符号距离大于危险阈值，计算惩罚项
                penalty = custom_a * ca.exp(custom_b * (custom_danger_threshold - dist_expr_casadi_val))  # 计算惩罚项

                # 将每个时间步的惩罚项加到总成本函数中
                self.collision_cost_fn += penalty

            except Exception as e:
                self.logger.warn(f"⚠️ 计算碰撞惩罚时失败：{e}，跳过 {pair}")
                continue

        self.logger.info("✅ 符号碰撞惩罚构建完成（CasADi 变量）")



    def adjust_R_matrix(self, current_ee_pose, target_ee_pose):
        # 位置误差
        distance = ca.norm_2(current_ee_pose[:3] - target_ee_pose[:3])  # 只考虑位置的欧氏距离

        # 四元数误差
        current_q = current_ee_pose[3:]  # 提取四元数
        target_q = target_ee_pose[3:]  # 提取四元数
        rotation_error = self.quaternion_distance(current_q, target_q)

        # 将 CasADi 的 DM 对象转换为 NumPy 数组并格式化输出
        distance_value = distance.full().flatten()[0]  # 获取距离的数值
        rotation_error_value = rotation_error.full().flatten()[0]  # 获取旋转误差的数值

        # 设置距离和旋转误差阈值（根据实际情况调整）
        # distance_threshold = 0.05  # 假设0.05是一个合适的距离阈值
        error_threshold = 1

        # 设置位置误差和旋转误差的权重
        w_position = 0.8  # 位置误差的权重
        w_rotation = 0.2  # 旋转误差的权重

        # 计算加权的总误差
        based_error=distance
        total_error = w_position * distance + w_rotation * rotation_error

        # 使用 Sigmoid 函数平滑过渡（sigmoid 函数会根据总误差平滑调节权重）
        sig_value = 1 / (1 + ca.exp(-5 * (total_error - error_threshold)))  # 控制平滑过渡
        based_sig_value=1 / (1 + ca.exp(-4 * (based_error - error_threshold)))  # 控制平滑过渡
        # 打印 total_error 和 error_threshold 的值，帮助你理解 sig_value 的变化
        print(f"total_error: {total_error}, error_threshold: {error_threshold}")

        # 打印 sig_value 的值
        print(f"sig_value: {sig_value}")
        # 根据总误差平滑调节 R 矩阵
        self.R = ca.diagcat(
            (1 - based_sig_value) * 30 + based_sig_value * 0.03,  # 底盘的权重（总误差大时底盘更重要）
            (1 - based_sig_value) * 30 + based_sig_value * 0.03,  # 底盘的权重（总误差大时底盘更重要）
            ((1 - based_sig_value) * 30 + based_sig_value * 0.03)*2,  # 底盘的权重（总误差大时底盘更重要）
            (sig_value * 30 + (1 - sig_value) * 0.03)*1 ,  # 机械臂的权重（总误差小且接近目标时机械臂更重要）
            (sig_value * 30 + (1 - sig_value) * 0.03)*1 ,  # 机械臂的权重（总误差小且接近目标时机械臂更重要）
            (sig_value * 30 + (1 - sig_value) * 0.03)*1 ,  # 机械臂的权重（总误差小且接近目标时机械臂更重要）
            (sig_value * 30 + (1 - sig_value) * 0.03)*1 ,  # 机械臂的权重（总误差小且接近目标时机械臂更重要）
            (sig_value * 30 + (1 - sig_value) * 0.03)*1 ,  # 机械臂的权重（总误差小且接近目标时机械臂更重要）
            (sig_value * 30 + (1 - sig_value) * 0.03)*1  # 机械臂的权重（总误差小且接近目标时机械臂更重要）
        )


        # 打印误差和 R 矩阵
        print(
            f"当前末端执行器与目标位置的距离: {distance_value:.5f}，旋转误差: {rotation_error_value:.5f}，使用的R矩阵为:")

        # 将 CasADi 表达式转换为 NumPy 数组
        R_numpy = self.R.full()

        # 将矩阵重新格式化为 9x9
        R_reshaped = R_numpy.reshape((9, 9))

        # 打印格式化后的 9x9 矩阵
        for row in R_reshaped:
            print(" ".join([f"{val:.6f}" for val in row]))  # 使用 6 位小数进行格式化输出

        # if distance>rotation_error:
        #     distance=distance*3


        control_unit=(distance-rotation_error)/(min(distance,rotation_error)+0.00001)
        print(control_unit)
        # 使用 Sigmoid 函数平滑过渡，得到 sig_value_Q
        sig_value_Q = 1 / (1 + ca.exp(-1 * (control_unit)))  # 控制平滑过渡
        #sig_value越大 d越大 r越小 所以优先减小方向误差 需要大r 也就是小sig_value
        print(f"sig_value的值是{sig_value_Q}")
        d=sig_value_Q*1+(1-sig_value_Q)*0.1
        r=(1-sig_value_Q)*1+sig_value_Q*0.1

        # if distance<rotation_error or distance<0.1:
        #     d=d*50
        #     r=r*50

        d=d/(total_error*total_error*total_error*total_error+0.0001)
        r=r/(total_error*total_error*total_error*total_error+0.0001)
        self.Q_task = ca.diagcat(
            d,
            d,
            d,
            r,
            r,
            r,
            r
        )

        # 打印 Q_task 矩阵
        print(f"当前 Q_task 矩阵为:")
        Q_task_numpy = self.Q_task.full()

        # 将矩阵重新格式化为 7x7
        Q_task_reshaped = Q_task_numpy.reshape((7, 7))

        # 打印格式化后的 7x7 矩阵
        for row in Q_task_reshaped:
            print(" ".join([f"{val:.6f}" for val in row]))  # 使用 6 位小数进行格式化输出