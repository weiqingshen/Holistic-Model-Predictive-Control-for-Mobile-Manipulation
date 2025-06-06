import casadi as ca
import numpy as np
from fk_generator import FKGenerator  # 依然基于之前写好的

class HMPCPlanner_task3:
    def __init__(self, logger, urdf_path,valid_steps,x_p, y_p, r):
        self.logger = logger
        self.logger.info("初始化 HMPC 规划器 task3 完整版")
        self.valid_steps = valid_steps
        self.n_states = 9  # base 3 + arm 6
        self.n_controls = 9
        self.N = 30
        self.step_horizon = 0.1
        self.chassis_dist_constraint = None  # 用于存参数

        #初始化碰撞代价
        self.collision_cost_fn=0
        self.collision_constraints=[]

        # 权重矩阵：位置 + 姿态（rpy）
        # 终端代价（可以设更高）
        self.Q_terminal = ca.diagcat(500, 500, 500, 100, 100, 100, 100)  # 强化终端误差优化

        self.Q_execute=ca.diagcat(1000, 1000, 1000, 500, 500, 500, 500)  # 强化终端误差优化

        self.Q_continuity=ca.diagcat(10, 10, 10, 10, 10, 10, 10, 10, 10)  # 连续性惩罚的对角矩阵

        # self.Q_task = ca.diagcat(50, 50, 50, 5, 5, 5, 5)
        self.Q_task = ca.diagcat(5, 5, 5, 3, 3, 3, 3)
        # 优先移动机械臂
        # self.R = ca.diagcat(100, 100, 100, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        #self.R = ca.diagcat(10, 10, 10, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
        # self.R = ca.diagcat(1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        #v15原来的
        # self.R = ca.diagcat(0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        # self.R = ca.diagcat(20, 20, 25, 5, 5, 5, 5, 5, 5)

        #self.R = ca.diagcat(5, 5, 5, 5, 5, 5, 5, 5, 5)

        cost=1
        self.R = ca.diagcat(cost,cost , cost, cost, cost, cost, cost, cost, cost)

        # max=99999999999999
        # self.R = ca.diagcat(max, max, max, 5, 5, 5, 5, 5, 5)


        #优先移动底盘
        #self.R = ca.diagcat(0.1, 0.1, 0.1, 10, 10, 10, 10, 10, 10)

        # self.v_min = -1
        # self.v_max = 1
        # self.v_arm_min = -1
        # self.v_arm_max = 1

        self.v_min = -1
        self.v_max = 1
        self.v_arm_min = -1
        self.v_arm_max = 1

        #这里是用来试验用
        # self.v_min = -0.2
        # self.v_max = 0.2
        # self.v_arm_min = -0.2
        # self.v_arm_max = 0.2

        self.fk_generator = FKGenerator(urdf_path)
        self.fk_func = self.fk_generator.fk_func_with_base_quat()  # 正确的

        self.logger.info(f"添加底盘距离约束: 保持至少距离 {r}，目标点=({x_p}, {y_p})")
        self.chassis_dist_constraint = (x_p, y_p, r)

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
        constraints = []

        num_eq = 0  # 等式约束数量
        num_ineq = 0  # 不等式约束数量

        # ===== 1. 初始状态等式约束 =====
        constraints.append(self.X[:, 0] - self.P[7:])  # X[:,0] - 当前状态 == 0
        num_eq += self.n_states  # 9

        target_ee_pose = self.P[0:7]  # 目标位姿 (xyz + rpy)

        # ===== 2. 位置、姿态优化和状态转移约束 =====
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]

            q_total = st[0:9]  # base 3 + arm 6
            ee_pose = self.fk_func(q_total)

            # 姿态误差
            pose_error = ee_pose - target_ee_pose  # xyz + rpy
            self.cost_fn += ca.mtimes(pose_error.T, self.Q_task @ pose_error)
            self.cost_fn += ca.mtimes(con.T, self.R @ con)

            # 状态转移等式约束
            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_pred = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            constraints.append(st_next - st_next_pred)
            num_eq += self.n_states  # 9

        # ===== 3. 碰撞代价加到目标函数里 =====
        print(f"⚡ 正在加入碰撞惩罚项: {self.collision_cost_fn}")
        self.cost_fn += self.collision_cost_fn

        # ===== 4. 末端误差终端权重 =====
        terminal_state = self.X[:, self.valid_steps]
        q_total_terminal = terminal_state[:9]
        ee_pose_terminal = self.fk_func(q_total_terminal)
        terminal_error = ee_pose_terminal - target_ee_pose
        self.cost_fn += ca.mtimes(terminal_error.T, self.Q_terminal @ terminal_error)

        execute_state = self.X[:, self.valid_steps]
        q_execute = execute_state[:9]
        ee_q_execute = self.fk_func(q_execute)
        execute_error = ee_q_execute - target_ee_pose
        self.cost_fn += ca.mtimes(execute_error.T, self.Q_execute @ execute_error)

        # ===== 5. 连续性惩罚 =====
        continuity_penalty = 0
        for k in range(self.N - 1):
            delta_u = self.U[:, k + 1] - self.U[:, k]
            continuity_penalty += ca.mtimes(delta_u.T, self.Q_continuity @ delta_u)
        self.cost_fn += continuity_penalty

        # ===== 6. 底盘距离不等式约束 =====
        if self.chassis_dist_constraint is not None:
            x_p, y_p, r = self.chassis_dist_constraint
            for k in range(self.N + 1):
                x = self.X[0, k]
                y = self.X[1, k]
                expr = r ** 2 - (x - x_p) ** 2 - (y - y_p) ** 2  # (x,y)与(x_p,y_p)距离>=r
                constraints.append(expr)
                num_ineq += 1

        # ===== 7. 其他碰撞硬约束等（如果用到）=====
        constraints.extend(self.collision_constraints)  # 通常为空

        print(f"\n📌 constraints 添加前=后总数: {len(constraints)}")
        self.g = ca.vertcat(*constraints)
        self.num_eq = num_eq
        self.num_ineq = num_ineq

    # def build_cost_function(self):
    #     self.logger.info("构建代价函数和约束（位置 + 姿态优化）")
    #     self.cost_fn = 0  # 初始化代价函数
    #     constraints = [self.X[:, 0] - self.P[7:]]  # 当前状态约束
    #
    #     target_ee_pose = self.P[0:7]  # 目标位姿 (xyz + rpy)
    #
    #     # 位置、姿态优化和控制代价
    #     for k in range(self.N):
    #         st = self.X[:, k]
    #         con = self.U[:, k]
    #
    #         q_total = st[0:9]  # base 3 + arm 6
    #         ee_pose = self.fk_func(q_total)
    #
    #         # 姿态误差
    #         pose_error = ee_pose - target_ee_pose  # xyz + rpy
    #         self.cost_fn += ca.mtimes(pose_error.T, self.Q_task @ pose_error)
    #         self.cost_fn += ca.mtimes(con.T, self.R @ con)
    #
    #         # 状态转移约束
    #         st_next = self.X[:, k + 1]
    #         k1 = self.f(st, con)
    #         k2 = self.f(st + self.step_horizon / 2 * k1, con)
    #         k3 = self.f(st + self.step_horizon / 2 * k2, con)
    #         k4 = self.f(st + self.step_horizon * k3, con)
    #         st_next_pred = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    #
    #         constraints.append(st_next - st_next_pred)
    #
    #     # ✅ 将碰撞代价作为目标函数的一部分
    #     print(f"⚡ 正在加入碰撞惩罚项: {self.collision_cost_fn}")
    #
    #     # 将碰撞代价加入目标函数
    #     self.cost_fn += self.collision_cost_fn
    #
    #     # 计算最终预测步（第 N+1 步）的关节状态
    #     terminal_state = self.X[:, self.valid_steps]  # 最后一步的关节状态
    #     q_total_terminal = terminal_state[:9]  # 提取 base 3 + arm 6
    #
    #     # 计算该状态下的末端执行器位姿
    #     ee_pose_terminal = self.fk_func(q_total_terminal)  # FK 计算末端位姿
    #
    #     # 计算末端执行器与目标的误差
    #     terminal_error = ee_pose_terminal - target_ee_pose  # xyz + rpy 误差
    #
    #     # 施加更高权重，让 MPC 终点更精确
    #     self.cost_fn += ca.mtimes(terminal_error.T, self.Q_terminal @ terminal_error)   # 放大权重
    #
    #     execute_state = self.X[:, self.valid_steps]  # 最后一步的关节状态
    #     q_execute = execute_state[:9]  # 提取 base 3 + arm 6
    #     ee_q_execute = self.fk_func(q_execute)  # FK 计算末端位姿
    #     execute_error = ee_q_execute - target_ee_pose  # xyz + rpy 误差
    #     self.cost_fn += ca.mtimes(execute_error.T, self.Q_execute @ execute_error)   # 放大权重
    #
    #     # 添加连续性惩罚
    #     continuity_penalty = 0  # 初始化连续性惩罚项
    #
    #     for k in range(self.N - 1):  # 不在最后一步惩罚，因为最后一步没有下一个控制输入
    #         delta_u = self.U[:, k + 1] - self.U[:, k]  # 控制输入之间的差异
    #         continuity_penalty += ca.mtimes(delta_u.T, self.Q_continuity @ delta_u)  # 加入惩罚项
    #
    #     self.cost_fn += continuity_penalty  # 将连续性惩罚项加到总代价函数中
    #
    #     # 统一添加底盘距离约束
    #     if self.chassis_dist_constraint is not None:
    #         x_p, y_p, r = self.chassis_dist_constraint
    #         for k in range(self.N + 1):
    #             x = self.X[0, k]
    #             y = self.X[1, k]
    #             expr = r ** 2 - (x - x_p) ** 2 - (y - y_p) ** 2
    #             constraints.append(expr)
    #
    #     # # ✅ 输出 X 矩阵的形状
    #     # print(f"📌 X 形状: {self.X.shape}")
    #     #
    #     # # ✅ 输出 ee_pose_terminal 和 terminal_error 形状
    #     # print(f"📌 ee_pose_terminal 形状: {ee_pose_terminal.shape}")
    #     # print(f"📌 terminal_error 形状: {terminal_error.shape}")
    #     # print(f"📌 目标末端位姿 target_ee_pose: {target_ee_pose}")
    #     # print(f"📌 终端误差 terminal_error: {terminal_error}")
    #     #
    #     # # ✅ 继续调试 `constraints` 的数量
    #     # print(f"\n📌 constraints 添加前总数: {len(constraints)}")
    #     #
    #     # # 我们不再将碰撞约束加到 `self.g`，而是已经在目标函数中处理了
    #     # print(f"\n📌 collision_constraints 总数: {len(self.collision_constraints)}")
    #
    #     # 这里只添加状态约束
    #     constraints.extend(self.collision_constraints)  # 已经不再需要这个步骤，如果不再使用碰撞约束作为硬性约束可以移除
    #
    #     print(f"\n📌 constraints 添加前=后总数: {len(constraints)}")
    #     self.g = ca.vertcat(*constraints)  # 只有状态约束和目标函数计算的约束，碰撞不再作为硬约束存在


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
        print(f"📌 当前约束 g 的形状: {self.g.shape}")
        print(f"📌 当前约束 g 的内容: {self.g}")

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

        # 区分等式和不等式上下界
        lbg = []
        ubg = []

        # 等式约束：-1e-5 ~ 1e-5
        for _ in range(self.num_eq):
            lbg.append(-1e-5)
            ubg.append(1e-5)

        # 不等式约束：-inf ~ 0
        for _ in range(self.num_ineq):
            lbg.append(-ca.inf)
            ubg.append(0)

        self.args = {
            'lbg': ca.DM(lbg),
            'ubg': ca.DM(ubg),
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

        print(f"🔴 当前碰撞代价: {self.collision_cost_fn}")

        self.logger.info(f"✅ 碰撞代价已成功加入目标函数")






