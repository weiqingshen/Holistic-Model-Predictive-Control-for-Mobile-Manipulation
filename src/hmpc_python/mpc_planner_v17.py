import casadi as ca
import numpy as np
from fk_generator import FKGenerator  # 依然基于之前写好的

class MPCPlannerV17:
    def __init__(self, logger, urdf_path,valid_steps):
        self.logger = logger
        self.logger.info("初始化 MPC 规划器 v17 完整版")

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
        self.Q_terminal = ca.diagcat(30, 30, 30, 10, 10, 10, 10)  # 强化终端误差优化

        self.Q_task = ca.diagcat(50, 50, 50, 5, 5, 5, 5)

        # 优先移动机械臂
        # self.R = ca.diagcat(100, 100, 100, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        self.R = ca.diagcat(10, 10, 10, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        #优先移动底盘
        #self.R = ca.diagcat(0.1, 0.1, 0.1, 10, 10, 10, 10, 10, 10)

        self.v_min = -1200
        self.v_max = 1200
        self.v_arm_min = -100
        self.v_arm_max = 100

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


        # 定义每个关键点的权重矩阵（类似 Q_terminal 的矩阵）
        #注意力权重矩阵
        key_points_weight=100
        self.Q_key_points = [
            ca.diagcat(key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight),  # 关键点 1 的权重矩阵
            ca.diagcat(key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight),  # 关键点 2 的权重矩阵
            ca.diagcat(key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight),  # 关键点 3 的权重矩阵
            ca.diagcat(key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight, key_points_weight),  # 关键点 4 的权重矩阵

        ]

        # 定义关键点和它们对应的时间步
        self.key_points = np.array([
            [-1.1, -0.169, 0.021, -0.025, -0.212, -0.5],
            [-1.776, -0.119, 0.098, -0.074, -0.206, -0.5],
            [-2.823, -0.069, 0.174, -0.122, -0.201, -0.5],
            [-3.267, -0.063, 0.173, -0.123, -0.196, -0.5]
        ])


        # 定义每个关键点对应的时间步
        self.key_point_steps = [10, 15, 20, 25]  # 这些关键点应该在时间步5, 10, 15, 20, 25, 30处被应用

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
        print(f"⚡ 正在加入碰撞惩罚项: {self.collision_cost_fn}")

        # if self.collision_cost_fn > 0:
        #     print("✅ 碰撞代价已成功加入目标函数！")
        # else:
        #     print("⚠️ 警告：碰撞代价仍然为 0，MPC 可能未考虑避障！")

        # 将碰撞代价加入目标函数
        self.cost_fn += self.collision_cost_fn

        # 计算最终预测步（第 N+1 步）的关节状态
        # terminal_state = self.X[:, -1]  # 最后一步的关节状态
        terminal_state = self.X[:, self.valid_steps]  # 最后一步的关节状态
        q_total_terminal = terminal_state[:9]  # 提取 base 3 + arm 6

        # 计算该状态下的末端执行器位姿
        ee_pose_terminal = self.fk_func(q_total_terminal)  # FK 计算末端位姿

        # 计算末端执行器与目标的误差
        terminal_error = ee_pose_terminal - target_ee_pose  # xyz + rpy 误差

        # 施加更高权重，让 MPC 终点更精确
        self.cost_fn += ca.mtimes(terminal_error.T, self.Q_terminal @ terminal_error)   # 放大权重

        # # ✅ 输出 X 矩阵的形状
        # print(f"📌 X 形状: {self.X.shape}")
        #
        # # ✅ 输出 ee_pose_terminal 和 terminal_error 形状
        # print(f"📌 ee_pose_terminal 形状: {ee_pose_terminal.shape}")
        # print(f"📌 terminal_error 形状: {terminal_error.shape}")
        # print(f"📌 目标末端位姿 target_ee_pose: {target_ee_pose}")
        # print(f"📌 终端误差 terminal_error: {terminal_error}")
        #
        # # ✅ 继续调试 `constraints` 的数量
        # print(f"\n📌 constraints 添加前总数: {len(constraints)}")
        #
        # # 我们不再将碰撞约束加到 `self.g`，而是已经在目标函数中处理了
        # print(f"\n📌 collision_constraints 总数: {len(self.collision_constraints)}")

        # 这里只添加状态约束
        constraints.extend(self.collision_constraints)  # 已经不再需要这个步骤，如果不再使用碰撞约束作为硬性约束可以移除

        print(f"\n📌 constraints 添加前=后总数: {len(constraints)}")
        self.g = ca.vertcat(*constraints)  # 只有状态约束和目标函数计算的约束，碰撞不再作为硬约束存在

    def build_cost_function_follow(self):
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

            # 对于关键点，添加误差代价
            if k in self.key_point_steps:
                key_point_idx = self.key_point_steps.index(k)
                key_point = self.key_points[key_point_idx]
                key_point_error = st[3:9] - key_point  # 关节配置误差
                self.cost_fn += ca.mtimes(key_point_error.T, self.Q_key_points[key_point_idx] @ key_point_error)

            # 状态转移约束
            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_pred = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

            constraints.append(st_next - st_next_pred)

        terminal_state = self.X[:, -1]# 最后一步的关节状态
        q_total_terminal = terminal_state[:9]  # 提取 base 3 + arm 6

        # 计算该状态下的末端执行器位姿
        ee_pose_terminal = self.fk_func(q_total_terminal)  # FK 计算末端位姿

        # 计算末端执行器与目标的误差
        terminal_error = ee_pose_terminal - target_ee_pose  # xyz + rpy 误差

        # 施加更高权重，让 MPC 终点更精确
        self.cost_fn += ca.mtimes(terminal_error.T, self.Q_terminal @ terminal_error)  # 放大权重

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

        print(f"🔴 当前碰撞代价: {self.collision_cost_fn}")

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
        error_threshold = 1.2

        # 设置位置误差和旋转误差的权重
        w_position = 0.8  # 位置误差的权重
        w_rotation = 0.2  # 旋转误差的权重

        # 计算加权的总误差
        based_error=distance
        total_error = w_position * distance + w_rotation * rotation_error

        # 使用 Sigmoid 函数平滑过渡（sigmoid 函数会根据总误差平滑调节权重）
        sig_value = 1 / (1 + ca.exp(-3 * (total_error - error_threshold)))  # 控制平滑过渡
        based_sig_value=1 / (1 + ca.exp(-1* (based_error - error_threshold)))  # 控制平滑过渡
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
        d=sig_value_Q*0.5+(1-sig_value_Q)*0.05
        r=(1-sig_value_Q)*0.5+sig_value_Q*0.05

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



    def follow_adjust_matrix(self, current_ee_pose, target_ee_pose):
        base=10000000000
        manipulator=1
        self.R = ca.diagcat(
            base,
            base,
            base,
            manipulator,
            manipulator,
            manipulator,
            manipulator,
            manipulator,
            manipulator,
        )

        self.Q_task = ca.diagcat(10, 10 ,10, 5, 5, 5, 5)

        self.Q_terminal = ca.diagcat(500, 500, 500, 100, 100, 100, 100)  # 强化终端误差优化