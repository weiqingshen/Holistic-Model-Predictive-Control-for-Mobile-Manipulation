import casadi as ca
import numpy as np


class MPCPlanner:
    def __init__(self, logger):
        """初始化 MPC 参数"""
        self.logger = logger
        self.logger.info("初始化 MPC 规划器")

        # 状态权重矩阵 Q (9x9)
        self.Q = ca.diagcat(10, 10, 5, 1, 1, 1, 1, 1, 1)  # 底盘权重
        # 控制权重矩阵 R (9x9)
        self.R = ca.diagcat(1, 1, 1, 1, 1, 1, 1, 1, 1)
        # 机械臂关节误差权重
        self.Qa = ca.diagcat(50, 50, 50, 50, 50, 50)  # 机械臂关节误差权重

        # 预测步长
        self.step_horizon = 0.1
        self.N = 30  # 预测窗口

        # 状态和控制变量维度
        self.n_states = 9
        self.n_controls = 9

        # 速度约束
        self.v_max = 0.2
        self.v_min = -0.2
        self.v_arm_max = 0.1
        self.v_arm_min = -0.1

        # 生成 MPC 相关变量
        self.create_symbolic_variables()
        self.build_cost_function()
        self.set_limit()
        self.set_optimize_option()

    def create_symbolic_variables(self):
        """定义符号变量"""
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)  # 预测状态
        self.U = ca.SX.sym('U', self.n_controls, self.N)  # 预测控制输入

        # 目标状态 `P`，包含 `2*n_states` (初始 + 目标) 和 `7*(N+1)` (机械臂目标轨迹)
        self.P = ca.SX.sym('P', 2 * self.n_states + 6 * (self.N + 1))
        self.P_arm = ca.reshape(self.P[2 * self.n_states:], 6, self.N + 1)  # 机械臂轨迹

        # 运动学模型
        x_t = ca.SX.sym('x_t', self.n_states)
        u_t = ca.SX.sym('u_t', self.n_controls)
        theta_t = x_t[2]
        f_x = u_t[0] * ca.cos(theta_t) - u_t[1] * ca.sin(theta_t)
        f_y = u_t[0] * ca.sin(theta_t) + u_t[1] * ca.cos(theta_t)
        f_theta = u_t[2]
        q_dot_t = u_t[3:]

        self.f = ca.Function('f', [x_t, u_t], [ca.vertcat(f_x, f_y, f_theta, q_dot_t)])

    def build_cost_function(self):
        """构建 MPC 目标函数"""
        self.logger.info("构建代价函数和约束")
        self.cost_fn = 0
        constraints = [self.X[:, 0] - self.P[:self.n_states]]  # 初始状态约束

        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]

            # 状态误差（目标函数）
            state_error = st - self.P[self.n_states:2 * self.n_states]
            self.cost_fn += ca.mtimes(state_error.T, self.Q @ state_error) + ca.mtimes(con.T, self.R @ con)

            # 机械臂关节误差（目标函数）
            q_arm_error = st[3:] - self.P_arm[:, k]
            self.cost_fn += q_arm_error.T @ self.Qa @ q_arm_error

            # 状态转移约束（等式约束）
            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_pred = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            constraints.append(st_next - st_next_pred)

        # 合并所有约束
        self.g = ca.vertcat(*constraints)
        self.logger.info(f"优化约束 g 维度: {self.g.shape}")  # 应为 (9 + 30*9, 1) = (279, 1)

        # 新增调试信息
        self.logger.debug(f"初始状态约束示例: {constraints[0]}")
        self.logger.debug(f"第 0 步状态转移约束示例: {constraints[1]}")
        self.logger.debug(f"总约束数量: {len(constraints)} (预期: {self.N + 1})")

    def set_limit(self):
        """定义变量约束"""
        self.logger.info("设置变量约束")

        num_constraints = self.g.shape[0]
        lbx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N, 1))
        ubx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N, 1))

        # 新增调试信息：打印变量总数
        total_vars = self.n_states * (self.N + 1) + self.n_controls * self.N
        self.logger.info(f"总变量数量: {total_vars} (预期: {9 * 31 + 9 * 30} = 549)")

        # 设置控制输入的上下限
        lbx[self.n_states * (self.N + 1):] = self.v_arm_min
        ubx[self.n_states * (self.N + 1):] = self.v_arm_max

        lbx[self.n_states * (self.N + 1)::self.n_controls] = self.v_min
        ubx[self.n_states * (self.N + 1)::self.n_controls] = self.v_max

        # 新增调试信息：打印约束和变量维度
        self.logger.info(f"等式约束维度 (lbg/ubg): {num_constraints} (预期: 279)")
        self.logger.info(f"变量上下限维度 (lbx/ubx): {lbx.shape[0]} (预期: 549)")

        self.args = {
            'lbg': ca.DM.zeros(num_constraints, 1),
            'ubg': ca.DM.zeros(num_constraints, 1),
            'lbx': lbx,
            'ubx': ubx
        }

    def set_optimize_option(self):
        """优化求解器设置"""
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
                'print_level': 5,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-6,
                'max_cpu_time': 5
            },
            'print_time': 1
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def set_reference(self, p, p_arm):
        """设定参考轨迹 (包括底盘和机械臂)"""
        self.args['p'] = ca.vertcat(p, ca.reshape(p_arm, 6 * (self.N + 1), 1))

    def set_x0(self, X0, u0):
        """设置优化初始值"""
        self.args['x0'] = ca.vertcat(
            ca.reshape(X0, self.n_states * (self.N + 1), 1),
            ca.reshape(u0, self.n_controls * self.N, 1)
        )

    def get_states_and_control(self):
        sol = self.solver(**self.args)
        if not sol['success']:  # 检查求解是否成功
            self.logger.error("求解失败，未找到可行解！")
            return None
        X_opt = ca.reshape(sol['x'][:self.n_states * (self.N + 1)], self.n_states, self.N + 1)
        return X_opt