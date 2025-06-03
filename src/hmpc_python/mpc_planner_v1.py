import casadi as ca
import numpy as np


class MPCPlanner:
    def __init__(self, logger):
        """初始化 MPC 参数"""
        self.logger = logger
        self.logger.info("初始化 MPC 规划器")

        # **状态权重矩阵**
        self.Q = ca.diagcat(1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        # **控制权重矩阵（9x9）**
        self.R = ca.diagcat(1, 1, 1,  # vx, vy, omega
                            0.1, 0.1, 0.1, 0.1, 0.1, 0.1)  # 6 个关节速度

        self.step_horizon = 0.1
        self.N = 30

        self.n_states = 9
        self.n_controls = 9

        self.state_init = ca.DM([0, 0, 0, -1.591, -1.096, 1.577, -0.293, -0.194, 1.735])
        self.state_target = ca.DM([0.5, -0.5, ca.pi / 4, 0, 0, 0, 0, 0, 0])

        self.v_max = 0.2
        self.v_min = -0.2
        self.v_arm_max = 0.1
        self.v_arm_min = -0.1

        self.create_symbolic_variables()
        self.build_cost_function()
        self.set_limit()
        self.set_optimize_option()

    def create_symbolic_variables(self):
        """定义符号变量"""
        self.logger.info("创建符号变量")
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)  # 机器人状态变量 (9,31)
        self.U = ca.SX.sym('U', self.n_controls, self.N)  # 控制变量 (9,30)
        self.P = ca.SX.sym('P', 2 * self.n_states)  # 参考轨迹 (18,)

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

        self.logger.info("符号变量创建完成")

    def build_cost_function(self):
        """构建 MPC 目标函数"""
        self.logger.info("构建代价函数和约束")
        self.cost_fn = 0
        self.g = ca.SX.zeros((self.n_states * self.N, 1))  # 约束 (270,1)

        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]

            state_error = st - self.P[self.n_states:]

            self.cost_fn += ca.mtimes(state_error.T, ca.mtimes(self.Q, state_error)) \
                            + ca.mtimes(con.T, ca.mtimes(self.R, con))

            st_next = self.X[:, k + 1]
            st_next_pred = st + self.step_horizon * self.f(st, con)

            self.g[self.n_states * k:self.n_states * (k + 1)] = st_next - st_next_pred

        self.logger.info(f"优化约束 g 维度: {self.g.shape}")

    def set_optimize_option(self):
        """优化求解器设置"""
        self.logger.info("设置优化器参数")
        OPT_variables = ca.vertcat(self.X.reshape((-1, 1)), self.U.reshape((-1, 1)))
        print(f"🚨 OPT_variables 维度: {OPT_variables.shape}")

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

    def set_limit(self):
        """定义变量约束"""
        self.logger.info("设置变量约束")

        num_constraints = self.g.shape[0]

        lbx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N, 1))
        ubx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N, 1))

        lbx[self.n_states * (self.N + 1):] = self.v_arm_min
        ubx[self.n_states * (self.N + 1):] = self.v_arm_max

        lbx[self.n_states * (self.N + 1)::self.n_controls] = self.v_min
        ubx[self.n_states * (self.N + 1)::self.n_controls] = self.v_max

        self.args = {
            'lbg': ca.DM.zeros(num_constraints, 1),
            'ubg': ca.DM.zeros(num_constraints, 1),
            'lbx': lbx,
            'ubx': ubx
        }

        self.logger.info(f"lbg 维度: {self.args['lbg'].shape}, 期望 g 维度: {num_constraints}")

    def set_reference(self, p):
        """设定参考轨迹 (目标状态)"""
        self.logger.info(f"设定参考路径: {p}")

        if not isinstance(p, ca.DM) or p.shape[0] != self.P.shape[0]:
            self.logger.error("参考路径的维度错误！请检查目标状态的格式。")
            return

        self.args['p'] = p

    def set_x0(self, X0, u0):
        """设置优化初始值"""
        self.logger.info("设置优化初始值")

        if not isinstance(X0, ca.DM) or not isinstance(u0, ca.DM):
            self.logger.error("X0 或 u0 的数据格式错误，必须为 CasADi DM 类型")
            return

        print(f"🚨 X0 维度: {X0.shape}, u0 维度: {u0.shape}")

        self.args['x0'] = ca.vertcat(
            ca.reshape(X0, self.n_states * (self.N + 1), 1),
            ca.reshape(u0, self.n_controls * self.N, 1)
        )
        print(f"🚨 x0 维度: {self.args['x0'].shape}")

        self.logger.info(f"X0 形状: {X0.shape}, u0 形状: {u0.shape}")

    def get_states_and_control(self):
        """求解最优轨迹"""
        self.logger.info("开始求解 MPC 轨迹...")

        try:
            sol = self.solver(
                x0=self.args['x0'],
                lbx=self.args['lbx'],
                ubx=self.args['ubx'],
                lbg=self.args['lbg'],
                ubg=self.args['ubg'],
                p=self.args['p']
            )
            self.logger.info("MPC 求解完成！")

            X_opt = ca.reshape(sol['x'][:self.n_states * (self.N + 1)], self.n_states, self.N + 1)
            print(X_opt)
            return X_opt
        except Exception as e:
            self.logger.error(f"MPC 求解失败: {e}")
            return None
