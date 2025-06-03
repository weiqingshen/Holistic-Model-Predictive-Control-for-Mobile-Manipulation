import casadi as ca
import numpy as np

class MPCPlanner:
    def __init__(self):
        """初始化 MPC 参数"""
        # 状态权重矩阵 (Q_X, Q_Y, Q_THETA, q1, ..., q6)
        self.Q = ca.diagcat(1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        # 控制权重矩阵 (V, W, q1_dot, ..., q6_dot)
        self.R = ca.diagcat(1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        # 时间步长 & 预测步数
        self.step_horizon = 0.1
        self.N = 30

        # 机器人状态变量
        self.n_states = 9  # (x, y, theta, q1, ..., q6)
        self.n_controls = 8  # (v, w, q1_dot, ..., q6_dot)

        # 初始状态 & 目标状态
        self.state_init = ca.DM([0, 0, 0, -1.591, -1.096, 1.577, -0.293, -0.194, 1.735])
        self.state_target = ca.DM([0.5, -0.5, ca.pi / 4, 0, 0, 0, 0, 0, 0])

        # 最大速度约束
        self.v_max = 0.2
        self.v_min = -0.2
        self.v_arm_max = 0.1
        self.v_arm_min = -0.1

        # 创建优化变量
        self.create_symbolic_variables()
        self.build_cost_function()
        self.set_limit()
        self.set_optimize_option()

    def create_symbolic_variables(self):
        """定义符号变量"""
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)  # 状态变量
        self.U = ca.SX.sym('U', self.n_controls, self.N)  # 控制输入
        self.P = ca.SX.sym('P', 2 * self.n_states)  # 参数 (初始 & 目标状态)

        # 运动学方程
        theta = self.X[2, :]
        v = self.U[0, :]
        omega = self.U[1, :]
        q_dot = self.U[2:, :]

        f_x = v * ca.cos(theta)
        f_y = v * ca.sin(theta)
        f_theta = omega

        self.f = ca.Function('f', [self.X, self.U],
                             [ca.vertcat(f_x, f_y, f_theta, q_dot)])

    def build_cost_function(self):
        """构建 MPC 目标函数"""
        self.cost_fn = 0
        self.g = self.X[:, 0] - self.P[:self.n_states]  # 初始状态约束

        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            self.cost_fn += ca.mtimes((st - self.P[self.n_states:]).T, self.Q, (st - self.P[self.n_states:])) \
                          + ca.mtimes(con.T, self.R, con)

            # 预测下一步状态 (Euler 积分)
            st_next = self.X[:, k + 1]
            st_next_pred = st + self.step_horizon * self.f(st, con)
            self.g = ca.vertcat(self.g, st_next - st_next_pred)

    def set_optimize_option(self):
        """优化求解器设置"""
        OPT_variables = ca.vertcat(self.X.reshape((-1, 1)), self.U.reshape((-1, 1)))

        nlp_prob = {
            'f': self.cost_fn,
            'x': OPT_variables,
            'g': self.g,
            'p': self.P
        }

        opts = {
            'ipopt': {
                'max_iter': 500,
                'print_level': 0,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def set_limit(self):
        """定义变量约束"""
        lbx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N, 1))
        ubx = ca.DM.zeros((self.n_states * (self.N + 1) + self.n_controls * self.N, 1))

        lbx[:self.n_states * (self.N + 1): self.n_states] = -ca.inf
        lbx[1:self.n_states * (self.N + 1): self.n_states] = -ca.inf
        lbx[2:self.n_states * (self.N + 1): self.n_states] = -ca.inf

        ubx[:self.n_states * (self.N + 1): self.n_states] = ca.inf
        ubx[1:self.n_states * (self.N + 1): self.n_states] = ca.inf
        ubx[2:self.n_states * (self.N + 1): self.n_states] = ca.inf

        lbx[self.n_states * (self.N + 1):] = self.v_arm_min
        ubx[self.n_states * (self.N + 1):] = self.v_arm_max

        lbx[self.n_states * (self.N + 1)::self.n_controls] = self.v_min
        ubx[self.n_states * (self.N + 1)::self.n_controls] = self.v_max

        self.args = {
            'lbg': ca.DM.zeros((self.n_states * (self.N + 1), 1)),
            'ubg': ca.DM.zeros((self.n_states * (self.N + 1), 1)),
            'lbx': lbx,
            'ubx': ubx
        }

    def set_reference(self, p):
        """设定参考路径"""
        self.args['p'] = p

    def set_x0(self, X0, u0):
        """设置优化初始值"""
        self.args['x0'] = ca.vertcat(
            ca.reshape(X0, self.n_states * (self.N + 1), 1),
            ca.reshape(u0, self.n_controls * self.N, 1)
        )

    def get_states_and_control(self):
        """求解最优轨迹"""
        sol = self.solver(
            x0=self.args['x0'],
            lbx=self.args['lbx'],
            ubx=self.args['ubx'],
            lbg=self.args['lbg'],
            ubg=self.args['ubg'],
            p=self.args['p']
        )
        X_opt = ca.reshape(sol['x'][:self.n_states * (self.N + 1)], self.n_states, self.N + 1)
        return X_opt

