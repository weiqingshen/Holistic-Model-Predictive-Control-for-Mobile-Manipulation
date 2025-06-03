import casadi as ca
import numpy as np
from fk_generator import FKGenerator  # 依然基于之前写好的

class MPCPlannerV7:
    def __init__(self, logger, urdf_path):
        self.logger = logger
        self.logger.info("初始化 MPC 规划器 v6 完整版")

        self.n_states = 9  # base 3 + arm 6
        self.n_controls = 9
        self.N = 30
        self.step_horizon = 0.1

        # 权重矩阵：位置 + 姿态（rpy）
        self.Q_task = ca.diagcat(10, 10, 10, 5, 5, 5, 5)

        self.R = ca.diagcat(1, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

        self.v_min = -0.2
        self.v_max = 0.2
        self.v_arm_min = -0.1
        self.v_arm_max = 0.1

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
        self.cost_fn = 0
        constraints = [self.X[:, 0] - self.P[7:]]  # 当前状态

        target_ee_pose = self.P[0:7]  # xyz + rpy

        for k in range(self.N):
            st = self.X[:, k]
            print(st)
            con = self.U[:, k]

            q_total = st[0:9]  # base 3 + arm 6
            ee_pose = self.fk_func(q_total)

            pose_error = ee_pose - target_ee_pose  # xyz + rpy
            self.cost_fn += ca.mtimes(pose_error.T, self.Q_task @ pose_error)
            self.cost_fn += ca.mtimes(con.T, self.R @ con)

            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_pred = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

            constraints.append(st_next - st_next_pred)

        self.g = ca.vertcat(*constraints)

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
                'print_level': 5,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-6,
                'max_cpu_time': 5
            },
            'print_time': 0
        }

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
