import casadi as ca

# 欧拉角转换为旋转矩阵（CasADi）
def euler_to_rotation_matrix_sym(rpy):
    roll, pitch, yaw = rpy
    cr, sr = ca.cos(roll), ca.sin(roll)
    cp, sp_ = ca.cos(pitch), ca.sin(pitch)
    cy, sy = ca.cos(yaw), ca.sin(yaw)

    R = ca.SX(3, 3)
    R[0, 0] = cy * cp
    R[0, 1] = cy * sp_ * sr - sy * cr
    R[0, 2] = cy * sp_ * cr + sy * sr
    R[1, 0] = sy * cp
    R[1, 1] = sy * sp_ * sr + cy * cr
    R[1, 2] = sy * sp_ * cr - cy * sr
    R[2, 0] = -sp_
    R[2, 1] = cp * sr
    R[2, 2] = cp * cr
    return R

# 构建4x4变换矩阵（CasADi）
def make_transform_sym(rpy, xyz):
    R = euler_to_rotation_matrix_sym(rpy)
    T = ca.SX(4, 4)
    T[0:3, 0:3] = R
    T[0:3, 3] = ca.vertcat(*xyz)
    T[3, 0:3] = ca.SX.zeros(1, 3)
    T[3, 3] = 1
    return T

# 单个时间步的变换链（CasADi）
def compute_transforms_symbolic_one_step(X_t):
    x, y, theta = X_t[0], X_t[1], X_t[2]
    q1, q2, q3, q4, q5, q6 = X_t[3], X_t[4], X_t[5], X_t[6], X_t[7], X_t[8]
    g = 0.001

    T = {}
    T["base_link"] = make_transform_sym([0, 0, theta], [x, y, 0])
    T["obstacle"] = make_transform_sym([0, 0, 0], [0, 0, 0])
    T["link1"] = T["base_link"] @ make_transform_sym([0, 0, q1], [0, 0, 0.1284])
    T["link2"] = T["link1"] @ make_transform_sym([ca.pi/2, -q2, ca.pi/2], [0, 0, 0.0927])
    T["link3"] = T["link2"] @ make_transform_sym([-ca.pi, 0, -ca.pi/2 + q3], [0, 0.22, 0])
    T["link4"] = T["link3"] @ make_transform_sym([ca.pi/2, -q4, 0], [0, 0, 0])
    T["link5"] = T["link4"] @ make_transform_sym([ca.pi/2, -ca.pi/2 + q5, 0], [0, 0, 0.1685])
    T["link6"] = T["link5"] @ make_transform_sym([ca.pi/2, -q6, 0], [0, 0, 0])
    T["link_hand"] = T["link6"] @ make_transform_sym([0, 0, 0], [0, 0, 0])
    T["link_right"] = T["link6"] @ make_transform_sym([-ca.pi/2, 0, 0], [0, -g/2, 0.1465])
    T["link_left"]  = T["link6"] @ make_transform_sym([-ca.pi/2, 0, 0], [0, g/2, 0.1465])
    return T

# 所有时间步的变换链
def compute_transforms_all_timesteps(X):
    num_steps = X.shape[1]
    all_T = []

    for t in range(num_steps):
        X_t = X[:, t]
        T_t = compute_transforms_symbolic_one_step(X_t)
        all_T.append(T_t)

    return all_T

# 小球变换（CasADi）
def transform_sphere_symbolic(T_link, T_sphere_to_link):
    return T_link @ ca.SX(T_sphere_to_link)

# 针对多个时间步计算距离表达式
def get_symbolic_transforms_of_closest_spheres(min_distances_data, X):
    all_transforms = compute_transforms_all_timesteps(X)
    output = []

    for t_idx, transforms in enumerate(all_transforms):
        for item in min_distances_data:
            link_a, link_b, dist_val, T_a_to_link, T_b_to_link = item

            T_a_world = transform_sphere_symbolic(transforms[link_a], T_a_to_link)
            T_b_world = transform_sphere_symbolic(transforms[link_b], T_b_to_link)

            pos_a = T_a_world[0:3, 3]
            pos_b = T_b_world[0:3, 3]

            dist_expr = ca.sqrt(
                (pos_a[0] - pos_b[0])**2 +
                (pos_a[1] - pos_b[1])**2 +
                (pos_a[2] - pos_b[2])**2
            )

            output.append([
                [link_a, link_b],
                dist_val,
                dist_expr,
                t_idx  # 添加时间索引便于追踪
            ])

    return output
