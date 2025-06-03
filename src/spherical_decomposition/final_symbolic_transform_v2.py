import json
import sympy as sp

# 定义符号变量
x, y, theta = sp.symbols('x y theta')
q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
g = sp.symbols('g')
joint_symbols = [x, y, theta, q1, q2, q3, q4, q5, q6, g]

# 欧拉角转换为旋转矩阵（符号）
def euler_to_rotation_matrix_sym(rpy):
    roll, pitch, yaw = rpy
    cr, sr = sp.cos(roll), sp.sin(roll)
    cp, sp_ = sp.cos(pitch), sp.sin(pitch)
    cy, sy = sp.cos(yaw), sp.sin(yaw)
    return sp.Matrix([
        [cy*cp, cy*sp_*sr - sy*cr, cy*sp_*cr + sy*sr],
        [sy*cp, sy*sp_*sr + cy*cr, sy*sp_*cr - cy*sr],
        [-sp_, cp*sr, cp*cr]
    ])

# 构建4x4变换矩阵（符号）
def make_transform_sym(rpy, xyz):
    R = euler_to_rotation_matrix_sym(rpy)
    T = sp.eye(4)
    T[:3, :3] = R
    T[:3, 3] = sp.Matrix(xyz)
    return T

# 构建所有link的变换链（符号）
def compute_transforms_symbolic():
    T = {}
    T["base_link"] = make_transform_sym([0, 0, theta], [x, y, 0])
    T["obstacle"] = make_transform_sym([0, 0, 0], [0, 0, 0])
    T["link1"] = T["base_link"] @ make_transform_sym([0, 0, q1], [0, 0, 0.1284])
    T["link2"] = T["link1"] @ make_transform_sym([sp.pi/2, -q2, sp.pi/2], [0, 0, 0.0927])
    T["link3"] = T["link2"] @ make_transform_sym([-sp.pi, 0, -sp.pi/2 + q3], [0, 0.22, 0])
    T["link4"] = T["link3"] @ make_transform_sym([sp.pi/2, -q4, 0], [0, 0, 0])
    T["link5"] = T["link4"] @ make_transform_sym([sp.pi/2, -sp.pi/2 + q5, 0], [0, 0, 0.1685])
    T["link6"] = T["link5"] @ make_transform_sym([sp.pi/2, -q6, 0], [0, 0, 0])
    T["link_hand"] = T["link6"] @ make_transform_sym([0, 0, 0], [0, 0, 0])
    T["link_right"] = T["link6"] @ make_transform_sym([-sp.pi/2, 0, 0], [0, -g/2, 0.1465])
    T["link_left"]  = T["link6"] @ make_transform_sym([-sp.pi/2, 0, 0], [0, g/2, 0.1465])
    return T

# 小球变换（符号）
def transform_sphere_symbolic(T_link, T_sphere_to_link):
    return T_link @ sp.Matrix(T_sphere_to_link)

def     get_symbolic_transforms_of_closest_spheres(min_distances_data):
    transforms = compute_transforms_symbolic()
    output = []

    for item in min_distances_data:
        link_a, link_b, min_dist_value, T_a_to_link, T_b_to_link = item

        T_a_world = transform_sphere_symbolic(transforms[link_a], T_a_to_link)
        T_b_world = transform_sphere_symbolic(transforms[link_b], T_b_to_link)

        pos_a = T_a_world[:3, 3]
        pos_b = T_b_world[:3, 3]

        dist_expr = sp.sqrt(
            (pos_a[0] - pos_b[0])**2 +
            (pos_a[1] - pos_b[1])**2 +
            (pos_a[2] - pos_b[2])**2
        )

        output.append([
            [link_a, link_b],
            min_dist_value,
            dist_expr
        ])

    return output


# 如果直接运行这个文件，可以输出所有结果
if __name__ == "__main__":
    result = get_symbolic_transforms_of_closest_spheres("output_results.json")
    for pair, dist_value, dist_expr in result:
        print(f"{pair}: 最短值 = {dist_value}, 符号距离 = {dist_expr}")
