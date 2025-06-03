import json
import sympy as sp

# 定义符号变量
x, y, theta = sp.symbols('x y theta')
q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
g = sp.symbols('g')  # gripper translation
joint_symbols = [x, y, theta, q1, q2, q3, q4, q5, q6, g]

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

def make_transform_sym(rpy, xyz):
    R = euler_to_rotation_matrix_sym(rpy)
    T = sp.eye(4)
    T[:3, :3] = R
    T[:3, 3] = sp.Matrix(xyz)
    return T

def compute_transforms_symbolic():
    T = {}
    T["base_link"] = make_transform_sym([0, 0, theta], [x, y, 0])
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

def transform_sphere_symbolic(T_link, T_sphere_to_link):
    return T_link @ sp.Matrix(T_sphere_to_link)

if __name__ == "__main__":
    with open("output_results.json", "r") as f:
        saved_data = json.load(f)

    transforms = compute_transforms_symbolic()
    output = []

    for item in saved_data["min_distances"]:
        link_a, link_b, _, T_a_to_link, T_b_to_link = item
        T_a_world = transform_sphere_symbolic(transforms[link_a], T_a_to_link)
        T_b_world = transform_sphere_symbolic(transforms[link_b], T_b_to_link)

        pos_a = T_a_world[:3, 3]
        pos_b = T_b_world[:3, 3]

        output.append({
            "link_a": link_a,
            "link_b": link_b,
            "sphere_a_world_position": [str(sp.simplify(c)) for c in pos_a],
            "sphere_b_world_position": [str(sp.simplify(c)) for c in pos_b]
        })

    with open("symbolic_positions.json", "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)

    print("小球的符号位姿已写入文件：symbolic_positions.json")
