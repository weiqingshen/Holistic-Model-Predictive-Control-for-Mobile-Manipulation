import json
import numpy as np

def euler_to_rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return R_z @ R_y @ R_x

def compute_transforms():
    T = {}
    T["base_link"] = np.eye(4)

    def transform(rpy, xyz):
        R = euler_to_rotation_matrix(rpy)
        T_link = np.eye(4)
        T_link[:3, :3] = R
        T_link[:3, 3] = xyz
        return T_link

    T["link1"] = T["base_link"] @ transform([0, 0, 0], [0, 0, 0.1284])
    T["link2"] = T["link1"] @ transform([1.5708, 0, 1.5708], [0, 0, 0.0927])
    T["link3"] = T["link2"] @ transform([-3.1416, 0, -1.5708], [0, 0.22, 0])
    T["link4"] = T["link3"] @ transform([1.5708, 0, 0], [0, 0, 0])
    T["link5"] = T["link4"] @ transform([1.5708, -1.5708, 0], [0, 0, 0.1685])
    T["link6"] = T["link5"] @ transform([1.5708, 0, 0], [0, 0, 0])
    T["link_hand"] = T["link6"] @ transform([0, 0, 0], [0, 0, 0])
    T["link_right"] = T["link_hand"] @ transform([-1.5708, 0, 0], [0, 0, 0.1465])
    T["link_left"] = T["link_hand"] @ transform([-1.5708, 0, 0], [0, 0, 0.1465])

    return T

def convert_absolute_to_relative(input_path, output_path):
    with open(input_path, "r") as f:
        data = json.load(f)

    transforms = compute_transforms()
    relative_data = {}

    for link, abs_positions in data.items():
        if link not in transforms:
            print(f"❗️ 警告：{link} 没有对应的变换矩阵，跳过")
            continue

        T_link_inv = np.linalg.inv(transforms[link])
        relative_positions = []

        for pos in abs_positions:
            abs_pos_hom = np.array(pos + [1.0])  # 转为齐次坐标
            rel_pos = T_link_inv @ abs_pos_hom
            relative_positions.append([float(x) for x in rel_pos[:3]])

        relative_data[link] = relative_positions

    with open(output_path, "w") as f:
        json.dump(relative_data, f, indent=2)

    print(f"✅ 相对坐标已保存至 {output_path}")

# 执行转换
convert_absolute_to_relative("link_spheres.json", "link_spheres_relative.json")
