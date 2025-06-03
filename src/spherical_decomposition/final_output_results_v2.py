# final_output_results.py
import os  # 确保在文件顶部引入 os 模块
import json
import numpy as np
from scipy.spatial.distance import cdist  # 用于计算两个点集之间的欧几里得距离

# 定义每个机器人连杆对应的RGBA颜色（用于可视化）
LINK_COLORS = {
    "base_link": [1, 0, 0, 0.8],
    "link1": [0, 1, 0, 0.8],
    "link2": [0, 0, 1, 0.8],
    "link3": [1, 1, 0, 0.8],
    "link4": [1, 0, 1, 0.8],
    "link5": [0, 1, 1, 0.8],
    "link6": [0.5, 0.5, 0.5, 0.8],
    "link_hand": [1, 0.5, 0, 0.8],
    "link_right": [0.2, 0.8, 0.3, 0.8],
    "link_left": [0.7, 0.3, 0.9, 0.8]
}

# 忽略的连杆对（这些连杆之间不需要检测最小距离）
IGNORE_PAIRS = {
    ("base_link", "link1"), ("link1", "base_link"),
    ("link1", "link2"), ("link2", "link1"),
    ("link1", "link3"), ("link3", "link1"),
    ("link2", "link3"), ("link3", "link2"),
    ("link3", "link4"), ("link4", "link3"),
    ("link3", "link5"), ("link5", "link3"),
    ("link3", "link6"), ("link6", "link3"),
    ("link4", "link5"), ("link5", "link4"),
    ("link4", "link6"), ("link6", "link4"),
    ("link5", "link6"), ("link6", "link5"),
    ("link5", "link_hand"), ("link_hand", "link5"),
    ("link5", "link_left"), ("link_left", "link5"),
    ("link5", "link_right"), ("link_right", "link5"),
    ("link6", "link_hand"), ("link_hand", "link6"),
    ("link6", "link_left"), ("link_left", "link6"),
    ("link6", "link_right"), ("link_right", "link6"),
    ("link_hand", "link_left"), ("link_left", "link_hand"),
    ("link_hand", "link_right"), ("link_right", "link_hand"),
    ("link_left", "link_right"), ("link_right", "link_left")
}


# 欧拉角 -> 旋转矩阵
def euler_to_rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # 生成3x3旋转矩阵
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])


# 计算每个连杆的变换矩阵（从base到该连杆的世界坐标位姿）
def compute_transforms(joint_angles):
    T = {}

    # 构造4x4齐次变换矩阵
    def make_transform(rpy, xyz):
        R = euler_to_rotation_matrix(rpy)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T

    # 逐级连接机器人各个连杆
    base_x, base_y, base_theta = joint_angles[0:3]
    T["base_link"] = make_transform([0, 0, base_theta], [base_x, base_y, 0])
    T["obstacle"] = make_transform([0, 0, 0], [0, 0, 0])
    T["link1"] = T["base_link"] @ make_transform([0, 0, joint_angles[3]], [0, 0, 0.1284])
    T["link2"] = T["link1"] @ make_transform([1.5708, -joint_angles[4], 1.5708], [0, 0, 0.0927])
    T["link3"] = T["link2"] @ make_transform([-3.1416, 0, -1.5708 + joint_angles[5]], [0, 0.22, 0])
    T["link4"] = T["link3"] @ make_transform([1.5708, -joint_angles[6], 0], [0, 0, 0])
    T["link5"] = T["link4"] @ make_transform([1.5708, -1.5708 + joint_angles[7], 0], [0, 0, 0.1685])
    T["link6"] = T["link5"] @ make_transform([1.5708, -joint_angles[8], 0], [0, 0, 0])
    T["link_hand"] = T["link6"] @ make_transform([0, 0, 0], [0, 0, 0])

    # 手爪夹角决定左右指尖之间的间距
    g = joint_angles[9]
    T["link_right"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, -g/2, 0.1465])
    T["link_left"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, g/2, 0.1465])

    return T


# 计算所有有效连杆对之间的最小球心距离及其相关位姿变换
def compute_min_distances(transforms, rel_spheres):
    world_positions = {}
    sphere_transforms = {}
    sphere_to_link_transforms = {}

    # 将局部球心坐标变换到世界坐标，并记录变换信息
    for link, spheres in rel_spheres.items():
        if link not in transforms:
            continue
        T_link = transforms[link]
        R_link = T_link[:3, :3]
        world_positions[link] = []
        sphere_transforms[link] = []
        sphere_to_link_transforms[link] = []

        for pos in spheres:
            local = np.array(pos + [1.0])  # 齐次坐标
            world = T_link @ local  # 世界坐标
            world_positions[link].append(world[:3])

            T_sphere = np.eye(4)
            T_sphere[:3, :3] = R_link
            T_sphere[:3, 3] = world[:3]
            sphere_transforms[link].append(T_sphere)

            T_sphere_to_link = np.linalg.inv(T_link) @ T_sphere
            sphere_to_link_transforms[link].append(T_sphere_to_link)

        world_positions[link] = np.array(world_positions[link])

    min_distances = []
    closest_pairs = []

    # 计算所有未忽略的连杆对之间的最小球心距离
    links = list(world_positions.keys())
    for i in range(len(links)):
        for j in range(i + 1, len(links)):
            a, b = links[i], links[j]
            if (a, b) in IGNORE_PAIRS:
                continue
            pos_a, pos_b = world_positions[a], world_positions[b]
            dists = cdist(pos_a, pos_b)  # 所有球之间的距离矩阵
            min_idx = np.unravel_index(np.argmin(dists), dists.shape)
            min_d = dists[min_idx]  # 最小距离
            closest_a = pos_a[min_idx[0]]
            closest_b = pos_b[min_idx[1]]
            closest_pairs.append((closest_a, closest_b))

            T_a_to_link = sphere_to_link_transforms[a][min_idx[0]]
            T_b_to_link = sphere_to_link_transforms[b][min_idx[1]]

            min_distances.append((a, b, min_d, T_a_to_link, T_b_to_link))

    return min_distances, closest_pairs

# 主流程函数，执行整套距离计算和结果输出
def run_pipeline(joint_angles, sphere_json_path="link_spheres_relative_v2.json", output_path="output_results.json"):
    # 如果传入的是相对路径，则转为相对于当前模块的路径
    if not os.path.isabs(sphere_json_path):
        base_dir = os.path.dirname(__file__)  # 当前 py 文件的目录
        sphere_json_path = os.path.join(base_dir, sphere_json_path)

    with open(sphere_json_path, "r") as f:
        data = json.load(f)  # 加载球心位置数据（相对坐标）
    transforms = compute_transforms(joint_angles)  # 计算每个连杆的世界变换
    min_distances, closest_pairs = compute_min_distances(transforms, data)  # 执行距离计算
    return min_distances, closest_pairs  # 返回数据用于后续处理
