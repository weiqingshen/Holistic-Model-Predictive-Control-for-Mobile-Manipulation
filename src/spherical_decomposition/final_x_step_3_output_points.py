
import json
import numpy as np
from scipy.spatial.distance import cdist

# 每个 link 使用不同颜色
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

def euler_to_rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])

def compute_transforms(joint_angles):
    T = {}
    def make_transform(rpy, xyz):
        R = euler_to_rotation_matrix(rpy)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T

    # 处理 base_link 的初始位姿（用 x, y, theta）
    base_x, base_y, base_theta = joint_angles[0], joint_angles[1], joint_angles[2]
    T["base_link"] = make_transform([0, 0, base_theta], [base_x, base_y, 0])

    # 关节角度对应调整
    T["link1"] = T["base_link"] @ make_transform([0, 0, joint_angles[3]], [0, 0, 0.1284])
    T["link2"] = T["link1"] @ make_transform([1.5708, -joint_angles[4], 1.5708], [0, 0, 0.0927])
    T["link3"] = T["link2"] @ make_transform([-3.1416, 0, -1.5708 + joint_angles[5]], [0, 0.22, 0])
    T["link4"] = T["link3"] @ make_transform([1.5708, -joint_angles[6], 0], [0, 0, 0])
    T["link5"] = T["link4"] @ make_transform([1.5708, -1.5708 + joint_angles[7], 0], [0, 0, 0.1685])
    T["link6"] = T["link5"] @ make_transform([1.5708, -joint_angles[8], 0], [0, 0, 0])
    T["link_hand"] = T["link6"] @ make_transform([0, 0, 0], [0, 0, 0])
    # 加上夹爪的平移，joint_angles[9]是平移量
    gripper_translation = joint_angles[9]
    T["link_right"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, -gripper_translation / 2, 0.1465])
    T["link_left"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, gripper_translation / 2, 0.1465])

    return T

def get_joint_angles_from_input():
    print("请输入10个数，前三个代表base_link的位姿（x，y，theta），中间六个是六个link的角度，最后一个代表夹爪的平移位移，用空格分隔：")
    while True:
        try:
            angles = list(map(float, input(">>> ").strip().split()))
            if len(angles) != 10:
                raise ValueError("需要输入10个数值")
            return angles
        except Exception as e:
            print(f"输入有误：{e}，请重新输入。")


def compute_min_distances(transforms, rel_spheres):
    world_positions = {}
    sphere_transforms = {}  # 保存每个小球对应的完整变换矩阵
    sphere_to_link_transforms = {}  # 保存每个小球到对应 link 的变换矩阵

    for link, spheres in rel_spheres.items():
        if link not in transforms:
            continue
        T_link = transforms[link]
        R_link = T_link[:3, :3]  # link的旋转
        world_positions[link] = []
        sphere_transforms[link] = []
        sphere_to_link_transforms[link] = []

        for pos in spheres:
            local = np.array(pos + [1.0])  # 把局部坐标转换为齐次坐标
            world = T_link @ local  # 将局部坐标转换为世界坐标
            world_positions[link].append(world[:3])

            # 小球自己的完整变换矩阵（继承link的朝向，位置是球的世界位置）
            T_sphere = np.eye(4)
            T_sphere[:3, :3] = R_link  # 使用link的旋转
            T_sphere[:3, 3] = world[:3]  # 使用小球的世界位置
            sphere_transforms[link].append(T_sphere)

            # 计算小球到对应link的变换矩阵
            T_sphere_to_link = np.linalg.inv(T_link) @ T_sphere
            sphere_to_link_transforms[link].append(T_sphere_to_link)

        world_positions[link] = np.array(world_positions[link])

    min_distances = []
    closest_pairs = []

    links = list(world_positions.keys())
    for i in range(len(links)):
        for j in range(i + 1, len(links)):
            a, b = links[i], links[j]
            if (a, b) in IGNORE_PAIRS or (b, a) in IGNORE_PAIRS:
                continue

            pos_a, pos_b = world_positions[a], world_positions[b]

            dists = cdist(pos_a, pos_b)  # 计算两个链接的小球之间的欧几里得距离
            min_idx = np.unravel_index(np.argmin(dists), dists.shape)  # 找到最小距离的索引
            min_d = dists[min_idx]  # 最小距离

            # 保存最近小球对的位置
            closest_a = pos_a[min_idx[0]]
            closest_b = pos_b[min_idx[1]]
            closest_pairs.append((closest_a, closest_b))



            # 保存小球到对应 link 的变换矩阵
            T_a_to_link = sphere_to_link_transforms[a][min_idx[0]]
            T_b_to_link = sphere_to_link_transforms[b][min_idx[1]]

            min_distances.append((a, b, min_d, T_a_to_link, T_b_to_link))

    return min_distances, closest_pairs


def save_results(min_distances, closest_pairs):
    # 转换为 Python 原生类型（例如 list）
    min_distances_serializable = [
        (a, b, float(min_d), T_a_to_link.tolist(), T_b_to_link.tolist())
        for a, b, min_d, T_a_to_link, T_b_to_link in min_distances
    ]
    closest_pairs_serializable = [
        (list(closest_a), list(closest_b)) for closest_a, closest_b in closest_pairs
    ]

    output_data = {
        "min_distances": min_distances_serializable,
        "closest_pairs": closest_pairs_serializable
    }

    with open("output_results.json", "w") as f:
        json.dump(output_data, f, indent=4)


if __name__ == "__main__":
    joint_angles = get_joint_angles_from_input()
    with open("link_spheres_relative_final_1.json", "r") as f:
        data = json.load(f)
    transforms = compute_transforms(joint_angles)
    min_distances, closest_pairs = compute_min_distances(transforms, data)
    save_results(min_distances, closest_pairs)
    print("最小距离计算完毕，结果已保存到 output_results.json 文件")
