import pybullet as p
import pybullet_data
import time
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
    for link, spheres in rel_spheres.items():
        if link not in transforms: continue
        T = transforms[link]
        world_positions[link] = []
        for pos in spheres:
            local = np.array(pos + [1])
            world = T @ local
            world_positions[link].append(world[:3])
        world_positions[link] = np.array(world_positions[link])

    min_distances = []
    closest_pairs = []  # 新增：记录最近的小球位置对（用于画黑色）

    links = list(world_positions.keys())
    for i in range(len(links)):
        for j in range(i+1, len(links)):
            a, b = links[i], links[j]
            if (a, b) in IGNORE_PAIRS or (b, a) in IGNORE_PAIRS:
                continue

            pos_a, pos_b = world_positions[a], world_positions[b]

            dists = cdist(pos_a, pos_b)
            min_idx = np.unravel_index(np.argmin(dists), dists.shape)
            min_d = dists[min_idx]

            # 保存最近小球对的位置
            closest_a = pos_a[min_idx[0]]
            closest_b = pos_b[min_idx[1]]
            closest_pairs.append((closest_a, closest_b))

            min_distances.append((a, b, min_d))

    return min_distances, closest_pairs

# 输出并可视化所有可能的最近小球
def visualize_relative_spheres(json_file, joint_angles):
    with open(json_file, "r") as f:
        data = json.load(f)
    transforms = compute_transforms(joint_angles)
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    radius = 0.01

    for link_name, rel_positions in data.items():
        if link_name not in transforms:
            print(f"跳过未定义变换的 link: {link_name}")
            continue
        color = LINK_COLORS.get(link_name, [1, 1, 1, 0.6])
        T = transforms[link_name]
        for pos in rel_positions:
            local = np.array(pos + [1.0])
            world = T @ local
            world_pos = world[:3].tolist()
            vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
            p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs_id, basePosition=world_pos)

    #--- 新增部分：画最小距离的小球对 ---
    min_dists, closest_pairs = compute_min_distances(transforms, data)
    for pos1, pos2 in closest_pairs:
        for pos in [pos1, pos2]:
            vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius * 1.2, rgbaColor=[0, 0, 0, 1])
            p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs_id, basePosition=pos.tolist())
            # --- 用黑色线连接最小距离的小球对 ---
            for pos1, pos2 in closest_pairs:
                p.addUserDebugLine(pos1.tolist(), pos2.tolist(), lineColorRGB=[0, 0, 0], lineWidth=2.0)

    print("最小距离统计（忽略邻接 link）：")
    for a, b, d in sorted(min_dists, key=lambda x: x[2]):
        print(f"{a} 与 {b} 的最小距离为 {d:.4f} 米")
    print("可视化完成。按 Ctrl+C 退出。")
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)





if __name__ == "__main__":
    joint_angles = get_joint_angles_from_input()
    visualize_relative_spheres("link_spheres_relative.json", joint_angles)
#输出顺序为
# ("link2", "link4"),
# ("link4", "link_hand"),
# ("base_link", "link2"),
# ("base_link", "link3"),
# ("link2", "link5"),
# ("link4", "link_right"),
# ("link4", "link_left"),
# ("link3", "link_hand"),
# ("base_link", "link_hand"),
# ("base_link", "link_left"),
# ("link2", "link6"),
# ("link2", "link_hand"),
# ("base_link", "link4"),
# ("base_link", "link6"),
# ("link1", "link_hand"),
# ("link1", "link_left"),
# ("base_link", "link_right"),
# ("base_link", "link5"),
# ("link1", "link4"),
# ("link3", "link_left"),
# ("link1", "link6"),
# ("link2", "link_left"),
# ("link3", "link_right"),
# ("link1", "link5"),
# ("link1", "link_right"),
# ("link2", "link_right"),
