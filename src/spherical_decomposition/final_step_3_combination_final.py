import pybullet as p
import pybullet_data
import time
import json
import numpy as np
from scipy.spatial.distance import cdist

# 颜色定义
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

# 忽略相邻 link 的距离检测
IGNORE_PAIRS = {
    ("base_link", "link1"), ("link1", "link2"), ("link2", "link3"),
    ("link3", "link4"), ("link3", "link5"), ("link3", "link6"),
    ("link4", "link5"), ("link4", "link6"), ("link5", "link6"),
    ("link5", "link_hand"), ("link5", "link_left"), ("link5", "link_right"),
    ("link6", "link_hand"), ("link6", "link_left"), ("link6", "link_right"),
    ("link_hand", "link_left"), ("link_hand", "link_right"),
    ("link_left", "link_right")
}

# 读取用户输入的关节角度
def get_joint_angles_from_input():
    print("请输入10个关节角度（单位：弧度），用空格分隔：")
    while True:
        try:
            angles = list(map(float, input(">>> ").strip().split()))
            if len(angles) != 10:
                raise ValueError("需要输入10个数值")
            return angles
        except Exception as e:
            print(f"输入有误：{e}，请重新输入。")

# 欧拉角转旋转矩阵
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

# 计算每个 link 的变换矩阵
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

    # 加上夹爪的平移，joint_angles[9]是平移量
    gripper_translation = joint_angles[9]
    T["link_right"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, -gripper_translation / 2, 0.1465])
    T["link_left"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, gripper_translation / 2, 0.1465])

    return T
# 主函数
def main():
    joint_angles = get_joint_angles_from_input()

    # 初始化 PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    # 加载 URDF 机器人
    robot_id = p.loadURDF("/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf", useFixedBase=True)

    # 关节映射
    joint_name_to_index = {
        'joint1': 3,
        'joint2': 4,
        'joint3': 5,
        'joint4': 6,
        'joint5': 7,
        'joint6': 8,
        'joint_hand': 9,
        'joint_right': 10,
        'joint_left': 11
    }
    mapping = [
        ('joint1', joint_angles[0]),
        ('joint2', joint_angles[1]),
        ('joint3', joint_angles[2]),
        ('joint4', joint_angles[3]),
        ('joint5', joint_angles[4]),
        ('joint6', joint_angles[5]),
        ('joint_hand', joint_angles[6]),
        ('joint_right', joint_angles[7]),
        ('joint_left', joint_angles[8])
    ]
    for joint_name, angle in mapping:
        joint_index = joint_name_to_index[joint_name]
        p.resetJointState(robot_id, joint_index, angle)

    # 可视化球体
    with open("link_spheres_relative.json", "r") as f:
        sphere_data = json.load(f)
    transforms = compute_transforms(joint_angles)
    radius = 0.01
    for link_name, rel_positions in sphere_data.items():
        if link_name not in transforms:
            print(f"跳过未定义变换的 link: {link_name}")
            continue
        T = transforms[link_name]
        color = LINK_COLORS.get(link_name, [1, 1, 1, 0.6])
        for pos in rel_positions:
            local = np.array(pos + [1.0])
            world = T @ local
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color),
                basePosition=world[:3].tolist()
            )

    print("仿真启动，包含机构模型与球体标记。按 Ctrl+C 退出。")

    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)

if __name__ == "__main__":
    main()
