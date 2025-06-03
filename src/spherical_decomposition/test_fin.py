import numpy as np
import pybullet as p
import json
import time

MAX_RADIUS = 0.02
# 预定义需要跳过计算的关节对（注意双向性）
disable_collisions = {
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

def transform_sphere_pos(T, s_local):
    s_local_homog = np.append(np.array(s_local), 1)
    s_world_homog = T.dot(s_local_homog)
    return s_world_homog[:3]

def euler_to_rotation_matrix(rpy):
    """根据rpy角度生成旋转矩阵"""
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
    return R_z.dot(R_y).dot(R_x)

def compute_transforms(joint_angles):
    T = {}

    # base_link的变换矩阵
    T["base_link"] = np.eye(4)

    # joint1的变换矩阵
    rpy1 = [0, 0, joint_angles[0]]
    xyz1 = [0, 0, 0.1284]
    R1 = euler_to_rotation_matrix(rpy1)
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T1[:3, 3] = xyz1
    T["joint1"] = T["base_link"].dot(T1)

    # link1的变换矩阵
    rpy2 = [0, 0, 0]
    xyz2 = [-0.000491, -0.000362, 0.044474]
    R2 = euler_to_rotation_matrix(rpy2)
    T2 = np.eye(4)
    T2[:3, :3] = R2
    T2[:3, 3] = xyz2
    T["link1"] = T["joint1"].dot(T2)

    # joint2的变换矩阵
    rpy3 = [1.5708, 0, 1.5708+joint_angles[1]]
    xyz3 = [0, 0, 0.0927]
    R3 = euler_to_rotation_matrix(rpy3)
    T3 = np.eye(4)
    T3[:3, :3] = R3
    T3[:3, 3] = xyz3
    T["joint2"] = T["link1"].dot(T3)

    # link2的变换矩阵
    rpy4 = [0, 0, 0]
    xyz4 = [0.006885, 0.097138, 0.013427]
    R4 = euler_to_rotation_matrix(rpy4)
    T4 = np.eye(4)
    T4[:3, :3] = R4
    T4[:3, 3] = xyz4
    T["link2"] = T["joint2"].dot(T4)

    # joint3的变换矩阵
    rpy5 = [ - 3.1416, 0, -1.5708+joint_angles[2]]
    xyz5 = [0, 0.22, 0]
    R5 = euler_to_rotation_matrix(rpy5)
    T5 = np.eye(4)
    T5[:3, :3] = R5
    T5[:3, 3] = xyz5
    T["joint3"] = T["link2"].dot(T5)

    # link3的变换矩阵
    rpy6 = [0, 0, 0]
    xyz6 = [-0.016272, -0.033930, 0.008989]
    R6 = euler_to_rotation_matrix(rpy6)
    T6 = np.eye(4)
    T6[:3, :3] = R6
    T6[:3, 3] = xyz6
    T["link3"] = T["joint3"].dot(T6)

    # joint4的变换矩阵
    rpy7 = [1.5708,0, 0+ joint_angles[3]]
    xyz7 = [0, 0, 0]
    R7 = euler_to_rotation_matrix(rpy7)
    T7 = np.eye(4)
    T7[:3, :3] = R7
    T7[:3, 3] = xyz7
    T["joint4"] = T["link3"].dot(T7)

    # link4的变换矩阵
    rpy8 = [0, 0, 0]
    xyz8 = [-3.1472e-05, 0.011420, 0.119063]
    R8 = euler_to_rotation_matrix(rpy8)
    T8 = np.eye(4)
    T8[:3, :3] = R8
    T8[:3, 3] = xyz8
    T["link4"] = T["joint4"].dot(T8)

    # joint5的变换矩阵
    rpy9 = [0, 0, joint_angles[4]]
    xyz9 = [0, 0, 0]
    R9 = euler_to_rotation_matrix(rpy9)
    T9 = np.eye(4)
    T9[:3, :3] = R9
    T9[:3, 3] = xyz9
    T["joint5"] = T["link4"].dot(T9)

    # link5的变换矩阵
    rpy10 = [0, 0, 0]
    xyz10 = [3.9009e-05, -0.016660, 0.009755]
    R10 = euler_to_rotation_matrix(rpy10)
    T10 = np.eye(4)
    T10[:3, :3] = R10
    T10[:3, 3] = xyz10
    T["link5"] = T["joint5"].dot(T10)

    # joint6的变换矩阵
    rpy11 = [1.5708, 0, joint_angles[5]]
    xyz11 = [0, 0, 0]
    R11 = euler_to_rotation_matrix(rpy11)
    T11 = np.eye(4)
    T11[:3, :3] = R11
    T11[:3, 3] = xyz11
    T["joint6"] = T["link5"].dot(T11)

    # link6的变换矩阵
    rpy12 = [0, 0, 0]
    xyz12 = [9.9559e-05, -2.9871e-05, 0.049717]
    R12 = euler_to_rotation_matrix(rpy12)
    T12 = np.eye(4)
    T12[:3, :3] = R12
    T12[:3, 3] = xyz12
    T["link6"] = T["joint6"].dot(T12)

    # joint_hand的变换矩阵
    rpy13 = [0, 0, joint_angles[6]]
    xyz13 = [0, 0, 0]
    R13 = euler_to_rotation_matrix(rpy13)
    T13 = np.eye(4)
    T13[:3, :3] = R13
    T13[:3, 3] = xyz13
    T["joint_hand"] = T["link6"].dot(T13)

    # link_hand的变换矩阵
    rpy14 = [0, 0, 0]
    xyz14 = [0.012192, -0.001161, 0.097033]
    R14 = euler_to_rotation_matrix(rpy14)
    T14 = np.eye(4)
    T14[:3, :3] = R14
    T14[:3, 3] = xyz14
    T["link_hand"] = T["joint_hand"].dot(T14)

    # joint_right的变换矩阵
    rpy15 = [-1.5708, 0, joint_angles[7]]
    xyz15 = [0, 0, 0.1465]
    R15 = euler_to_rotation_matrix(rpy15)
    T15 = np.eye(4)
    T15[:3, :3] = R15
    T15[:3, 3] = xyz15
    T["joint_right"] = T["link_hand"].dot(T15)

    # link_right的变换矩阵
    rpy16 = [0, 0, 0]
    xyz16 = [-0.006735, -0.005156, -0.046502]
    R16 = euler_to_rotation_matrix(rpy16)
    T16 = np.eye(4)
    T16[:3, :3] = R16
    T16[:3, 3] = xyz16
    T["link_right"] = T["joint_right"].dot(T16)

    # joint_left的变换矩阵
    rpy17 = [-1.5708, 0, joint_angles[8]]
    xyz17 = [0, 0, 0.1465]
    R17 = euler_to_rotation_matrix(rpy17)
    T17 = np.eye(4)
    T17[:3, :3] = R17
    T17[:3, 3] = xyz17
    T["joint_left"] = T["link_hand"].dot(T17)

    # link_left的变换矩阵
    rpy18 = [0, 0, 0]
    xyz18 = [0.006735, -0.005156, -0.046502]
    R18 = euler_to_rotation_matrix(rpy18)
    T18 = np.eye(4)
    T18[:3, :3] = R18
    T18[:3, 3] = xyz18
    T["link_left"] = T["joint_left"].dot(T18)

    return T


def visualize_spheres_with_pybullet(T, spheres, link_names):
    # 初始化 pybullet
    physicsClient = p.connect(p.GUI)  # 连接到图形界面
    p.setGravity(0, 0, 0)  # 去除重力
    # 创建一个平面作为地面
    planeId = p.loadURDF(
        "/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf")  # 使用内置的平面URDF

    # 遍历每个链接的球体数据，并将其在 pybullet 中创建为物体
    for link in link_names:
        spheres_link = spheres.get(link, [])
        # 使用链接的变换矩阵
        T_link = T.get(link, np.eye(4))  # 如果没有找到链接，使用单位矩阵作为默认值

        for sphere in spheres_link:
            sphere_position = transform_sphere_pos(T_link, sphere["center"])
            # 在pybullet中添加一个球体
            sphere_radius = sphere.get("radius", MAX_RADIUS)
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[1, 0, 0, 1]  # 红色球体
            )
            collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
            p.createMultiBody(
                baseMass=0,  # 质量为0，保持固定
                baseCollisionShapeIndex=collision_shape_id,
                baseVisualShapeIndex=visual_shape_id,
                basePosition=sphere_position.tolist()  # 设置球体的位置
            )

    # 设置视角，便于观察
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    # 运行仿真并保持可视化
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)  # 等待下一帧

    p.disconnect()


def get_parent_joint_name(link_name):
    """
    根据link_name获取其父关节的名称。
    这里需要根据你的URDF结构进行修改。
    """
    parent_joint_map = {
        "link1": "base_link",
        "link2": "link1",
        "link3": "link2",
        "link4": "link3",
        "link5": "link4",
        "link6": "link5",
        "link_hand": "link6",
        "link_right": "link_hand",
        "link_left": "link_hand"
    }
    return parent_joint_map.get(link_name)

if __name__ == "__main__":
    # 读取球体数据
    with open("link_spheres.json", "r") as f:
        spheres = json.load(f)
    link_names = list(spheres.keys())
    # 输入关节角度
    posture_input = input("请输入关节角度（以空格分隔，如：0.5236 -0.7854 0.3491 ...）：")
    posture_angles = np.array([float(angle) for angle in posture_input.split()])
    # 确保输入的角度数量为10
    if len(posture_angles) != 10:
        raise ValueError("请确保输入10个关节角度。")
    # 计算变换矩阵
    T = compute_transforms(posture_angles)
    # 在 pybullet 中可视化球体
    visualize_spheres_with_pybullet(T, spheres, link_names)


