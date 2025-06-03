
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
    rpy1 = [0, 0, joint_angles[1]] #    特意跳过joint——angles【0】，因为这个是baselink的角度，而在本程序中默认baselink不动
    xyz1 = [0, 0, 0.1284]
    R1 = euler_to_rotation_matrix(rpy1)
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T1[:3, 3] = xyz1
    T["link1"] = T["base_link"].dot(T1)


    # joint2的变换矩阵
    rpy3 = [1.5708, 0, 1.5708+joint_angles[2]]
    xyz3 = [0, 0, 0.0927]
    R3 = euler_to_rotation_matrix(rpy3)
    T3 = np.eye(4)
    T3[:3, :3] = R3
    T3[:3, 3] = xyz3
    T["link2"] = T["link1"].dot(T3)


    # joint3的变换矩阵
    rpy5 = [ - 3.1416, 0, -1.5708+joint_angles[3]]
    xyz5 = [0, 0.22, 0]
    R5 = euler_to_rotation_matrix(rpy5)
    T5 = np.eye(4)
    T5[:3, :3] = R5
    T5[:3, 3] = xyz5
    T["link3"] = T["link2"].dot(T5)


    # joint4的变换矩阵
    rpy7 = [1.5708,0, 0+ joint_angles[4]]
    xyz7 = [0, 0, 0]
    R7 = euler_to_rotation_matrix(rpy7)
    T7 = np.eye(4)
    T7[:3, :3] = R7
    T7[:3, 3] = xyz7
    T["link4"] = T["link3"].dot(T7)


    # joint5的变换矩阵
    rpy9 = [0, 0, joint_angles[5]]
    xyz9 = [0, 0, 0]
    R9 = euler_to_rotation_matrix(rpy9)
    T9 = np.eye(4)
    T9[:3, :3] = R9
    T9[:3, 3] = xyz9
    T["link5"] = T["link4"].dot(T9)


    # joint6的变换矩阵
    rpy11 = [1.5708, 0, joint_angles[6]]
    xyz11 = [0, 0, 0]
    R11 = euler_to_rotation_matrix(rpy11)
    T11 = np.eye(4)
    T11[:3, :3] = R11
    T11[:3, 3] = xyz11
    T["link6"] = T["link5"].dot(T11)


    # joint_hand的变换矩阵
    rpy13 = [0, 0, joint_angles[7]]
    xyz13 = [0, 0, 0]
    R13 = euler_to_rotation_matrix(rpy13)
    T13 = np.eye(4)
    T13[:3, :3] = R13
    T13[:3, 3] = xyz13
    T["link_hand"] = T["link6"].dot(T13)


    # joint_right的变换矩阵
    rpy15 = [-1.5708, 0, joint_angles[8]]
    xyz15 = [0, 0, 0.1465]
    R15 = euler_to_rotation_matrix(rpy15)
    T15 = np.eye(4)
    T15[:3, :3] = R15
    T15[:3, 3] = xyz15
    T["link_right"] = T["link_hand"].dot(T15)


    # joint_left的变换矩阵
    rpy17 = [-1.5708, 0, joint_angles[9]]
    xyz17 = [0, 0, 0.1465]
    R17 = euler_to_rotation_matrix(rpy17)
    T17 = np.eye(4)
    T17[:3, :3] = R17
    T17[:3, 3] = xyz17
    T["link_left"] = T["link_hand"].dot(T17)


    return T


def visualize_spheres_with_pybullet(T, spheres, link_names):
    # 初始化 pybullet
    physicsClient = p.connect(p.GUI)  # 连接到图形界面
    p.setGravity(0, 0, 0)  # 去除重力
    p.setAdditionalSearchPath("/home/fins/myrobot_move/src/robot_arm_description/urdf/")
    # 创建一个平面作为地面
    planeId = p.loadURDF("robot_arm_description.urdf", useFixedBase=1)  # 使用内置的平面URDF

    # 遍历每个链接的球体数据，并将其在 pybullet 中创建为物体
    for link in link_names:
        spheres_link = spheres.get(link, [])
        # 使用链接的变换矩阵
        T_link = np.eye(4)
        if link in T:
            T_link = T.get(link, np.eye(4))  # 如果没有找到链接，使用单位矩阵作为默认值
        elif "link" in link:
            T_link = T.get(link, np.eye(4))

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

# if __name__ == "__main__":
#     # 读取球体数据
#     with open("link_spheres.json", "r") as f:
#         spheres = json.load(f)
#     link_names = list(spheres["link_spheres"].keys())
#     spheres = spheres["link_spheres"]
#     # 输入关节角度
#     posture_input = input("请输入关节角度（以空格分隔，如：0.5236 -0.7854 0.3491 ...）：")
#     posture_angles = np.array([float(angle) for angle in posture_input.split()])
#     # 确保输入的角度数量为10
#     if len(posture_angles) != 10:
#         raise ValueError("请确保输入10个关节角度。")
#     # 计算变换矩阵
#     T = compute_transforms(posture_angles)
#     # 在 pybullet 中可视化球体
#     visualize_spheres_with_pybullet(T, spheres, link_names)

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
