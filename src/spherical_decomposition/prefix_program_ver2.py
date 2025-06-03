
import pybullet as p
import pybullet_data
import time
import math
import json
import numpy as np  # 导入 numpy
from scipy.spatial.transform import Rotation as R  # 导入旋转类

# 全局常量：球的最大半径
MAX_RADIUS = 0.02  # 1cm

# 为便于识别调试定义颜色（此处颜色可以忽略或删除）
COLORS = [
    [1, 0, 0, 0.3],  # 红色
    [0, 1, 0, 0.3],  # 绿色
    [0, 0, 1, 0.3],  # 蓝色
    [1, 1, 0, 0.3],  # 黄色
    [0, 1, 1, 0.3],  # 青色
    [1, 0, 1, 0.3],  # 品红
    [1, 0.5, 0, 0.3],  # 橙色
    [0.5, 0, 0.5, 0.3],  # 紫色
    [0.5, 0.5, 0.5, 0.3]  # 灰色
]

# 专用于长方体分解的颜色（可以删除颜色参数）
CUBES_COLOR = [1, 0, 0, 0.3]


def decompose_spheres(aabb_min, aabb_max, color):
    """
    针对给定的 AABB 区域进行球状分解，确保每个球半径不超过 MAX_RADIUS，
    返回的球数据列表中，每个球记录了局部坐标下的 'center'
    """
    extents = [aabb_max[i] - aabb_min[i] for i in range(3)]
    # 每个轴上至少生成两个球
    num_spheres_in_x = max(math.ceil(extents[0] / (2 * MAX_RADIUS)), 2)
    num_spheres_in_y = max(math.ceil(extents[1] / (2 * MAX_RADIUS)), 2)
    num_spheres_in_z = max(math.ceil(extents[2] / (2 * MAX_RADIUS)), 2)

    spheres = []
    # 均匀分布，球心采用局部坐标
    for i in range(num_spheres_in_x):
        for j in range(num_spheres_in_y):
            for k in range(num_spheres_in_z):
                center = [
                    aabb_min[0] + (i + 0.5) * 2 * MAX_RADIUS,
                    aabb_min[1] + (j + 0.5) * 2 * MAX_RADIUS,
                    aabb_min[2] + (k + 0.5) * 2 * MAX_RADIUS,
                ]
                spheres.append({'center': center, 'color': color})
    return spheres


def euler_to_rotation_matrix(rpy):
    """
    将欧拉角（弧度制）转换为旋转矩阵。
    """
    r = R.from_euler('xyz', rpy)  # 假设欧拉角顺序为 XYZ
    return r.as_matrix()


def compute_transforms(joint_angles):
    """
    计算每个关节的变换矩阵。
    """
    T = {}

    # base_link的变换矩阵
    T["base_link"] = np.eye(4)

    # joint1的变换矩阵
    rpy1 = [0, 0, 0]  # 特意跳过joint——angles【0】，因为这个是baselink的角度，而在本程序中默认baselink不动
    xyz1 = [0, 0, 0.1284]
    R1 = euler_to_rotation_matrix(rpy1)
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T1[:3, 3] = xyz1
    T["link1"] = T["base_link"].dot(T1)

    # joint2的变换矩阵
    rpy3 = [1.5708, 0, 1.5708]
    xyz3 = [0, 0, 0.0927]
    R3 = euler_to_rotation_matrix(rpy3)
    T3 = np.eye(4)
    T3[:3, :3] = R3
    T3[:3, 3] = xyz3
    T["link2"] = T["link1"].dot(T3)

    # joint3的变换矩阵
    rpy5 = [-3.1416, 0, -1.5708]
    xyz5 = [0, 0.22, 0]
    R5 = euler_to_rotation_matrix(rpy5)
    T5 = np.eye(4)
    T5[:3, :3] = R5
    T5[:3, 3] = xyz5
    T["link3"] = T["link2"].dot(T5)

    # joint4的变换矩阵
    rpy7 = [1.5708, 0, 0]
    xyz7 = [0, 0, 0]
    R7 = euler_to_rotation_matrix(rpy7)
    T7 = np.eye(4)
    T7[:3, :3] = R7
    T7[:3, 3] = xyz7
    T["link4"] = T["link3"].dot(T7)

    # joint5的变换矩阵
    rpy9 = [0, 0, 0]
    xyz9 = [0, 0, 0]
    R9 = euler_to_rotation_matrix(rpy9)
    T9 = np.eye(4)
    T9[:3, :3] = R9
    T9[:3, 3] = xyz9
    T["link5"] = T["link4"].dot(T9)

    # joint6的变换矩阵
    rpy11 = [1.5708, 0, 0]
    xyz11 = [0, 0, 0]
    R11 = euler_to_rotation_matrix(rpy11)
    T11 = np.eye(4)
    T11[:3, :3] = R11
    T11[:3, 3] = xyz11
    T["link6"] = T["link5"].dot(T11)

    # joint_hand的变换矩阵
    rpy13 = [0, 0, 0]
    xyz13 = [0, 0, 0]
    R13 = euler_to_rotation_matrix(rpy13)
    T13 = np.eye(4)
    T13[:3, :3] = R13
    T13[:3, 3] = xyz13
    T["link_hand"] = T["link6"].dot(T13)

    # joint_right的变换矩阵
    rpy15 = [-1.5708, 0, 0]
    xyz15 = [0, 0, 0.1465]
    R15 = euler_to_rotation_matrix(rpy15)
    T15 = np.eye(4)
    T15[:3, :3] = R15
    T15[:3, 3] = xyz15
    T["link_right"] = T["link_hand"].dot(T15)

    # joint_left的变换矩阵
    rpy17 = [-1.5708, 0, 0]
    xyz17 = [0, 0, 0.1465]
    R17 = euler_to_rotation_matrix(rpy17)
    T17 = np.eye(4)
    T17[:3, :3] = R17
    T17[:3, 3] = xyz17
    T["link_left"] = T["link_hand"].dot(T17)

    return T


def decompose_links_in_urdf(urdf_path, output_filename="link_spheres.json"):
    """
    从 URDF 文件中提取所有 link，对每个 link 进行球状分解，
    对编号为 3 的 link 使用四个长方体分解替换原有分解结果，
    最后将所有分解结果保存到 JSON 文件中。
    """
    # 使用 DIRECT 模式连接
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # 加载 URDF（固定底座）
    robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

    num_joints = p.getNumJoints(robot_id)
    print("检测到的关节数（不含 base_link）：", num_joints)

    link_spheres = {}

    # 计算变换矩阵
    joint_angles = [0] * num_joints  # 假设所有关节角度为 0
    transforms = compute_transforms(joint_angles)

    # base_link 的处理：使用 p.getAABB(robot_id, -1)
    aabb_min, aabb_max = p.getAABB(robot_id, -1)
    spheres = decompose_spheres(aabb_min, aabb_max, COLORS[0])

    # 将 base_link 的球体转换到 base_link 坐标系 (实际上不需要转换，因为已经是 base_link 坐标系)
    transformed_spheres = []
    for sphere in spheres:
        # 将球心坐标转换为齐次坐标
        sphere_center_homogeneous = np.array(sphere['center'] + [1])
        # 应用变换矩阵
        transformed_center = transforms["base_link"].dot(sphere_center_homogeneous)[:3].tolist()
        transformed_spheres.append({'center': transformed_center, 'color': sphere['color']})

    link_spheres["base_link"] = transformed_spheres
    print("base_link 分解成 {} 个球".format(len(link_spheres["base_link"])))

    # 处理每个 link 的分解，从编号 0 到 num_joints-1
    color_index = 1
    for idx in range(num_joints):
        # 跳过编号为 -1, 0, 1, 2 的 link
        if idx in [-1, 0, 1, 2]:
            continue

        # 获取 link 的 AABB 与名称
        aabb_min, aabb_max = p.getAABB(robot_id, idx)
        info = p.getJointInfo(robot_id, idx)
        link_name = info[12].decode("utf-8")  # 第13个元素为 link 名称

        # 若编号为 3 的 link 按新要求处理
        if idx == 3:
            print("对编号为3的 link({})采用特殊长方体分解".format(link_name))

            cubes = []
            # 第一个长方体：30m * 18cm * 20cm
            aabb_min_1 = [-0.18, -0.26, 0]
            aabb_max_1 = [0.35 - 0.18, 0.18 - 0.26, 0.2]
            cubes.extend(decompose_spheres(aabb_min_1, aabb_max_1, CUBES_COLOR))

            # 第二个长方体：30m * 12cm * 20cm
            aabb_min_2 = [-0.18, 0.18 - 0.26, 0]
            aabb_max_2 = [0.35 - 0.18, 0.32 - 0.26, 0.12]
            cubes.extend(decompose_spheres(aabb_min_2, aabb_max_2, CUBES_COLOR))

            # 第三个长方体：6m * 6cm * 22cm
            aabb_min_3 = [0.05 - 0.2, 0.18 - 0.275, 0.12]
            aabb_max_3 = [0.09 - 0.2, 0.22 - 0.275, 0.36]
            cubes.extend(decompose_spheres(aabb_min_3, aabb_max_3, CUBES_COLOR))

            # 第四个长方体：6m * 8cm * 6cm
            aabb_min_4 = [0.05 - 0.2, 0.25 - 0.275, 0.25]
            aabb_max_4 = [0.09 - 0.2, 0.30 - 0.275, 0.31]
            cubes.extend(decompose_spheres(aabb_min_4, aabb_max_4, CUBES_COLOR))

            spheres = cubes

        else:
            # 对其他 link 按照默认 AABB 分解
            spheres = decompose_spheres(aabb_min, aabb_max, COLORS[color_index % len(COLORS)])
            color_index += 1

        # 确定要使用的变换矩阵的键
        transform_key = link_name  # 优先使用 link_name
        if transform_key not in transforms:
            transform_key = "joint" + str(idx + 1)  # 尝试使用 "jointX" 格式的键
            if transform_key not in transforms:
                print(f"警告：找不到  {link_name} 的变换矩阵，跳过坐标转换。")
                link_spheres[link_name] = spheres  # 不做任何转换
                print("Link {} 分解成 {} 个球".format(link_name, len(spheres)))
                continue

        # 转换每个球的坐标到当前 link 的坐标系
        transformed_spheres = []
        for sphere in spheres:
            # 将球心坐标转换为齐次坐标
            sphere_center_homogeneous = np.array(sphere['center'] + [1])
            # 应用变换矩阵
            transformed_center = transforms[transform_key].dot(sphere_center_homogeneous)[:3].tolist()
            transformed_spheres.append({'center': transformed_center, 'color': sphere['color']})

        link_spheres[link_name] = transformed_spheres
        print("Link {} 分解成 {} 个球".format(link_name, len(spheres)))

    # 将所有 link 的分解结果写入 JSON 文件
    with open(output_filename, "w") as f:
        json.dump(link_spheres, f, indent=4)
    print("分解数据已保存到文件：", output_filename)

    p.disconnect()


if __name__ == "__main__":
    # 请修改下面的 URDF 路径为你实际使用的文件路径
    urdf_path = "/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf"
    output_filename = "link_spheres.json"
    decompose_links_in_urdf(urdf_path, output_filename)
    time.sleep(1)

