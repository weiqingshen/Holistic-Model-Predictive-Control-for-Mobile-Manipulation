
import pybullet as p
import pybullet_data
import time
import math
import json
import numpy as np

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


def transform_to_link_frame(position, orientation, link_center):
    """
    将球的位置从世界坐标系转换到链接坐标系
    position 是球在世界坐标系下的位置
    orientation 是链接的旋转四元数
    link_center 是链接的中心点位置
    """
    # 旋转矩阵
    rot_matrix = p.getMatrixFromQuaternion(orientation)
    rot_matrix = [rot_matrix[0:3], rot_matrix[3:6], rot_matrix[6:9]]

    # 计算相对坐标
    relative_position = [
        position[0] - link_center[0],
        position[1] - link_center[1],
        position[2] - link_center[2]
    ]

    # 使用旋转矩阵将相对坐标旋转到链接坐标系
    transformed_position = [
        sum(rot_matrix[i][j] * relative_position[j] for j in range(3))
        for i in range(3)
    ]

    return transformed_position


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
    joint_info_dict = {}  # 用于存储关节信息的字典

    # base_link 的处理：使用 p.getAABB(robot_id, -1)
    aabb_min, aabb_max = p.getAABB(robot_id, -1)
    link_spheres["base_link"] = decompose_spheres(aabb_min, aabb_max, COLORS[0])
    print("base_link 分解成 {} 个球".format(len(link_spheres["base_link"])))

    # 处理每个 link 的分解，从编号 0 到 num_joints-1
    color_index = 1
    for idx in range(num_joints):
        if idx in [-1, 0, 1, 2]:
            continue
        # 获取 link 的 AABB 与名称
        aabb_min, aabb_max = p.getAABB(robot_id, idx)
        info = p.getJointInfo(robot_id, idx)
        link_name = info[12].decode("utf-8")  # 第13个元素为 link 名称
        joint_index = idx  # 关节索引
        joint_type = info[2]  # 关节类型 (0=revolute, 1=prismatic, 2=spherical, 3=planar, 4=fixed)
        joint_axis = info[13]  # 关节轴 (世界坐标系)
        joint_lower_limit = info[8]  # 关节下限
        joint_upper_limit = info[9]  # 关节上限
        joint_max_force = info[10]  # 关节最大力
        joint_max_velocity = info[11]  # 关节最大速度

        # 将关节信息存储到字典中
        joint_info_dict[link_name] = {
            "joint_index": joint_index,
            "joint_type": joint_type,
            "joint_axis": joint_axis,
            "joint_lower_limit": joint_lower_limit,
            "joint_upper_limit": joint_upper_limit,
            "joint_max_force": joint_max_force,
            "joint_max_velocity": joint_max_velocity,
        }

        # 获取当前 link 的位置信息和旋转信息
        link_state = p.getLinkState(robot_id, idx)
        link_position = link_state[0]  # 世界坐标系下的链接位置
        link_orientation = link_state[1]  # 世界坐标系下的链接旋转四元数

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

        # 转换每个球的坐标到当前 link 的坐标系
        transformed_spheres = []
        for sphere in spheres:
            transformed_center = transform_to_link_frame(link_position, link_orientation, sphere['center'])
            transformed_spheres.append({'center': transformed_center, 'color': sphere['color']})

        # 如果是 base_link，保持球在大地坐标系下
        if link_name == "base_link":
            transformed_spheres = spheres  # 不做任何转换

        link_spheres[link_name] = transformed_spheres
        print("Link {} 分解成 {} 个球".format(link_name, len(spheres)))

    # 将所有 link 的分解结果和关节信息写入 JSON 文件
    output_data = {
        "link_spheres": link_spheres,
        "joint_info": joint_info_dict,
    }

    with open(output_filename, "w") as f:
        json.dump(output_data, f, indent=4)
    print("分解数据已保存到文件：", output_filename)

    p.disconnect()


if __name__ == "__main__":
    # 请修改下面的 URDF 路径为你实际使用的文件路径
    urdf_path = "/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf"
    output_filename = "link_spheres.json"
    decompose_links_in_urdf(urdf_path, output_filename)
    time.sleep(1)



