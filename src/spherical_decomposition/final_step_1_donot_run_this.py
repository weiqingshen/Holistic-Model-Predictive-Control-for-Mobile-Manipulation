import pybullet as p
import time
import pybullet_data
import math
import json

# 最大半径（单位：米）
MAX_RADIUS = 0.01

# 用于对不同 link 着色的颜色列表
COLORS = [
    [1, 0, 0, 0.3], [0, 1, 0, 0.3], [0, 0, 1, 0.3],
    [1, 1, 0, 0.3], [0, 1, 1, 0.3], [1, 0, 1, 0.3],
    [1, 0.5, 0, 0.3], [0.5, 0, 0.5, 0.3], [0.5, 0.5, 0.5, 0.3],
]

# 为四个长方体统一使用的颜色
CUBES_COLOR = [1, 0, 0, 0.3]

# 用于按 link 名称记录球的坐标
link_spheres_dict = {}  # ✅ 保存球坐标为按 link 分类的字典结构 JSON 文件


def decompose_spheres(aabb_min, aabb_max, color, link_name):
    """
    将一个 AABB 区域分解成多个球体，每个球体半径不超过 MAX_RADIUS。
    分解的球体信息会按 link_name 分类保存。
    """
    extents = [aabb_max[i] - aabb_min[i] for i in range(3)]
    radius = MAX_RADIUS
    num_x = max(math.ceil(extents[0] / (2 * radius)), 2)
    num_y = max(math.ceil(extents[1] / (2 * radius)), 2)
    num_z = max(math.ceil(extents[2] / (2 * radius)), 2)

    for i in range(num_x):
        for j in range(num_y):
            for k in range(num_z):
                center = [
                    aabb_min[0] + (i + 0.5) * 2 * radius,
                    aabb_min[1] + (j + 0.5) * 2 * radius,
                    aabb_min[2] + (k + 0.5) * 2 * radius
                ]
                vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
                p.createMultiBody(baseVisualShapeIndex=vs_id, basePosition=center, baseMass=0)

                # ✅ 将球的绝对坐标保存到对应 link 的字典项中
                if link_name not in link_spheres_dict:
                    link_spheres_dict[link_name] = []
                link_spheres_dict[link_name].append(center)


def decompose_4_cubes(link_name):
    """
    ✅ 替代 link 编号 3 的默认 AABB 分解逻辑，使用四个手动定义的长方体区域进行分解。
    """
    aabbs = [
        ([-0.18, -0.26, 0], [0.17, -0.08, 0.2]),
        ([-0.18, -0.08, 0], [0.17, 0.06, 0.12]),
        ([-0.15, -0.07, 0.12], [-0.10, -0.03, 0.36]),
        ([-0.15, -0.01, 0.25], [-0.10, 0.04, 0.31]),
        ([-0.15, -0.1, 0.31], [-0.10, -0.13, 0.34]),
        ([-0.15, -0.13, 0.31], [-0.10, -0.18, 0.34])
    ]
    for aabb_min, aabb_max in aabbs:
        decompose_spheres(aabb_min, aabb_max, CUBES_COLOR, link_name)


def decompose_urdf_links(robot_id):
    """
    遍历 URDF 中的所有链接，对其进行球体分解。
    """
    num_joints = p.getNumJoints(robot_id)
    color_index = 0
    for idx in range(num_joints):

        # ✅ 跳过 link 编号 -1, 0, 1, 2
        if idx in [-1, 0, 1, 2]:
            continue

        # 获取 link 名字
        link_info = p.getJointInfo(robot_id, idx)
        link_name = link_info[12].decode("utf-8")

        # ✅ 对 link 编号为 3 的 link 使用四个长方体替代默认分解
        if idx == 3:
            decompose_4_cubes(link_name)
        else:
            # 普通 link：获取 AABB 后进行球体分解
            aabb_min, aabb_max = p.getAABB(robot_id, idx)
            color = COLORS[color_index % len(COLORS)]
            color_index += 1
            decompose_spheres(aabb_min, aabb_max, color, link_name)
            print(f"Link {idx} ({link_name}) decomposed with color: {color}")


def visualize_urdf_with_sphere_decomposition(urdf_path):
    """
    主函数：加载 URDF 模型，进行球形分解，并将坐标保存为 JSON 文件。
    """
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

    decompose_urdf_links(robot_id)

    # 将所有关节质量设为 0，防止掉落
    for i in range(p.getNumJoints(robot_id)):
        p.changeDynamics(robot_id, i, mass=0)

    # ✅ 保存球坐标为按 link 分类的字典结构 JSON 文件
    with open("link_spheres.json", "w") as f:
        json.dump(link_spheres_dict, f, indent=2)
    print(f"所有链接的球坐标已保存至 link_spheres.json 文件。")

    # 可视化运行
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)
    p.disconnect()


# ✅ 执行入口
visualize_urdf_with_sphere_decomposition(
    "/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf"
)
