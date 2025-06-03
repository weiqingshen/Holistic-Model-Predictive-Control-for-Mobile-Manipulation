# final_output_results.py
import os  # 确保在文件顶部引入 os 模块
import json
import numpy as np
from pybullet_examples.jacobian import result
from scipy.spatial.distance import cdist  # 用于计算两个点集之间的欧几里得距离
from final_symbolic_transform_v7 import get_symbolic_transforms_of_closest_spheres,make_transform_sym
from final_symbolic_transform_v7 import compute_transforms_symbolic_one_step
import casadi as ca
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

# 为特定的Link对单独设置危险距离（无方向性）
CUSTOM_THRESHOLDS = {
    ("link2", "link4"): 0.032,
    ("link4", "link2"): 0.032,  # 加上反向，以确保无论顺序如何都能匹配
    ("link3", "obstacle"): 0.2,
    ("obstacle", "link3"): 0.2,  # 加上反向，以确保无论顺序如何都能匹配
    ("link4", "obstacle"): 0.2,
    ("obstacle", "link4"): 0.2,  # 加上反向，以确保无论顺序如何都能匹配
    ("link_hand", "obstacle"): 0.2,
    ("obstacle", "link_hand"): 0.2,
}
DEFAULT_THRESHOLD = 0.03
# 定义每对连杆需要保留的最近点数，如果没有特别设置则使用默认值
CUSTOM_TOP_K = {

    ("link3", "obstacle"): 30,  # 对于link3和obstacle，保留5个最近点
    ("obstacle", "link3"): 30,  # 对于反向的obstacle和link3，保留5个最近点
    ("link4", "obstacle"): 30,  # 对于link4和obstacle，保留5个最近点
    ("obstacle", "link4"): 30,  # 对于反向的obstacle和link4，保留5个最近点
    ("link_hand", "obstacle"): 20,  # 对于link_hand和obstacle，保留7个最近点
    ("obstacle", "link_hand"): 20,  # 对于反向的obstacle和link_hand，保留7个最近点
    # 如果没有特别设置，可以在此处添加更多自定义的连杆对设置
}
DEFAULT_TOP_K = 10
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


def compute_min_distances(transforms, rel_spheres):
    from scipy.spatial.distance import cdist
    from collections import defaultdict

    world_positions = {}
    sphere_to_link_transforms = {}

    for link, spheres in rel_spheres.items():
        if link not in transforms:
            continue
        T_link = transforms[link]
        R_link = T_link[:3, :3]
        world_positions[link] = []
        sphere_to_link_transforms[link] = []

        for pos in spheres:
            local = np.array(pos + [1.0])
            world = T_link @ local
            world_positions[link].append(world[:3])

            T_sphere = np.eye(4)
            T_sphere[:3, :3] = R_link
            T_sphere[:3, 3] = world[:3]
            T_sphere_to_link = np.linalg.inv(T_link) @ T_sphere
            sphere_to_link_transforms[link].append(T_sphere_to_link)

        world_positions[link] = np.array(world_positions[link])

    # 用于收集每个link对的所有近距离点
    pairwise_candidates = defaultdict(list)

    links = list(world_positions.keys())
    for i in range(len(links)):
        for j in range(i + 1, len(links)):
            a, b = links[i], links[j]
            if (a, b) in IGNORE_PAIRS:
                continue
            pos_a, pos_b = world_positions[a], world_positions[b]
            dists = cdist(pos_a, pos_b)

            for idx_a in range(len(pos_a)):
                for idx_b in range(len(pos_b)):
                    dist = dists[idx_a, idx_b]
                    pair_key = (a, b)
                    threshold = CUSTOM_THRESHOLDS.get(pair_key, DEFAULT_THRESHOLD)
                    if dist < threshold:
                        T_a_to_link = sphere_to_link_transforms[a][idx_a]
                        T_b_to_link = sphere_to_link_transforms[b][idx_b]
                        pairwise_candidates[pair_key].append(
                            (dist, a, b, T_a_to_link, T_b_to_link, pos_a[idx_a], pos_b[idx_b])
                        )

    # 最终筛选结果
    min_distances = []
    closest_pairs = []

    for pair_key, candidates in pairwise_candidates.items():
        # 按距离排序
        candidates.sort(key=lambda x: x[0])
        top_k_value = CUSTOM_TOP_K.get(pair_key, DEFAULT_TOP_K)  # 如果没有指定，使用默认的30个点
        top_k = candidates[:top_k_value]  # 获取最近的`top_k_value`个点
        # top_k = candidates[:30]  # 只保留前一些个最近的点
        for dist, a, b, T_a_to_link, T_b_to_link, p1, p2 in top_k:
            min_distances.append((a, b, dist, T_a_to_link, T_b_to_link))
            closest_pairs.append((p1, p2))

    return min_distances, closest_pairs



    return [(min_point, proj)]
# 计算点到圆柱的最小距离（只计算z方向的距离差）
def point_to_cylinder_distance(p, cyl_center, cyl_radius, cyl_height):
    x, y, z = p
    cx, cy, cz = cyl_center

    # 计算XY平面上的距离
    d_xy = np.hypot(x - cx, y - cy)

    # 计算Z轴上的距离
    dz_below = max(0.0, cz - z)  # 计算Z轴下端的距离
    dz_above = max(0.0, z - (cz + cyl_height))  # 计算Z轴上端的距离

    # 计算到圆柱表面的径向距离（如果点已经在圆柱表面，返回0）
    dr = max(0.0, d_xy - cyl_radius)

    if dz_below == 0 and dz_above == 0:
        return dr  # 如果Z坐标在圆柱的高度范围内，返回径向距离
    return np.hypot(dr, dz_below + dz_above)  # 如果Z坐标不在圆柱的高度范围内，返回总的最短距离


def compute_cylinder_pair_for_hand(link_name, transforms, data, cyl_center, cyl_radius, cyl_height, max_xy_error=0.05):
    if link_name not in transforms or link_name not in data:
        return []

    T_link = transforms[link_name]  # 获取该连杆的变换矩阵

    # 使用 numpy 来计算每个点的位置
    positions = [np.dot(T_link, np.array(rel + [1]))[:3] for rel in data[link_name]]  # 转为3D坐标（3x1）

    if not positions:
        return []

    # 计算每个点到圆柱的最小距离，只考虑 z 方向
    valid_positions = []
    for p in positions:
        x, y, z = p
        dist_xy = np.hypot(x - cyl_center[0], y - cyl_center[1])

        # 如果 x/y 平面的误差在允许范围内，忽略 x/y，计算 z 方向的距离差
        if abs(dist_xy) <= max_xy_error + cyl_radius:
            # 打包为 dist, T_a_to_link, T_b_to_link
            T_sphere = np.eye(4)
            T_sphere[:3, :3] = T_link[:3, :3]
            T_sphere[:3, 3] = p
            sphere_to_link_transform = np.linalg.inv(T_link) @ T_sphere  # Transform of the sphere to the link

            valid_positions.append((dist_xy, T_link, sphere_to_link_transform))  # 保存需要的值

    if not valid_positions:
        return []

    # 按 z 方向距离排序，返回最近的10个点
    valid_positions.sort(key=lambda x: abs(x[1][2, 3] - cyl_center[2]))  # 按照 Z 方向的距离排序

    # 初始化空列表用于存储结果
    min_distances = []

    # 使用 append 将每个结果添加到 min_distances 列表中
    for dist, T_a_to_link, T_b_to_link in valid_positions[:10]:
        min_distances.append([dist, T_b_to_link])  # 只保存 dist 和 T_b_to_link

    return min_distances


# 计算每个连杆到圆柱的最小点对（只计算link_hand）
def compute_cylinder_distances_for_link_hand(transforms, data, cyl_center, cyl_radius, cyl_height):
    result = {}

    # 计算 link_hand 到圆柱的最小距离
    result['link_hand'] = compute_cylinder_pair_for_hand('link_hand', transforms, data, cyl_center, cyl_radius, cyl_height)

    return result

# 小球变换（CasADi）
def transform_sphere_symbolic(T_link, T_sphere_to_link):
    return ca.mtimes(T_link, T_sphere_to_link)

def compute_symbolic_distances_for_time_steps(all_T, result_hand, cyl_center, cyl_radius, cyl_height):
    distances = []

    # 遍历每个时间步的变换矩阵 all_T
    for t_idx, T_t in enumerate(all_T):
        # 遍历 result_hand 中的每个连杆（link_hand 或其他连杆）
        for dist, T_to_link in result_hand:
            # 使用 T_t 计算球体在世界坐标系中的位置
            T_world = transform_sphere_symbolic(T_t["link_hand"], T_to_link)

            # 提取球体位置的 x, y, z 坐标
            pos_a = T_world[0:3, 3]  # 球体在世界坐标系中的位置

            x_ball_world = pos_a[0]
            y_ball_world = pos_a[1]
            z_ball_world = pos_a[2]

            # # 计算球体与圆柱上表面的最小距离（只考虑 xy 平面）
            # dist_xy = ca.sqrt((x_ball_world - cyl_center[0]) ** 2 + (y_ball_world - cyl_center[1]) ** 2) - cyl_radius
            #
            # # 计算 z 方向上的距离差
            # dz_above = ca.fmax(0.0, z_ball_world - (cyl_center[2] + cyl_height))  # 超过圆柱上表面的部分
            # dz_below = ca.fmax(0.0, cyl_center[2] - z_ball_world)  # 低于圆柱下表面的部分
            #
            # # 最终的符号距离函数
            # total_distance = ca.sqrt(dist_xy ** 2 + (dz_above + dz_below) ** 2)
            # 计算 z 方向上的距离差（修改后的计算）
            dz = z_ball_world - (cyl_center[2] + 0.5 * cyl_height)  # 球体与圆柱中心 z 坐标的差值

            # 最终的符号距离函数
            total_distance = ca.sqrt(dz ** 2)

            # 将计算出的距离加入到列表中
            distances.append([t_idx, dist, total_distance])  # 返回时间步、原始距离和符号距离

    return distances
# 主流程函数，执行整套距离计算和结果输出

def compute_gripper_cylinder_distance_independent(transforms, data, cyl_center, cyl_radius, cyl_height, max_xy_error=0.05):
    """
    独立函数：计算 gripper（link_left 和 link_right）到圆柱的最近点信息。
    返回格式：
    {
        'link_left': [(dist, T_sphere_to_link), ...],
        'link_right': [(dist, T_sphere_to_link), ...]
    }
    """

    result = {}

    for link_name in ['link_left', 'link_right']:
        if link_name not in transforms or link_name not in data:
            result[link_name] = []
            continue

        T_link = transforms[link_name]
        positions = [np.dot(T_link, np.array(rel + [1]))[:3] for rel in data[link_name]]
        valid_positions = []

        for p in positions:
            x, y, z = p
            # 判断 Z 方向的差值是否小于圆柱高度的一半
            if abs(z - cyl_center[2]) <= 0.5 * cyl_height:
                dist_xy = np.hypot(x - cyl_center[0], y - cyl_center[1])

                # 如果 X, Y 平面的距离满足要求，计算并保存结果
                if abs(dist_xy) <= max_xy_error + cyl_radius:
                    T_sphere = np.eye(4)
                    T_sphere[:3, :3] = T_link[:3, :3]
                    T_sphere[:3, 3] = p
                    sphere_to_link_transform = np.linalg.inv(T_link) @ T_sphere
                    valid_positions.append((dist_xy, sphere_to_link_transform))

        # 按 Z 高度差（即与圆柱中心 Z 坐标的差值）和 X, Y 平面距离排序
        valid_positions.sort(key=lambda x: (abs(x[1][2, 3] - cyl_center[2]), x[0]))

        result[link_name] = valid_positions[:10]  # 最多返回10个

    return result

def compute_symbolic_distances_for_gripper_time_steps(all_T, result_gripper, cyl_center, cyl_radius):
    distances = []

    # 遍历每个时间步的变换矩阵 all_T
    for t_idx, T_t in enumerate(all_T):
        # 遍历 result_gripper 中的每个连杆（link_left 或 link_right）
        for link_name, entries in result_gripper.items():
            for dist, T_to_link in entries:
                # 使用 T_t 计算球体在世界坐标系中的位置
                T_world = transform_sphere_symbolic(T_t[link_name], T_to_link)

                # 提取球体位置的 x, y, z 坐标
                pos_a = T_world[0:3, 3]  # 球体在世界坐标系中的位置

                x_ball_world = pos_a[0]
                y_ball_world = pos_a[1]
                z_ball_world = pos_a[2]

                # 计算球体与圆柱上表面的最小距离（只考虑 xy 平面）
                dist_xy = ca.sqrt((x_ball_world - cyl_center[0]) ** 2 + (y_ball_world - cyl_center[1]) ** 2) - cyl_radius

                # 最终的符号距离函数（只考虑 XY 平面）
                total_distance = dist_xy

                # 将计算出的距离加入到列表中
                distances.append([t_idx, link_name, dist, total_distance])  # 返回时间步、link_name、原始距离和符号距离

    return distances

def run_pipeline(X,joint_angles, sphere_json_path="link_spheres_relative_v2.json", cyl_center=(0.5, 0.7, 0.035),
                 cyl_radius=0.025, cyl_height=0.07):
    # 1) 加载球心相对坐标
    if not os.path.isabs(sphere_json_path):
        base_dir = os.path.dirname(__file__)
        sphere_json_path = os.path.join(base_dir, sphere_json_path)
    with open(sphere_json_path, "r") as f:
        data = json.load(f)

    # 2) 计算所有连杆世界变换
    transforms = compute_transforms(joint_angles)


    # 3) 计算连杆间最小球对距离及对应位置
    min_distances, closest_pairs = compute_min_distances(transforms, data)
    # print(min_distances)
    result = get_symbolic_transforms_of_closest_spheres(min_distances, X)

    # 计算所有连杆世界变换
    num_steps = X.shape[1]
    all_T = []
    for t in range(num_steps):
        X_t = X[:, t]
        T_t = compute_transforms_symbolic_one_step(X_t)
        all_T.append(T_t)

    # 计算 link_hand 到圆柱的最小距离
    # result_hand = compute_cylinder_distances_for_link_hand(transforms, data, cyl_center, cyl_radius, cyl_height)
    result_hand=compute_cylinder_pair_for_hand('link_hand', transforms, data, cyl_center, cyl_radius, cyl_height)
    result_gripper=compute_gripper_cylinder_distance_independent(transforms, data, cyl_center, cyl_radius, cyl_height)
    # print(result_hand)
    # 调用函数计算每个时间步下的符号化距离
    hand_distances = compute_symbolic_distances_for_time_steps(all_T, result_hand, cyl_center, cyl_radius, cyl_height)
    # 计算 gripper 的符号化距离（只考虑 XY 平面）
    gripper_distances = compute_symbolic_distances_for_gripper_time_steps(all_T, result_gripper, cyl_center, cyl_radius)

    formatted_hand_distances = [
        [["link_hand", "cylinder"], dist, total_distance, t_idx]  # 添加 [link_hand, cylinder] 作为前缀
        for t_idx, dist, total_distance in hand_distances
    ]
    formatted_gripper_distances = [
        [[link_name, "cylinder"], dist, total_distance, t_idx]  # 添加 [link_name, cylinder] 作为前缀
        for t_idx, link_name, dist, total_distance in gripper_distances
    ]

    # 将 formatted_hand_distances 添加到 result 中
    result.extend(formatted_hand_distances)
    result.extend(formatted_gripper_distances)

    return result


