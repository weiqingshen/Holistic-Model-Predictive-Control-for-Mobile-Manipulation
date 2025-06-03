# final_output_results.py
import os  # 确保在文件顶部引入 os 模块
import json
import numpy as np
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

# 计算点到圆柱的符号化距离
def point_to_cylinder_distance_symbolic(point, T_link, cyl_center, cyl_radius, cyl_height):
    # 将point转换为CasADi符号矩阵，并确保是4x1列向量（齐次坐标）
    point_symbolic = ca.vertcat(*point, 1)  # 转换为4x1列向量（包含齐次坐标）

    # 确保T_link和point_symbolic是同一类型（SX类型）
    point_symbolic = ca.SX(point_symbolic)  # 确保point_symbolic是SX类型

    # 使用变换矩阵 T_link 对点进行变换
    # 由于T_link是4x4矩阵，point_symbolic是4x1列向量，可以进行矩阵乘法
    transformed_point = ca.mtimes(T_link, point_symbolic)  # 使用 T_link 对点进行变换，结果是4x1矩阵

    # 取前三个元素，即变换后的坐标（x, y, z）
    x, y, z = transformed_point[0], transformed_point[1], transformed_point[2]
    cx, cy, cz = cyl_center

    # 计算XY平面上的距离
    d_xy = ca.hypot(x - cx, y - cy)

    # 计算圆柱表面的径向距离（如果点在圆柱表面上，返回0）
    d_r = ca.fmax(0.0, d_xy - cyl_radius)

    # 计算Z轴上的距离
    dz_below = ca.fmax(0.0, cz - z)  # 计算Z轴下端的距离
    dz_above = ca.fmax(0.0, z - (cz + cyl_height))  # 计算Z轴上端的距离

    # 最终的符号化距离计算
    return ca.hypot(d_r, dz_below + dz_above)

# 计算点到圆柱的最小距离
def point_to_cylinder_distance(point, cyl_center, cyl_radius, cyl_height):
    x, y, z = point
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


# 投影点到圆柱表面（数值计算）
def project_point_to_cylinder(p, cyl_center, cyl_radius, cyl_height):
    x, y, z = p
    cx, cy, cz = cyl_center

    # 计算XY平面上的距离
    d_xy = np.hypot(x - cx, y - cy)

    # 计算点的投影点
    if d_xy == 0:
        proj_xy = (cx + cyl_radius, cy)  # 如果点在圆柱中心，投影到圆柱的表面
    else:
        proj_xy = (cx + cyl_radius * (x - cx) / d_xy, cy + cyl_radius * (y - cy) / d_xy)

    # 限制Z坐标在圆柱的高度范围
    proj_z = min(max(z, cz), cz + cyl_height)

    # 返回投影点的3D坐标
    return np.array([proj_xy[0], proj_xy[1], proj_z])


# 计算每个连杆到圆柱的最小点对
def compute_cylinder_pair(link_name, transforms, data, cyl_center, cyl_radius, cyl_height):
    if link_name not in transforms or link_name not in data:
        return []

    T_link = transforms[link_name]  # 获取该连杆的变换矩阵

    # 使用 numpy 来计算每个点的位置
    positions = [np.dot(T_link, np.array(rel + [1]))[:3] for rel in data[link_name]]  # 转为3D坐标（3x1）

    if not positions:
        return []

    # 计算每个点到圆柱的最小距离
    min_dist = float('inf')
    min_point = None

    for p in positions:
        dist = point_to_cylinder_distance(p, cyl_center, cyl_radius, cyl_height)  # 计算距离
        if dist < min_dist:
            min_dist = dist
            min_point = p

    # 计算投影到圆柱表面的点
    proj = project_point_to_cylinder(min_point, cyl_center, cyl_radius, cyl_height)

    return [(min_point, proj)]


# 主流程函数，执行整套距离计算和结果输出
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

    result = get_symbolic_transforms_of_closest_spheres(min_distances, X)

    # 计算所有连杆世界变换
    num_steps = X.shape[1]
    all_T = []
    for t in range(num_steps):
        X_t = X[:, t]
        T_t = compute_transforms_symbolic_one_step(X_t)
        all_T.append(T_t)


    # 5) 遍历每个时间步，计算连杆到圆柱的最小距离以及符号化距离表达式
    for t_idx in range(num_steps):
        for link in ['link_hand', 'link_left', 'link_right']:
            pairs = compute_cylinder_pair(link,transforms, data, cyl_center, cyl_radius, cyl_height)
            for p1, p2 in pairs:
                # 计算点 p1 和 p2 之间的符号化距离
                print(p1, p2)
                # print(all_T[t_idx])
                dist_expr = point_to_cylinder_distance_symbolic(p1, all_T[t_idx],cyl_center, cyl_radius, cyl_height)

                # 将结果添加到 result 中
                result.append([link, dist_expr, dist_expr, t_idx])

    return result


