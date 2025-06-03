import json
import numpy as np

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

def make_transform(rpy, xyz):
    R = euler_to_rotation_matrix(rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T

def compute_transforms(joint_angles):
    T = {}
    base_x, base_y, base_theta = joint_angles[0], joint_angles[1], joint_angles[2]
    T["base_link"] = make_transform([0, 0, base_theta], [base_x, base_y, 0])

    T["link1"] = T["base_link"] @ make_transform([0, 0, joint_angles[3]], [0, 0, 0.1284])
    T["link2"] = T["link1"] @ make_transform([1.5708, -joint_angles[4], 1.5708], [0, 0, 0.0927])
    T["link3"] = T["link2"] @ make_transform([-3.1416, 0, -1.5708 + joint_angles[5]], [0, 0.22, 0])
    T["link4"] = T["link3"] @ make_transform([1.5708, -joint_angles[6], 0], [0, 0, 0])
    T["link5"] = T["link4"] @ make_transform([1.5708, -1.5708 + joint_angles[7], 0], [0, 0, 0.1685])
    T["link6"] = T["link5"] @ make_transform([1.5708, -joint_angles[8], 0], [0, 0, 0])
    T["link_hand"] = T["link6"] @ make_transform([0, 0, 0], [0, 0, 0])
    gripper_translation = joint_angles[9]
    T["link_right"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, -gripper_translation / 2, 0.1465])
    T["link_left"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, gripper_translation / 2, 0.1465])
    return T

def get_joint_angles_from_input():
    print("请输入10个数（x y theta 六个link的角度 gripper的平移），用空格分隔：")
    while True:
        try:
            values = list(map(float, input(">>> ").strip().split()))
            if len(values) != 10:
                raise ValueError("输入的数值数量不正确（应为10个）")
            return values
        except Exception as e:
            print(f"错误：{e}，请重试")

def transform_sphere(T_link, T_sphere_to_link):
    return T_link @ T_sphere_to_link

def compute_new_distances(joint_angles, saved_data):
    transforms = compute_transforms(joint_angles)
    new_distances = []

    for item in saved_data["min_distances"]:
        link_a, link_b, _, T_a_to_link, T_b_to_link = item
        T_a_to_link = np.array(T_a_to_link)
        T_b_to_link = np.array(T_b_to_link)

        T_a = transform_sphere(transforms[link_a], T_a_to_link)
        T_b = transform_sphere(transforms[link_b], T_b_to_link)

        pos_a = T_a[:3, 3]
        pos_b = T_b[:3, 3]
        dist = np.linalg.norm(pos_a - pos_b)
        new_distances.append([link_a, link_b, round(float(dist), 6)])

    return new_distances

if __name__ == "__main__":
    with open("output_results.json", "r") as f:
        saved_data = json.load(f)

    joint_angles = get_joint_angles_from_input()
    results = compute_new_distances(joint_angles, saved_data)

    print("\n新姿态下每对小球之间的距离：")
    for link_a, link_b, d in results:
        print(f"{link_a} 与 {link_b} 的距离为 {d:.6f} 米")

    new_data = {"new_distances": results}
    with open("new_output_results.json", "w") as f:
        json.dump(new_data, f, indent=2)

    print("\n已保存到 new_output_results.json")
