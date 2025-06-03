import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import threading
from fk_generator import FKGenerator
import numpy as np
from collision_detection import detect_collisions  # 添加碰撞检测导入
import csv
import os

class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')

        self.subscription_joint = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        self.prev_positions = None
        self.change_count = 0
        self.recording = True

        self.joint_names = []
        self.joint_data = {}
        self.x_data = []
        self.ee_positions = []
        self.min_collision_distances = {}  # 记录碰撞对最小距离

        urdf_path = '/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf'
        self.fk_solver = FKGenerator(urdf_path)
        self.fk_func = self.fk_solver.fk_func_with_base_quat()

        input_thread = threading.Thread(target=self.input_listener, daemon=True)
        input_thread.start()

        self.get_logger().info('Recording started. Type "stop" and press Enter to finish.')

    def input_listener(self):
        while True:
            user_input = input()
            if user_input.strip().lower() == 'stop':
                self.recording = False
                self.get_logger().info('Stop command received. Plotting data...')
                self.plot_data()
                rclpy.shutdown()
                break

    def joint_callback(self, msg):
        if not self.recording:
            return

        if self.prev_positions is None:
            self.prev_positions = list(msg.position)
            self.joint_names = list(msg.name)
            for joint in self.joint_names:
                self.joint_data[joint] = []
            return

        if list(msg.position) != self.prev_positions:
            self.change_count += 1
            self.prev_positions = list(msg.position)
            self.x_data.append(self.change_count)

            print(f"\n变化次数 {self.change_count}:")
            for idx, joint in enumerate(msg.name):
                pos = msg.position[idx]
                print(f"  {joint}: {pos:.4f}")
                if joint not in self.joint_data:
                    self.joint_data[joint] = []
                self.joint_data[joint].append(pos)

            for joint in self.joint_data:
                if len(self.joint_data[joint]) < len(self.x_data):
                    self.joint_data[joint].append(self.joint_data[joint][-1])

            fk_joint_order = ['position_base_x', 'position_base_y', 'position_base_theta'] + [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
            ]
            joint_map = dict(zip(msg.name, msg.position))
            q_full = [joint_map.get(name, 0.0) for name in fk_joint_order]

            q_np = np.array(q_full)
            ee_pose = self.fk_func(q_np).full().flatten()
            self.ee_positions.append(ee_pose[:3])

            collisions = detect_collisions(q_full)
            for link1, link2, dist in collisions:
                key = tuple(sorted([link1, link2]))
                if key not in self.min_collision_distances:
                    self.min_collision_distances[key] = dist
                else:
                    self.min_collision_distances[key] = min(self.min_collision_distances[key], dist)

    def plot_data(self):
        plt.figure()
        for joint, positions in self.joint_data.items():
            plt.plot(self.x_data, positions, label=joint)
        plt.xlabel('Change Count')
        plt.ylabel('Joint Position')
        plt.title('Joint States after Stop')
        plt.legend()
        plt.grid(True)
        plt.show()

        print("\n累计位移（每个关节位置变化总和）:")
        displacement_map = {}
        for joint, positions in self.joint_data.items():
            displacement = sum(abs(positions[i] - positions[i - 1]) for i in range(1, len(positions)))
            displacement_map[joint] = displacement
            print(f"  {joint}: {displacement:.4f}")

            # 平均位移统计
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        base_joints = ['position_base_x', 'position_base_y', 'position_base_theta']

        arm_displacements = [displacement_map[j] for j in arm_joints if j in displacement_map]
        base_displacements = [displacement_map[j] for j in base_joints if j in displacement_map]

        if arm_displacements:
            avg_arm_disp = sum(arm_displacements) / len(arm_displacements)
            print(f"\n机械臂关节平均位移: {avg_arm_disp:.4f}")
        else:
            print("\n机械臂关节平均位移: 数据不足")

        if base_displacements:
            avg_base_disp = sum(base_displacements) / len(base_displacements)
            print(f"底盘关节平均位移: {avg_base_disp:.4f}")
        else:
            print("底盘关节平均位移: 数据不足")

        if len(self.ee_positions) > 1:
            total_distance = 0.0
            for i in range(1, len(self.ee_positions)):
                p1 = np.array(self.ee_positions[i - 1])
                p2 = np.array(self.ee_positions[i])
                dist = np.linalg.norm(p2 - p1)
                total_distance += dist
            print(f"\n末端执行器累计路程: {total_distance:.4f} 米")
        else:
            print("\n末端执行器累计路程: 数据不足")

        print("\n碰撞对最小距离统计：")
        for (link1, link2), dist in self.min_collision_distances.items():
            print(f"  {link1} - {link2}: {dist:.4f} 米")
        threshold = 0.02
        count_below_threshold = sum(1 for d in self.min_collision_distances.values() if d < threshold)
        print(f"\n小于 {threshold:.3f} 米 的碰撞对数量: {count_below_threshold}")

        # 设置保存路径
        save_path = "data.csv"
        file_exists = os.path.isfile(save_path)

        # 示例信息（你可以将这些替换为动态输入或参数）
        goal_x, goal_y = 0.121, 0.602
        algorithm = "BiTRRT"
        failures = 1

        # 构造输出数据
        row = {
            "目标点": f"(x={goal_x:.3f}, y={goal_y:.3f})",
            "算法": algorithm,
            "规划失败次数": failures,
            "机械臂平均位移": avg_arm_disp if arm_displacements else "N/A",
            "底盘平均位移": avg_base_disp if base_displacements else "N/A",
            "末端执行器累计路程": total_distance if len(self.ee_positions) > 1 else "N/A",
            "危险碰撞对数(<0.02m)": count_below_threshold
        }

        # 写入文件（追加或新建）
        with open(save_path, mode='a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

        print(f"\n结果已保存到 {save_path}")

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRecorder()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
