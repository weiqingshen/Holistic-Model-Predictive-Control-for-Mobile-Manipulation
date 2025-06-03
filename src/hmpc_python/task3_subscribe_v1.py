import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
import threading
from fk_generator import FKGenerator
import numpy as np
import os
import pickle

class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')

        self.subscription_joint = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        self.subscription_target = self.create_subscription(
            PoseArray,
            '/arm_pose',
            self.target_callback,
            10)

        self.prev_positions = None
        self.change_count = 0
        self.recording = True

        self.joint_names = []
        self.joint_data = {}
        self.x_data = []
        self.ee_positions = []
        self.target_positions = []
        self.target_differences = []

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

            for idx, joint in enumerate(msg.name):
                pos = msg.position[idx]
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
            ee_pos = ee_pose[:3]
            self.ee_positions.append(ee_pos)

            if self.target_positions:
                current_target = self.target_positions[-1]
                diff = np.linalg.norm(ee_pos[:2] - current_target)
                self.target_differences.append(diff)
                print(f"当前目标偏差: {diff:.4f} 米")

    def target_callback(self, msg):
        if msg.poses:
            pose1 = msg.poses[0]
            self.target_positions.append(np.array([pose1.position.x, pose1.position.y]))

    def plot_data(self):
        if len(self.ee_positions) > 1:
            # 创建 output 文件夹（如果不存在）
            output_dir = 'output'
            os.makedirs(output_dir, exist_ok=True)

            # 整理数据
            data_to_save = {
                'ee_positions': self.ee_positions,
                'target_positions': self.target_positions,
                'target_differences': self.target_differences,
                'time_steps': self.x_data  # 新增保存时间步
            }

            # 保存为 Pickle 文件
            with open(os.path.join(output_dir, 'trajectory_data.pkl'), 'wb') as f:
                pickle.dump(data_to_save, f)

            # ========== 以下是你的原始绘图代码 ==========
            ee_x = [pos[0] for pos in self.ee_positions]
            ee_y = [pos[1] for pos in self.ee_positions]

            target_x = [pos[0] for pos in self.target_positions]
            target_y = [pos[1] for pos in self.target_positions]

            # 图1：XY轨迹
            plt.figure()
            plt.plot(ee_x, ee_y, label='End Effector Path', linestyle='-', color='orangered', linewidth=2.5)
            if target_x and target_y:
                plt.plot(target_x, target_y, label='Target Path', linestyle='-', color='black', linewidth=2.5)
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title('End-Effector and Target Trajectory')
            plt.grid(True)
            plt.axis('equal')
            plt.legend(loc='upper right')
            plt.savefig(os.path.join(output_dir, 'trajectory_plot.png'), dpi=300)
            plt.show()

            # 图2：目标偏差随时间变化
            if self.target_differences:
                plt.figure()
                plt.plot(range(len(self.target_differences)), self.target_differences)
                plt.xlabel('Time Step')
                plt.ylabel('Position Error (m)')
                plt.title('Target Deviation Over Time')
                plt.grid(True)
                plt.savefig(os.path.join(output_dir, 'error_over_time.png'), dpi=300)
                plt.show()

                avg_diff = sum(self.target_differences) / len(self.target_differences)
                print(f"\n平均目标偏差: {avg_diff:.4f} 米")
        else:
            print("\n末端执行器轨迹数据不足，无法绘图。")

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
