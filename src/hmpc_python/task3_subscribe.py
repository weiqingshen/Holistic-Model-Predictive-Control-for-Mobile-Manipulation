import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import threading
from fk_generator import FKGenerator
import numpy as np
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

            # print(f"\n变化次数 {self.change_count}:")
            for idx, joint in enumerate(msg.name):
                pos = msg.position[idx]
                # print(f"  {joint}: {pos:.4f}")
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
            print(ee_pose)
            print("\n")
            self.ee_positions.append(ee_pose[:3])



    def plot_data(self):
        if len(self.ee_positions) > 1:
            x_coords = [pos[0] for pos in self.ee_positions]
            y_coords = [pos[1] for pos in self.ee_positions]

            plt.figure()
            plt.plot(x_coords, y_coords, marker='o')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.title('End-Effector Trajectory in XY Plane')
            plt.grid(True)
            plt.axis('equal')
            plt.show()
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
