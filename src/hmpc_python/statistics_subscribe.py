import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import threading
from fk_generator import FKGenerator
import numpy as np

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

        self.recording = True  # 一开始是接收的

        self.joint_names = []
        self.joint_data = {}  # {joint_name: [positions]}
        self.x_data = []

        # 假设你设置了 urdf 路径
        urdf_path =  '/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf'
        self.fk_solver = FKGenerator(urdf_path)
        self.fk_func = self.fk_solver.fk_func_with_base_quat()
        self.ee_positions = []  # 存储末端执行器位置

        # 开一个后台线程监听用户输入
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

            # 指定 base 和机械臂关节的顺序，确保与 FK 计算一致
            fk_joint_order = ['position_base_x', 'position_base_y', 'position_base_theta'] + [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
            ]
            joint_map = dict(zip(msg.name, msg.position))

            # 用 0.0 补全缺失的 joint
            q_full = [joint_map.get(name, 0.0) for name in fk_joint_order]

            q_np = np.array(q_full)
            ee_pose = self.fk_func(q_np)[0].full().flatten()  # xyz + quat
            self.ee_positions.append(ee_pose[:3])

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

        # 计算每个关节的累计位移
        print("\n累计位移（每个关节位置变化总和）:")
        for joint, positions in self.joint_data.items():
            displacement = sum(abs(positions[i] - positions[i - 1]) for i in range(1, len(positions)))
            print(f"  {joint}: {displacement:.4f}")

        # 计算末端执行器累计路程
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
