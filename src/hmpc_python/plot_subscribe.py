import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import threading

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
