import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from mpc_planner_v6 import MPCPlannerV6
import numpy as np
import casadi as ca
import time
import sys
import matplotlib.pyplot as plt  # 确认在文件头引入

class MoveItMPCPlannerV6(Node):
    def __init__(self):
        super().__init__('moveit_mpc_planner_v6')

        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.trajectory_publisher = self.create_publisher(JointState, "/mpc_joint_trajectory", 10)

        urdf_path = '/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf'
        self.mpc = MPCPlannerV6(self.get_logger(), urdf_path)

        self.current_state = None
        self.target_pose = self.define_target_pose()

        self.mpc_executed = False
        self.get_logger().info("MoveIt MPC Planner V6 启动完成")

    def define_target_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.0363116
        pose.pose.position.y = 0.204015
        pose.pose.position.z = 0.401
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        self.get_logger().info(f"目标末端位姿: ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}), "
                               f"({pose.pose.orientation.x}, {pose.pose.orientation.y}, "
                               f"{pose.pose.orientation.z}, {pose.pose.orientation.w})")
        return pose

    def joint_state_callback(self, msg):
        if self.mpc_executed:
            return

        joint_names = msg.name
        joint_positions = np.array(msg.position)

        joint_map = dict(zip(joint_names, joint_positions))

        self.current_state = ca.DM([
            joint_map.get("position_base_x", 0.0),
            joint_map.get("position_base_y", 0.0),
            joint_map.get("position_base_theta", 0.0),
            joint_map.get("joint1", 0.0),
            joint_map.get("joint2", 0.0),
            joint_map.get("joint3", 0.0),
            joint_map.get("joint4", 0.0),
            joint_map.get("joint5", 0.0),
            joint_map.get("joint6", 0.0)
        ])

        self.run_mpc()

    def run_mpc(self):
        if self.mpc_executed or self.current_state is None:
            return

        self.mpc_executed = True

        target_ee_pose = ca.DM([
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z,
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w
        ])

        self.mpc.set_reference(target_ee_pose, self.current_state)

        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)

        X_opt = self.mpc.get_states_and_control()

        if X_opt is not None:
            self.publish_trajectory(X_opt)

    def publish_trajectory(self, X_opt):
        """发布 MPC 计算出的最优轨迹，同时展示轨迹图"""

        # 时间步
        timesteps = np.arange(X_opt.shape[1])

        # Joint 名称
        joint_labels = [
            "position_base_x", "position_base_y", "position_base_theta",
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]

        # === 绘制轨迹图 ===
        plt.figure(figsize=(12, 6))
        for i in range(9):  # 9个控制量
            plt.plot(timesteps, X_opt[i, :].full().flatten(), label=joint_labels[i])

        plt.xlabel("Time step")
        plt.ylabel("Joint position / Base pose")
        plt.title("MPC Optimal Trajectory")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # === 发布 JointState 消息 ===
        for i in range(X_opt.shape[1]):  # 遍历时间步
            joint_trajectory_msg = JointState()
            joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            joint_trajectory_msg.header.frame_id = "world"
            joint_trajectory_msg.name = joint_labels

            # 发送当前时间步对应的 joint 状态
            joint_trajectory_msg.position = [float(X_opt[j, i]) for j in range(9)]

            self.trajectory_publisher.publish(joint_trajectory_msg)

            self.get_logger().info(f"发布第 {i} 帧：{joint_trajectory_msg.position}")

            time.sleep(0.1)  # 控制发送频率

        self.get_logger().info("MPC 轨迹已全部发布！")

        time.sleep(1)
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = MoveItMPCPlannerV6()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
