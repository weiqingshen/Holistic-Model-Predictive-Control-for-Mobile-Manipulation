import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import casadi as ca
import numpy as np
from mpc_planner import MPCPlanner  # 这里使用新类名

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # 订阅 MoveIt 计算的目标位姿
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mpc_target_pose',
            self.target_pose_callback,
            10)

        # 发布轨迹点
        self.trajectory_publisher = self.create_publisher(Path, '/mpc_trajectory', 10)

        # 初始化 MPC
        self.mpc = MPCPlanner()
        self.state_now = self.mpc.state_init
        self.t0 = 0
        self.u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        self.X0 = ca.repmat(self.mpc.state_init, 1, self.mpc.N + 1)

        self.get_logger().info("MPC 计算节点启动！")

    def target_pose_callback(self, msg):
        """收到目标位姿后，运行 MPC 计算轨迹"""
        self.get_logger().info(f"收到目标位姿: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")

        # 构造目标点数据
        target_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        target_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                              msg.pose.orientation.w]

        # 构造 MPC 目标参考路径
        arg_p_arm = ca.DM.zeros(7 * (self.mpc.N + 1))
        for k in range(self.mpc.N + 1):
            for j in range(7):
                arg_p_arm[k * 7 + j] = target_position[j] if j < 3 else target_orientation[j - 3]

        # 设定参考轨迹
        p = ca.vertcat(self.state_now, self.mpc.state_target, arg_p_arm)
        self.mpc.set_reference(p)
        self.mpc.set_x0(self.X0, self.u0)

        # 运行 MPC 计算
        self.X0, self.u0 = self.mpc.get_states_and_control()
        self.t0, self.state_now, self.u0 = self.mpc.shift_timestep(self.mpc.step_horizon, self.t0, self.state_now,
                                                                   self.u0)

        # 发布轨迹点
        self.publish_trajectory(self.X0)

    def publish_trajectory(self, X0):
        """发布 MPC 计算的轨迹点序列"""
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(X0.shape[1]):
            pose = PoseStamped()
            pose.pose.position.x = float(X0[0, i])
            pose.pose.position.y = float(X0[1, i])
            pose.pose.position.z = 0.0  # 假设基座运动是 2D
            path_msg.poses.append(pose)

        self.trajectory_publisher.publish(path_msg)
        self.get_logger().info("已发布 MPC 轨迹点")


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
