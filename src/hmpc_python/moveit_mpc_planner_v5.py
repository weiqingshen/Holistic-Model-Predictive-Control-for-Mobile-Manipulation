import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
import numpy as np
import casadi as ca
from mpc_planner_v5 import MPCPlanner  # 确保你已经创建了 MPCPlanner 类
import matplotlib.pyplot as plt
import sys
import time

class MoveItMPCPlanner(Node):
    def __init__(self):
        """初始化 MoveIt MPC 规划节点"""
        super().__init__('moveit_mpc_planner')

        # 订阅机械臂的关节状态
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 创建 IK 逆运动学求解的服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /compute_ik 服务...')

        self.trajectory_publisher = self.create_publisher(JointState, "/mpc_joint_trajectory", 10)

        # 创建 MPC 规划器对象，并传入 ROS 2 的 logger
        self.mpc = MPCPlanner(self.get_logger())

        # 存储当前机械臂的状态
        self.current_state = None
        self.mpc_executed = False  # 防止 MPC 执行多次

        # 目标位姿
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0363116
        self.target_pose.pose.position.y = 0.204015
        self.target_pose.pose.position.z = 0.401
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 1.0
        self.target_pose.pose.orientation.z = 0.0
        self.target_pose.pose.orientation.w = 0.0

        self.get_logger().info("MoveIt MPC 规划器启动，等待关节状态更新...")

    def joint_state_callback(self, msg):
        """回调函数：接收机械臂关节状态，并调用 MPC 进行规划"""
        if self.mpc_executed:
            self.get_logger().info("MPC 已执行，忽略本次回调")
            return

        # 解析关节状态
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

        self.get_logger().info(f"构造的当前状态向量: {self.current_state}")

        self.get_logger().info("调用 IK 计算目标关节角度...")
        self.request_ik_solution()

    def request_ik_solution(self):
        """请求 MoveIt 逆运动学（IK）求解"""
        if self.mpc_executed:
            self.get_logger().info("MPC 已执行，忽略 IK 计算")
            return

        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm_move"
        request.ik_request.pose_stamped = self.target_pose
        request.ik_request.pose_stamped.header.frame_id = "virtual_base_x"

        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        """IK 计算完成后回调"""
        if self.mpc_executed:
            self.get_logger().info("MPC 已执行，忽略本次 IK 计算")
            return

        try:
            response = future.result()
            if response and response.error_code.val == response.error_code.SUCCESS:
                joint_positions = response.solution.joint_state.position
                target_joint_angles = joint_positions[:9]

                self.get_logger().info(f"筛选后的目标关节角度: {target_joint_angles}")

                # 运行 MPC 计算
                self.run_mpc(target_joint_angles)
            else:
                self.get_logger().error('IK 计算失败')
        except Exception as e:
            self.get_logger().error(f"IK 计算错误: {e}")

    def run_mpc(self, target_joint_angles):
        """调用 MPC 进行轨迹规划"""
        if self.mpc_executed:
            self.get_logger().info("MPC 已执行，忽略本次调用")
            return

        self.mpc_executed = True  # 确保 MPC 只运行一次

        if self.current_state is None:
            self.get_logger().error("未接收到关节状态，无法计算轨迹！")
            return

        target_state = ca.DM(target_joint_angles)

        self.mpc.set_reference(ca.vertcat(self.current_state, target_state))

        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)

        self.get_logger().info("调用 MPC 求解器...")
        X_opt = self.mpc.get_states_and_control()

        if X_opt is None:
            self.get_logger().error("MPC 计算失败！")
            return

        self.get_logger().info("MPC 计算完成，发布轨迹...")
        self.publish_trajectory(X_opt)

    def publish_trajectory(self, X_opt):
        """发布 MPC 计算出的最优轨迹"""

        timesteps = np.arange(X_opt.shape[1])
        plt.figure(figsize=(10, 6))

        joint_labels = [
            "position_base_x", "position_base_y", "position_base_theta",
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]

        for i in range(9):
            plt.plot(timesteps, X_opt[i, :].full().flatten(), label=joint_labels[i])

        plt.xlabel("time")
        plt.ylabel("joint")
        plt.title("MPC motion_planning")
        plt.legend()
        plt.grid()
        plt.show()

        joint_trajectory_msg = JointState()
        joint_trajectory_msg.header.frame_id = "world"
        joint_trajectory_msg.name = joint_labels

        print(X_opt.shape)
        print(X_opt.shape[1])
        for i in range(X_opt.shape[1]):  # i 是时间步
            joint_trajectory_msg = JointState()
            joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            joint_trajectory_msg.header.frame_id = "world"
            joint_trajectory_msg.name = joint_labels

            joint_trajectory_msg.position = [float(X_opt[j, i]) for j in range(9)]  # 正常访问

            self.trajectory_publisher.publish(joint_trajectory_msg)

            self.get_logger().info(f"发布第 {i} 帧：{joint_trajectory_msg.position}")

            time.sleep(0.1)

        self.get_logger().info("MPC 轨迹已发布！等待订阅者处理...")

        # 等待1秒，确保消息被处理
        time.sleep(1)

        # 退出 ROS 2 进程
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    """ROS 2 节点主函数"""
    rclpy.init(args=args)
    node = MoveItMPCPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
