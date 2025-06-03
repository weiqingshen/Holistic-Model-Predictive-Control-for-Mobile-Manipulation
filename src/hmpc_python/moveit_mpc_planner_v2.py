import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from nav_msgs.msg import Path
import numpy as np
import casadi as ca
from mpc_planner_v2 import MPCPlanner  # 确保你已经创建了 MPCPlanner 类


class MoveItMPCPlanner(Node):
    def __init__(self):
        """初始化 MoveIt MPC 规划节点"""
        super().__init__('moveit_mpc_planner')

        # 订阅机械臂的关节状态
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',  # ROS 2 话题名称
            self.joint_state_callback,  # 回调函数，接收到消息时调用
            10  # 队列大小
        )

        # 创建 IK 逆运动学求解的服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /compute_ik 服务...')

        # 发布 MPC 计算出的轨迹
        self.trajectory_publisher = self.create_publisher(Path, '/mpc_trajectory', 10)

        # 创建 MPC 规划器对象，并传入 ROS 2 的 logger
        self.mpc = MPCPlanner(self.get_logger())

        # 存储当前机械臂的状态
        self.current_state = None
        self.mpc_executed = False  # 变量用于确保 MPC 只执行一次

        # 设置目标位姿
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
            # 如果 MPC 已经执行过，则不再重复执行
            return

        # self.get_logger().info("接收到新的关节状态，开始轨迹规划...")

        # 解析关节状态（将 ROS 消息中的关节角度转换为 numpy 数组）
        joint_names = msg.name  # 关节名称
        joint_positions = np.array(msg.position)  # 关节角度

        # **提取关节角度，确保顺序正确**
        joint_map = dict(zip(joint_names, joint_positions))  # 将 name 和 position 组成字典

        # 按照正确的顺序构造当前状态
        self.current_state = ca.DM([
            joint_map.get("position_base_x", 0.0),  # 机器人底盘 X 位置
            joint_map.get("position_base_y", 0.0),  # 机器人底盘 Y 位置
            joint_map.get("position_base_theta", 0.0),  # 机器人底盘角度 Theta
            joint_map.get("joint1", 0.0),  # 关节 1
            joint_map.get("joint2", 0.0),  # 关节 2
            joint_map.get("joint3", 0.0),  # 关节 3
            joint_map.get("joint4", 0.0),  # 关节 4
            joint_map.get("joint5", 0.0),  # 关节 5
            joint_map.get("joint6", 0.0)  # 关节 6
        ])

        self.get_logger().info(f"构造的当前状态向量: {self.current_state}")

        self.get_logger().info("调用 IK 计算目标关节角度...")
        self.request_ik_solution()

    def request_ik_solution(self):
        """请求 MoveIt 逆运动学（IK）求解，计算目标关节角度"""
        self.get_logger().info("请求 IK 计算目标关节角度...")

        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm_move"  # 机械臂运动组
        request.ik_request.pose_stamped = self.target_pose  # 目标位姿
        request.ik_request.pose_stamped.header.frame_id = "virtual_base_x"  # 基座坐标系

        # 发送异步请求并在完成时回调 ik_callback
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        """IK 计算完成后回调"""
        try:
            response = future.result()
            if response and response.error_code.val == response.error_code.SUCCESS:
                # 解析返回的关节数据
                joint_names = response.solution.joint_state.name
                joint_positions = response.solution.joint_state.position

                # 直接取前 9 个关节的角度
                target_joint_angles = joint_positions[:9]

                self.get_logger().info(f"筛选后的目标关节角度: {target_joint_angles}")

                # 运行 MPC 计算最优轨迹
                self.run_mpc(target_joint_angles)
            else:
                self.get_logger().error('IK 计算失败')
        except Exception as e:
            self.get_logger().error(f"IK 计算错误: {e}")

    def run_mpc(self, target_joint_angles):
        """调用 MPC 进行轨迹规划，并发布最优轨迹"""
        self.get_logger().info("开始运行 MPC 计算...")

        if self.current_state is None:
            self.get_logger().error("未接收到关节状态，无法计算轨迹！")
            return

        self.get_logger().info(f"目标关节角度: {target_joint_angles}")

        # 目标状态向量，包含目标位置和目标关节角度
        target_state = ca.DM([
            target_joint_angles[0],  # position_base_x
            target_joint_angles[1],  # position_base_y
            target_joint_angles[2],  # position_base_theta
            target_joint_angles[3],  # joint1
            target_joint_angles[4],  # joint2
            target_joint_angles[5],  # joint3
            target_joint_angles[6],  # joint4
            target_joint_angles[7],  # joint5
            target_joint_angles[8]  # joint6
        ])

        # 设定 MPC 参考轨迹
        self.mpc.set_reference(ca.vertcat(self.current_state, target_state))

        # 初始化控制输入（全 0）
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))

        # 状态变量初始化
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)

        # 设定 MPC 初始优化变量
        self.mpc.set_x0(X0, u0)

        self.get_logger().info("调用 MPC 求解器...")
        X_opt = self.mpc.get_states_and_control()

        if X_opt is None:
            self.get_logger().error("MPC 计算失败！")
            return

        self.get_logger().info("MPC 计算完成，发布轨迹...")
        self.publish_trajectory(X_opt)

        # 标记 MPC 规划已完成，防止重复执行
        self.mpc_executed = True

    def publish_trajectory(self, X_opt):
        """发布 MPC 计算出的最优轨迹"""
        self.get_logger().info("准备发布 MPC 轨迹...")

        path_msg = Path()
        path_msg.header.frame_id = "world"  # 参考坐标系
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 遍历 MPC 计算出的轨迹点并加入 Path 消息
        for i in range(X_opt.shape[1]):
            pose = PoseStamped()
            pose.pose.position.x = float(X_opt[0, i])
            pose.pose.position.y = float(X_opt[1, i])
            pose.pose.position.z = 0.0  # 仅用于 2D 轨迹
            path_msg.poses.append(pose)

        # 发布轨迹
        self.trajectory_publisher.publish(path_msg)
        self.get_logger().info("MPC 轨迹已发布！")


def main(args=None):
    """ROS 2 节点主函数"""
    rclpy.init(args=args)
    node = MoveItMPCPlanner()
    rclpy.spin(node)  # 进入 ROS 2 事件循环
    node.destroy_node()
    rclpy.shutdown()  # 关闭 ROS 2 节点


if __name__ == '__main__':
    main()
