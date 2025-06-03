import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
import numpy as np
import casadi as ca
from mpc_planner import MPCPlanner  # 确保你已经创建了 MPCPlanner 类

class MoveItMPCPlanner(Node):
    def __init__(self):
        """初始化 MoveIt MPC 规划节点"""
        super().__init__('moveit_mpc_planner')
        # 订阅机械臂的关节状态
        self.joint_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        # 创建 IK 逆运动学求解的服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /compute_ik 服务...')
        # 创建 MPC 规划器对象，并传入 ROS 2 的 logger
        self.mpc = MPCPlanner(self.get_logger())
        # 存储当前机械臂的状态
        self.current_state = None
        self.mpc_executed = False  # 确保 MPC 只执行一次
        self.get_logger().info("MoveIt MPC 规划器启动，等待关节状态更新...")

    def joint_state_callback(self, msg):
        """回调函数：接收机械臂关节状态，并调用 MPC 进行规划"""
        if self.mpc_executed:
            return
        # 解析关节状态
        joint_map = dict(zip(msg.name, msg.position))
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
        self.get_logger().info(f"当前关节状态: {self.current_state}")
        self.request_ik_solution()

    def request_ik_solution(self):
        """请求 MoveIt 逆运动学（IK）求解，计算目标关节角度"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm_move"
        request.ik_request.pose_stamped.header.frame_id = "virtual_base_x"
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        """IK 计算完成后回调"""
        try:
            response = future.result()
            if response and response.error_code.val == response.error_code.SUCCESS:
                target_joint_angles = response.solution.joint_state.position[:9]
                self.run_mpc(target_joint_angles)
            else:
                self.get_logger().error("IK 计算失败")
        except Exception as e:
            self.get_logger().error(f"IK 计算错误: {e}")

    def run_mpc(self, target_joint_angles):
        """调用 MPC 计算最优轨迹"""
        if self.current_state is None:
            self.get_logger().error("未接收到关节状态，无法计算轨迹！")
            return

        # 设置标志，确保 MPC 只执行一次
        if self.mpc_executed:
            return

        target_state = ca.DM(target_joint_angles)
        # 创建机械臂的目标轨迹 `p_arm`
        p_arm = ca.DM.zeros(6 * (self.mpc.N + 1))  # 6 个关节角，复制到 N+1 个时间步
        for k in range(self.mpc.N + 1):
            for j in range(6):
                p_arm[k * 6 + j] = target_joint_angles[j]
        # 设定 MPC 参考轨迹
        self.mpc.set_reference(ca.vertcat(self.current_state, target_state), p_arm)
        # 初始化控制输入（全 0）
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)
        self.get_logger().info("调用 MPC 求解器...")
        X_opt = self.mpc.get_states_and_control()
        if X_opt is None:
            self.get_logger().error("MPC 计算失败！")
            return
        self.get_logger().info("MPC 计算完成！")
        self.mpc_executed = True  # 标记 MPC 已执行

def main(args=None):
    rclpy.init(args=args)
    node = MoveItMPCPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()