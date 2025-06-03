import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetPositionFK
from moveit_msgs.msg import PlanningScene
from geometry_msgs.msg import PoseStamped

class MoveItListener(Node):
    def __init__(self):
        super().__init__('moveit_listener')

        # 订阅 /joint_states 话题，获取当前关节角度
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # 订阅 /planning_scene 话题，获取规划场景信息
        self.scene_subscriber = self.create_subscription(
            PlanningScene,
            '/planning_scene',
            self.scene_callback,
            10)

        # 创建获取规划场景的服务客户端
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        while not self.scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /get_planning_scene 服务...')

        # 创建 IK 逆运动学求解的服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /compute_ik 服务...')

        # 创建 FK 正运动学求解的服务客户端
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /compute_fk 服务...')

        # 用于控制日志打印频率的变量
        self.last_print_time = self.get_clock().now()
        self.print_interval = 2.0  # 每2秒打印一次

        self.get_logger().info("MoveIt 2 监听器已启动！")

    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        current_time = self.get_clock().now()
        if (current_time - self.last_print_time).nanoseconds / 1e9 > self.print_interval:
            joint_positions = dict(zip(msg.name, msg.position))
            self.get_logger().info(f'当前关节状态: {joint_positions}')
            self.last_print_time = current_time  # 更新上次打印时间

    def scene_callback(self, msg):
        """处理规划场景消息"""
        obstacles = len(msg.world.collision_objects)
        self.get_logger().info(f'接收到规划场景: {obstacles} 个障碍物')

    def request_planning_scene(self):
        """请求完整的规划场景"""
        request = GetPlanningScene.Request()
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info(f'获取到完整规划场景，包含 {len(response.scene.world.collision_objects)} 个障碍物')
        else:
            self.get_logger().error('获取规划场景失败')

    def request_ik(self):
        """请求逆运动学求解"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = "manipulator"  # 根据你的机械臂名称修改
        request.ik_request.pose_stamped.header.frame_id = "base_link"

        # 设置目标位姿
        request.ik_request.pose_stamped.pose.position.x = 0.5
        request.ik_request.pose_stamped.pose.position.y = 0.2
        request.ik_request.pose_stamped.pose.position.z = 0.3
        request.ik_request.pose_stamped.pose.orientation.w = 1.0  # 单位四元数

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info(f'IK 计算成功: {response.solution.joint_state.position}')
            # 调用 FK 计算末端执行器位姿
            self.request_fk(response.solution.joint_state)
        else:
            self.get_logger().error('IK 计算失败')

    def request_fk(self, joint_state):
        """请求正运动学求解，计算末端执行器位姿"""
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"  # 参考坐标系
        request.fk_link_names = ["link_hand"]  # 末端执行器的链接名称
        request.robot_state.joint_state = joint_state  # 传入关节状态

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            if response.error_code.val == response.error_code.SUCCESS:
                pose = response.pose_stamped[0].pose  # 获取末端执行器的位姿
                self.get_logger().info(f'末端执行器位姿: 位置=({pose.position.x}, {pose.position.y}, {pose.position.z}), 方向=({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})')
            else:
                self.get_logger().error(f'FK 计算失败: {response.error_code.val}')
        else:
            self.get_logger().error('FK 服务调用失败')

def main(args=None):
    rclpy.init(args=args)
    node = MoveItListener()

    # 主动请求场景和 IK
    node.request_planning_scene()
    node.request_ik()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()