import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


from mpc_planner_v8 import MPCPlannerV8
import numpy as np
import casadi as ca
import time
import sys
import matplotlib.pyplot as plt  # 确认在文件头引入
import distance_count_bind

class MoveItMPCPlannerV8(Node):
    def __init__(self, target_pose=None):
        super().__init__('moveit_mpc_planner_v8')

        # 订阅当前 joint_states
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 发布 MPC 轨迹
        self.trajectory_publisher = self.create_publisher(
            JointState,
            "/mpc_joint_trajectory",
            10
        )

        # URDF 路径（根据你的工程路径）
        urdf_path = '/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf'

        # 初始化 MPC 优化器
        self.mpc = MPCPlannerV8(self.get_logger(), urdf_path)

        self.current_state = None

        # 外部传进来的目标 pose
        if target_pose is None:
            self.get_logger().warn("未传入 target_pose，将使用默认值")
            self.target_pose = self.default_target_pose()
        else:
            self.target_pose = self.build_pose_stamped_from_dict(target_pose)

        self.mpc_executed = False

        # 打印目标信息
        self.get_logger().info(
            f"目标末端位姿: ({self.target_pose.pose.position.x}, "
            f"{self.target_pose.pose.position.y}, {self.target_pose.pose.position.z}), "
            f"四元数: ({self.target_pose.pose.orientation.x}, {self.target_pose.pose.orientation.y}, "
            f"{self.target_pose.pose.orientation.z}, {self.target_pose.pose.orientation.w})"
        )

        self.get_logger().info("MoveIt MPC Planner V8 启动完成")

    def build_pose_stamped_from_dict(self, pose_dict):
        """从外部传入的字典数据创建 PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = pose_dict['position']['x']
        pose.pose.position.y = pose_dict['position']['y']
        pose.pose.position.z = pose_dict['position']['z']
        pose.pose.orientation.x = pose_dict['orientation']['x']
        pose.pose.orientation.y = pose_dict['orientation']['y']
        pose.pose.orientation.z = pose_dict['orientation']['z']
        pose.pose.orientation.w = pose_dict['orientation']['w']
        return pose

    def default_target_pose(self):
        """默认目标位姿（用于调试，不传入时使用）"""
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.0363116
        pose.pose.position.y = 0.204015
        pose.pose.position.z = 0.401
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def joint_state_callback(self, msg):
        if self.mpc_executed:
            return  # 已经执行过，不重复执行

        joint_names = msg.name
        joint_positions = np.array(msg.position)

        joint_map = dict(zip(joint_names, joint_positions))

        # 当前关节状态（base + arm），共9自由度
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


        # 目标位姿
        target_ee_pose = ca.DM([
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z,
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w
        ])


        # 设定MPC参考目标
        self.mpc.set_reference(target_ee_pose, self.current_state)

        # 初始控制输入
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)

        # 计算当前关节对的距离和梯度
        # ✅ **创建 `joint_values`，在 index=6 (夹爪) 位置补默认值**
        joint_values = np.zeros(10)  # 10个关节值
        joint_values[:3] = self.current_state.full().flatten()[:3]  # 复制 base 关节
        joint_values[3:6] = self.current_state.full().flatten()[3:6]  # 复制机械臂关节
        joint_values[7:] = self.current_state.full().flatten()[6:]  # 复制剩余关节
        joint_values[6] = 0.0  # **补充夹爪默认值**

        link_pairs, gradient, joint_distances = distance_count_bind.compute_distance_gradient(
            joint_values)
        # ✅ 检查梯度矩阵的形状
        # ✅ 确保 gradient 是 NumPy 数组
        if isinstance(gradient, list):
            gradient = np.array(gradient)
        # **输出梯度矩阵形状**
        print(f"Gradient Matrix Shape: {len(gradient)} x {len(gradient[0])}\n")

        # 传递给 MPC
        self.mpc.set_collision_constraints(link_pairs, gradient, joint_distances)

        # ✅ 确保碰撞代价确实影响了优化
        print(f"\n🎯 MPC 碰撞代价总和: {self.mpc.collision_cost_fn}")


        self.mpc.build_cost_function()

        # self.mpc.set_limit()
        self.mpc.set_optimize_option()

        # 计算最优控制
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

    target_pose = None  # 默认值

    # 解析命令行参数
    if len(sys.argv) == 8:
        print("[main] 接收到7个目标位姿参数，使用外部传参作为 target_pose")

        target_pose = {
            'position': {
                'x': float(sys.argv[1]),
                'y': float(sys.argv[2]),
                'z': float(sys.argv[3]),
            },
            'orientation': {
                'x': float(sys.argv[4]),
                'y': float(sys.argv[5]),
                'z': float(sys.argv[6]),
                'w': float(sys.argv[7]),
            }
        }

    else:
        print("[main] 未接收到目标位姿参数，使用默认 target_pose")
        print("正确格式示例: python3 moveit_mpc_planner_v6.py pos_x pos_y pos_z quat_x quat_y quat_z quat_w")

    # 初始化节点
    node = MoveItMPCPlannerV8(target_pose)

    # Spin 循环
    rclpy.spin(node)

    # 退出
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()