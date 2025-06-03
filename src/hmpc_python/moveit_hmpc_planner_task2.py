import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from fk_generator import FKGenerator  # 依然基于之前写好的
from hmpc_planner_task2 import HMPCPlanner_task2
import numpy as np
import casadi as ca
import time
import sys
import matplotlib.pyplot as plt  # 确认在文件头引入
import distance_count_bind
import pandas as pd
import os
class MoveIthMPCPlanner_task2(Node):
    def __init__(self, target_pose=None):
        super().__init__('moveit_mpc_planner_task2')
        self.exit_flag = False  # 退出标志

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
        self.urdf_path = '/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf'
        self.fk_generator = FKGenerator(self.urdf_path)
        self.fk_func = self.fk_generator.fk_func_with_base_quat()  # 获取 FK 函数
        self.valid_steps = 1  # 有效的执行步数
        # 初始化 MPC 优化器
        self.mpc = HMPCPlanner_task2(self.get_logger(), self.urdf_path,self.valid_steps )

        self.current_state = None
        self.X_opt_all = None  # 用于存储所有的轨迹
        self.X_opt_follow=None


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

        self.get_logger().info("MoveIt HMPC Planner task2 启动完成")

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
        pose.pose.position.y = 1.204015
        pose.pose.position.z = 0.301
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
        ee_pose = self.fk_func(self.current_state)  # 计算末端执行器的实际位姿
        ee_pose_np = ee_pose.full().flatten()  # 转为 NumPy 数组方便计算
        # 动态调整 R 矩阵
        self.mpc.adjust_R_matrix(ee_pose_np, target_ee_pose)
        self.mpc.build_cost_function()

        self.mpc.set_optimize_option()
        # 设定MPC参考目标
        self.mpc.set_reference(target_ee_pose, self.current_state)

        # 初始控制输入
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)

        # 计算当前关节对的距离和梯度
        # ✅ **创建 `joint_values`，在 index=6 (夹爪) 位置补默认值**
        joint_values = np.zeros(10)  # 10个关节值 算
        joint_values[:3] = self.current_state.full().flatten()[:3]  # 复制 base 关节
        joint_values[3:6] = self.current_state.full().flatten()[3:6]  # 复制机械臂关节
        joint_values[7:] = self.current_state.full().flatten()[6:]  # 复制剩余关节
        joint_values[6] = 0.0  # **补充夹爪默认值**

        # 这里先暂时不添加避障
        # link_pairs, gradient, joint_distances = distance_count_bind.compute_distance_gradient(
        #     joint_values)
        # # ✅ 检查梯度矩阵的形状
        # # ✅ 确保 gradient 是 NumPy 数组
        # if isinstance(gradient, list):
        #     gradient = np.array(gradient)
        # # **输出梯度矩阵形状**
        # print(f"Gradient Matrix Shape: {len(gradient)} x {len(gradient[0])}\n")
        #
        # # 传递给 MPC
        # self.mpc.set_collision_constraints(link_pairs, gradient, joint_distances)
        #
        # # ✅ 确保碰撞代价确实影响了优化
        # print(f"\n🎯 MPC 碰撞代价总和: {self.mpc.collision_cost_fn}")
        # self.mpc.build_cost_function()
        # self.mpc.set_limit()
        # self.mpc.set_optimize_option()

        # 计算最优控制
        X_opt = self.mpc.get_states_and_control()

        if X_opt is not None:
            # 将X_opt转换为NumPy数组并添加到X_opt_all

            self.X_opt_all=X_opt

            # 将前step_size步添加到X_opt_all
            self.X_opt_all = self.X_opt_all[:,:self.valid_steps]  # 可选，按需要取前valid_steps步
            # 调用发布函数
            self.publish_trajectory()


    def publish_trajectory(self):
        """发布 MPC 计算出的最优轨迹，同时展示轨迹图"""

        # 时间步
        # timesteps = np.arange(self.X_opt_all.shape[1])

        # Joint 名称
        joint_labels = [
            "position_base_x", "position_base_y", "position_base_theta",
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]

        # === 发布 JointState 消息 ===
        # step_size = 10  # 每次发布的步数
        threshold_position_error = 0.001  # 位置误差阈值 (可以调整)
        threshold_quaternion_error = 0.05  # 四元数误差阈值 (可以调整)

        n = 0

        while True:
            # 将 X_opt_all 转换为 NumPy 数组
            # X_opt_all_np = np.array(self.X_opt_all)  # Convert the list of DM to a numpy array
            # print(self.X_opt_all)
            for i in range(n, min(n + self.valid_steps, self.X_opt_all.shape[1])):  # 发布每个时间步的数据
                joint_trajectory_msg = JointState()
                joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
                joint_trajectory_msg.header.frame_id = "world"
                joint_trajectory_msg.name = joint_labels

                # 发送当前时间步对应的 joint 状态
                joint_trajectory_msg.position = [float(self.X_opt_all[j, i]) for j in range(9)]

                self.trajectory_publisher.publish(joint_trajectory_msg)

                self.get_logger().info(f"发布第 {i} 帧：{joint_trajectory_msg.position}")

            # === 计算当前末端执行器坐标误差 ===
            print(n)
            final_state = self.X_opt_all[:, n + self.valid_steps - 1]  # 获取第 step_size 步的末端执行器状态


            ee_pose = self.fk_func(final_state)  # 包含 xyz + quaternion
            ee_pose_np = ee_pose.full().flatten()  # 转成 numpy 一维数组

            # 拆分 xyz + quaternion
            x, y, z = ee_pose_np[0:3]
            qx, qy, qz, qw = ee_pose_np[3:7]

            # 目标位姿中的数据
            target_pos = self.target_pose.pose.position
            target_ori = self.target_pose.pose.orientation

            # 计算位置误差（欧几里得距离）
            position_error = np.sqrt((x - target_pos.x) ** 2 + (y - target_pos.y) ** 2 + (z - target_pos.z) ** 2)

            # 计算四元数误差（四元数之间的角度差）
            quaternion_error = self.quaternion_error([qx, qy, qz, qw], [target_ori.x, target_ori.y, target_ori.z, target_ori.w])

            # 输出对比信息
            self.get_logger().info("\n=== 末端执行器坐标对比 ===")
            self.get_logger().info(f"当前末端位置:    x={x:.4f}, y={y:.4f}, z={z:.4f}")
            self.get_logger().info(f"当前末端四元数: qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}")
            self.get_logger().info(f"目标末端位置:    x={target_pos.x:.4f}, y={target_pos.y:.4f}, z={target_pos.z:.4f}")
            self.get_logger().info(f"目标末端四元数: qx={target_ori.x:.4f}, qy={target_ori.y:.4f}, "
                                   f"qz={target_ori.z:.4f}, qw={target_ori.w:.4f}")

            self.get_logger().info(f"位置误差: {position_error:.4f}, 四元数误差: {quaternion_error:.4f}")

            # 如果误差满足要求，则退出循环
            if position_error < threshold_position_error and quaternion_error < threshold_quaternion_error:
                self.get_logger().info("误差小于阈值，停止优化！")
                break

            # 如果误差过大，重新进行 MPC 规划
            self.get_logger().info("误差过大，重新进行 MPC 规划...")
            # 从上次的轨迹末尾作为新的起始状态
            self.current_state = self.X_opt_all[:, -1]

            self.run_mpc_dynamic()

            n += self.valid_steps  # 更新步数

        self.exit_flag = True  # 退出标志
        self.run_mpc_follow()

        # 发布 X_opt_follow
        for i in range(self.X_opt_follow.shape[1]):  # 发布 follow 轨迹的数据
            joint_trajectory_msg = JointState()
            joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            joint_trajectory_msg.header.frame_id = "world"
            joint_trajectory_msg.name = joint_labels

            joint_trajectory_msg.position = [float(self.X_opt_follow[j, i]) for j in range(9)]

            self.trajectory_publisher.publish(joint_trajectory_msg)

            self.get_logger().info(f"发布第 {i} 帧：{joint_trajectory_msg.position}")

        # 时间步
        # 时间步
        print("X_opt_all shape:", self.X_opt_all.shape)
        print("X_opt_follow shape:", self.X_opt_follow.shape)

        # 确定保存路径
        output_dir = os.path.join(os.path.dirname(__file__), 'output')
        os.makedirs(output_dir, exist_ok=True)

        # 关键点和时间步
        key_points = np.array([
            [-0.319, -0.681, 0.234, -0.002, -0.461, -0.371],
            [-0.690, -0.612, 0.204, -0.003, -0.421, -0.391],
            [-1.834, -0.402, 0.111, -0.008, -0.298, -0.454],
            [-2.666, -0.250, 0.043, -0.012, -0.209, -0.500],
            [-3.112, -0.168, 0.007, -0.014, -0.160, -0.524],
        ])

        key_point_steps = [5,8, 15, 20, 25]  # 对应时间步


        X_opt_data = ca.horzcat(self.X_opt_all, self.X_opt_follow)
        # 时间步
        # 改为真实时间（秒）
        # 转换时间步为秒
        delta_t = 0.1
        timesteps = np.arange(X_opt_data.shape[1]) * delta_t

        # === 绘制轨迹图 ===
        # 关节标签

        # 颜色映射，选择一个更好看的色板
        colors = plt.cm.plasma(np.linspace(0, 1, 9))

        # plt.figure(figsize=(12, 6))
        # for i in range(9):  # 9个关节
        #     plt.plot(timesteps, X_opt_data[i, :].full().flatten(), label=joint_labels[i], color=colors[i])
        plt.figure(figsize=(12, 6))
        for i in range(9):  # 9个关节
            unit = "(m)" if i < 2 else "(rad)"
            plt.plot(timesteps, X_opt_data[i, :].full().flatten(),
                     label=f"{joint_labels[i]} {unit}", color=colors[i])

        # 标注关键点（仅后6个关节）
        for i, step in enumerate(key_point_steps):
            for j in range(3, 9):  # 仅标注后6个关节
                time_point = (step + self.X_opt_all.shape[1]) * delta_t
                plt.scatter(time_point, key_points[i, j - 3], color=colors[j], marker='o')  # 使用对应关节的颜色
                # plt.text(step + self.X_opt_all.shape[1], key_points[i, j - 3], f'({key_points[i, j - 3]:.2f})',
                #          color='black', fontsize=9)

        plt.xlabel("Time (s)")

        plt.ylabel("Joint/Base position")
        plt.title("MPC Optimal Trajectory")
        plt.legend(loc='lower left')
        plt.grid(True)
        plt.tight_layout()
        # plt.show()
        # 保存图像
        plt.savefig(os.path.join(output_dir, 'trajectory.png'))
        plt.show()

        plt.figure(figsize=(12, 6))
        for i in range(9):
            velocity = np.diff(X_opt_data[i, :].full().flatten()) / 0.1  # 除以0.1s得到真实速度
            unit = "(m/s)" if i < 2 else "(rad/s)"
            plt.plot(timesteps[1:], velocity, label=f"{joint_labels[i]} {unit}", color=colors[i])

        plt.xlabel("Time (s)")
        plt.ylabel("Joint/Base velocity ")
        plt.title("MPC Joint Velocity Visualization")
        plt.legend(loc='lower left')
        plt.grid(True)
        plt.tight_layout()
        # plt.show()
        # 保存图像
        plt.savefig(os.path.join(output_dir, 'velocity.png'))
        plt.show()

        plt.figure(figsize=(12, 6))
        for i in range(9):
            acceleration = np.diff(X_opt_data[i, :].full().flatten(), n=2) / (delta_t ** 2)  # 除以(Δt)^2
            unit = "(m/s²)" if i < 2 else "(rad/s²)"
            plt.plot(timesteps[2:], acceleration, label=f"{joint_labels[i]} {unit}", color=colors[i])

        plt.xlabel("Time (s)")
        plt.ylabel("Joint/Base acceleration ")
        plt.title("MPC Joint Acceleration Visualization")
        plt.legend(loc='lower left')
        plt.grid(True)
        plt.tight_layout()
        # plt.show()

        # 保存图像
        plt.savefig(os.path.join(output_dir, 'acceleration.png'))
        plt.show()

        # 提取最后30步的数据
        last_30_steps = X_opt_data[:, -30:].full()

        # 将数据转换为DataFrame
        df = pd.DataFrame(last_30_steps.T, columns=joint_labels)

        # 确定保存路径
        output_dir = os.path.join(os.path.dirname(__file__), 'output')
        os.makedirs(output_dir, exist_ok=True)  # 如果output目录不存在，自动创建

        output_file = os.path.join(output_dir, 'mpc_last_30_steps.xlsx')

        # 保存为Excel文件
        df.to_excel(output_file, index=False)

        self.get_logger().info(f"最后30步已保存为 Excel 文件：{output_file}")

        self.get_logger().info("MPC 轨迹已全部发布！")
        return



    def quaternion_error(self, q1, q2):
        """计算两个四元数之间的误差，返回角度误差"""
        # 四元数点积：q1.q2
        dot_product = np.dot(q1, q2)
        # 四元数误差 (范围为 0 到 1)
        return 2 * np.arccos(min(1.0, abs(dot_product)))  # 计算四元数的角度差

    def run_mpc_dynamic(self):
        print("dynamic")
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

        # 计算当前末端执行器位姿
        ee_pose = self.fk_func(self.current_state)  # 计算末端执行器的实际位姿
        ee_pose_np = ee_pose.full().flatten()  # 转为 NumPy 数组方便计算

        # 计算末端执行器位置与目标位置的距离
        # position_error = np.sqrt((ee_pose_np[0] - target_ee_pose[0]) ** 2 +
        #                          (ee_pose_np[1] - target_ee_pose[1]) ** 2 +
        #                          (ee_pose_np[2] - target_ee_pose[2]) ** 2)

        # 动态调整 R 矩阵
        self.mpc.adjust_R_matrix(ee_pose_np, target_ee_pose)
        self.mpc.build_cost_function()
        self.mpc.set_optimize_option()
        # 设定MPC参考目标
        self.mpc.set_reference(target_ee_pose, self.current_state)

        # 初始控制输入
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)

        # 计算当前关节对的距离和梯度
        # ✅ **创建 `joint_values`，在 index=6 (夹爪) 位置补默认值**
        joint_values = np.zeros(10)  # 10个关节值 算
        joint_values[:3] = self.current_state.full().flatten()[:3]  # 复制 base 关节
        joint_values[3:6] = self.current_state.full().flatten()[3:6]  # 复制机械臂关节
        joint_values[7:] = self.current_state.full().flatten()[6:]  # 复制剩余关节
        joint_values[6] = 0.0  # **补充夹爪默认值**


        # 计算最优控制
        X_opt = self.mpc.get_states_and_control()

        print(self.X_opt_all)
        if X_opt is not None:
            self.X_opt_all = ca.horzcat(self.X_opt_all, X_opt[:, 1:self.valid_steps+1])

        print(self.X_opt_all)

    def check_exit_flag(self):
        """检查退出标志并退出"""
        if self.exit_flag:
            sys.exit(0)


    def run_mpc_follow(self):
        print("follow")

        target_state = ca.DM([
            self.X_opt_all[0,-1],
            self.X_opt_all[1,-1],
            self.X_opt_all[2,-1],
            -3.141,
            -0.154,
            -0.028,
            -0.001,
            -0.181,
            -0.5
        ])
        print(target_state)
        target_ee_pose = self.fk_func(target_state)
        print(target_ee_pose)
        # 计算当前末端执行器位姿
        ee_pose = self.fk_func(self.current_state)  # 计算末端执行器的实际位姿
        ee_pose_np = ee_pose.full().flatten()  # 转为 NumPy 数组方便计算

        # 动态调整 R 矩阵
        # self.mpc.adjust_R_matrix(ee_pose_np, target_ee_pose)
        self.mpc.follow_adjust_matrix(ee_pose_np, target_ee_pose)
        self.mpc.build_cost_function_follow()
        self.mpc.set_optimize_option()
        # 设定MPC参考目标
        self.mpc.set_reference(target_ee_pose, self.current_state)

        # 初始控制输入
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)

        # 计算最优控制
        X_opt = self.mpc.get_states_and_control()

        self.X_opt_follow=X_opt


        print(self.X_opt_follow)


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
    node = MoveIthMPCPlanner_task2(target_pose)

    while rclpy.ok():
        node.check_exit_flag()  # 检查是否需要退出
        if node.exit_flag:  # 如果 exit_flag 为 True，退出 spin 循环
            print("退出循环，exit_flag 已设置为 True")
            break  # 跳出循环，立即退出
        print("回调")
        rclpy.spin_once(node)

    # 退出
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()