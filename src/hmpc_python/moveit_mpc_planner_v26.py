import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from fk_generator import FKGenerator  # 依然基于之前写好的
from mpc_planner_v26 import MPCPlannerV26
import numpy as np
import casadi as ca
import time
import sys
import matplotlib.pyplot as plt  # 确认在文件头引入
import distance_count_bind
import sys
sys.path.append("/home/fins/myrobot_move/src")
from spherical_decomposition.final_main_v5 import symbolic_collision_analysis

class MoveItMPCPlannerv26(Node):
    def __init__(self, target_pose=None):
        super().__init__('moveit_mpc_planner_v26')
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
        self.mpc = MPCPlannerV26(self.get_logger(), self.urdf_path,self.valid_steps )

        self.current_state = None
        self.X_opt_all = None  # 用于存储所有的轨迹


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

        self.get_logger().info("MoveIt MPC Planner v26 启动完成")

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
        pose.pose.position.x = 0.0836233
        pose.pose.position.y = 0.599458
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

        # 当前状态（9自由度）转为 list
        current_state_list = self.current_state.full().flatten().tolist()

        # 添加默认夹爪开合值 g，比如设为 0.01
        joint_angles = current_state_list + [0.001]

        # 执行符号碰撞分析
        link_pair_counts, symbolic_result = symbolic_collision_analysis(joint_angles,self.mpc.X)

        # 输出碰撞点统计信息（可选）
        for pair, count in link_pair_counts.items():
            self._logger.info(f"Link对: {pair}，碰撞点数量: {count}")

        # 设置符号碰撞代价
        # self.mpc.set_symbolic_collision_constraints(symbolic_result, danger_threshold=0.04, a=100, b=800)
        # self.mpc.set_symbolic_collision_constraints(symbolic_result, danger_threshold=0.08, a=100, b=500)
        #
        custom_params = {
            ("link3", "obstacle"): {'danger_threshold': 0.06, 'a': 0.1, 'b': 100},
            ("obstacle", "link3"): {'danger_threshold': 0.06, 'a': 0.1, 'b': 100},
            ("obstacle", "link4"): {'danger_threshold': 0.06, 'a': 0.1, 'b': 100},
            ("link4", "obstacle"): {'danger_threshold': 0.06, 'a': 0.1, 'b': 100},
            ("obstacle", "link_hand"): {'danger_threshold': 0.06, 'a': 0.1, 'b': 100},
            ("link_hand", "obstacle"): {'danger_threshold': 0.06, 'a': 0.1, 'b': 100}
        }
        self.mpc.set_symbolic_collision_constraints(symbolic_result, danger_threshold=0.06, a=0.01, b=100,custom_params=custom_params)
        # # 动态调整 R 矩阵
        # self.mpc.adjust_R_matrix(ee_pose_np, target_ee_pose)
        self.mpc.adjust_Q_execute(ee_pose_np,target_ee_pose)
        self.mpc.build_cost_function()

        self.mpc.set_optimize_option()
        # 设定MPC参考目标
        self.mpc.set_reference(target_ee_pose, self.current_state)

        # 初始控制输入
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        self.mpc.set_x0(X0, u0)

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
        # threshold_position_error = 0.02  # 位置误差阈值 (可以调整)
        # threshold_quaternion_error = 0.2  # 四元数误差阈值 (可以调整)
        threshold_position_error = 0.001  # 位置误差阈值 (可以调整)
        threshold_quaternion_error = 0.03  # 四元数误差阈值 (可以调整)
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

        # # 时间步
        # timesteps = np.arange(self.X_opt_all.shape[1])
        # # === 绘制轨迹图 ===
        # plt.figure(figsize=(12, 6))
        # for i in range(9):  # 9个关节
        #     plt.plot(timesteps, self.X_opt_all[i, :].full().flatten(), label=joint_labels[i])
        #
        # plt.xlabel("Time step")
        # plt.ylabel("Joint position / Base pose")
        # plt.title("MPC Optimal Trajectory")
        # plt.legend()
        # plt.grid(True)
        # plt.tight_layout()
        # plt.show()

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

        # 当前状态（9自由度）转为 list
        current_state_list = self.current_state.full().flatten().tolist()

        # 添加默认夹爪开合值 g，比如设为 0.01
        joint_angles = current_state_list + [0.001]
        # 执行符号碰撞分析
        # 执行符号碰撞分析
        link_pair_counts, symbolic_result = symbolic_collision_analysis(joint_angles,self.mpc.X)

        # 输出碰撞点统计信息（可选）
        for pair, count in link_pair_counts.items():
            self._logger.info(f"Link对: {pair}，碰撞点数量: {count}")

        # 设置符号碰撞代价
        # self.mpc.set_symbolic_collision_constraints(symbolic_result, danger_threshold=0.08, a=100, b=500)
        # custom_params = {
        #     ("link_left", "cylinder"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("cylinder", "link_left"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("cylinder", "link_right"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("link_right", "cylinder"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("cylinder", "link_hand"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("link_hand", "cylinder"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("link3", "obstacle"): {'danger_threshold': 0.08, 'a': 0.5, 'b': 200},
        #     ("obstacle", "link3"): {'danger_threshold': 0.08, 'a': 0.5, 'b': 200},
        #     ("obstacle", "link4"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("link4", "obstacle"): {'danger_threshold': 0.08, 'a': 0.3, 'b': 200},
        #     ("obstacle", "link_hand"): {'danger_threshold': 0.08, 'a': 0.1, 'b': 200},
        #     ("link_hand", "obstacle"): {'danger_threshold': 0.08, 'a': 0.1, 'b': 200}
        # }
        custom_params = {
            ("link3", "obstacle"): {'danger_threshold': 0.06, 'a': 0.2, 'b': 120},
            ("obstacle", "link3"): {'danger_threshold': 0.06, 'a': 0.2, 'b': 120},
            ("obstacle", "link4"): {'danger_threshold': 0.06, 'a': 0.2, 'b': 120},
            ("link4", "obstacle"): {'danger_threshold': 0.06, 'a': 0.2, 'b': 120},
            ("obstacle", "link_hand"): {'danger_threshold': 0.07, 'a': 0.1, 'b': 150},
            ("link_hand", "obstacle"): {'danger_threshold': 0.07, 'a': 0.1, 'b': 150}
        }
        self.mpc.set_symbolic_collision_constraints(symbolic_result, danger_threshold=0.06, a=0.01, b=100,custom_params=custom_params)
        # # 动态调整 R 矩阵
        # self.mpc.adjust_R_matrix(ee_pose_np, target_ee_pose)


        self.mpc.adjust_Q_execute(ee_pose_np,target_ee_pose)
        self.mpc.build_cost_function()
        self.mpc.set_optimize_option()
        # 设定MPC参考目标
        self.mpc.set_reference(target_ee_pose, self.current_state)

        # 初始控制输入
        u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        X0 = ca.repmat(self.current_state, 1, self.mpc.N + 1)
        # print(X0)
        self.mpc.set_x0(X0, u0)

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

    # def publish_trajectory(self, X_opt):
    #     """发布 MPC 计算出的最优轨迹，同时展示轨迹图"""
    #
    #     # 时间步
    #     timesteps = np.arange(X_opt.shape[1])
    #
    #     # Joint 名称
    #     joint_labels = [
    #         "position_base_x", "position_base_y", "position_base_theta",
    #         "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    #     ]
    #
    #     # === 发布 JointState 消息 ===
    #     for i in range(X_opt.shape[1]):  # 遍历时间步
    #         joint_trajectory_msg = JointState()
    #         joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
    #         joint_trajectory_msg.header.frame_id = "world"
    #         joint_trajectory_msg.name = joint_labels
    #
    #         # 发送当前时间步对应的 joint 状态
    #         joint_trajectory_msg.position = [float(X_opt[j, i]) for j in range(9)]
    #
    #         self.trajectory_publisher.publish(joint_trajectory_msg)
    #
    #         self.get_logger().info(f"发布第 {i} 帧：{joint_trajectory_msg.position}")
    #
    #         # time.sleep(0.001)  # 控制发送频率
    #
    #     self.get_logger().info("MPC 轨迹已全部发布！")
    #
    #     # time.sleep(1)
    #
    #     # === 绘制轨迹图 ===
    #     plt.figure(figsize=(12, 6))
    #     for i in range(9):  # 9个控制量
    #         plt.plot(timesteps, X_opt[i, :].full().flatten(), label=joint_labels[i])
    #
    #     plt.xlabel("Time step")
    #     plt.ylabel("Joint position / Base pose")
    #     plt.title("MPC Optimal Trajectory")
    #     plt.legend()
    #     plt.grid(True)
    #     plt.tight_layout()
    #     plt.show()
    #
    #     # === 输出末端执行器坐标对比 ===
    #     final_state = X_opt[:, -1]  # 获取最终状态
    #
    #     # 获取末端执行器位置 + 四元数
    #     urdf_path = '/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf'
    #     fk_generator = FKGenerator(urdf_path)
    #     fk_func = fk_generator.fk_func_with_base_quat()  # 获取 FK 函数
    #
    #     ee_pose = fk_func(final_state)  # 包含 xyz + quaternion
    #     ee_pose_np = ee_pose.full().flatten()  # 转成 numpy 一维数组
    #
    #     # 拆分 xyz + quaternion
    #     x, y, z = ee_pose_np[0:3]
    #     qx, qy, qz, qw = ee_pose_np[3:7]
    #
    #     # 目标位姿中的数据
    #     target_pos = self.target_pose.pose.position
    #     target_ori = self.target_pose.pose.orientation
    #
    #     # 输出对比信息
    #     self.get_logger().info("\n=== 末端执行器坐标对比 ===")
    #     self.get_logger().info(f"当前末端位置:    x={x:.4f}, y={y:.4f}, z={z:.4f}")
    #     self.get_logger().info(f"当前末端四元数: qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}")
    #     self.get_logger().info(f"目标末端位置:    x={target_pos.x:.4f}, y={target_pos.y:.4f}, z={target_pos.z:.4f}")
    #     self.get_logger().info(f"目标末端四元数: qx={target_ori.x:.4f}, qy={target_ori.y:.4f}, "
    #                            f"qz={target_ori.z:.4f}, qw={target_ori.w:.4f}")
    #
    #     rclpy.shutdown()
    #     sys.exit(0)
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
    node = MoveItMPCPlannerv26(target_pose)

    while rclpy.ok():
        node.check_exit_flag()  # 检查是否需要退出
        if node.exit_flag:  # 如果 exit_flag 为 True，退出 spin 循环
            print("退出循环，exit_flag 已设置为 True")
            break  # 跳出循环，立即退出
        print("回调")
        rclpy.spin_once(node)

    # 退出
    # 退出
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()