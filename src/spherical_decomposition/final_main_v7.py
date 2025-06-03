import time
# from .final_symbolic_transform_v7 import get_symbolic_transforms_of_closest_spheres
# from .final_output_results_v7 import run_pipeline
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from final_symbolic_transform_v7 import get_symbolic_transforms_of_closest_spheres
from final_output_results_v7 import run_pipeline
from collections import defaultdict
import casadi as ca

class CollisionSubscriberNode(Node):
    def __init__(self, joint_angles):
        super().__init__('collision_subscriber_node')
        self.joint_angles = joint_angles
        self.cylinder_info = None

        # 订阅圆柱体位姿信息，只订阅一次
        self.subscription = self.create_subscription(
            PoseArray,
            'arm_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('订阅了话题: [arm_pose]')

    def pose_callback(self, msg: PoseArray):
        # 提取圆柱体的位姿信息（假设它在第一个Pose中）
        pose = msg.poses[0]
        cyl_center = (pose.position.x, pose.position.y, 0.035)  # 假设z值为常数0.035
        cyl_radius = 0.025  # 圆柱体半径
        cyl_height = 0.07  # 圆柱体高度

        # 调用符号碰撞分析，获取包含 cylinder 的结果
        X = ca.SX.sym('X', 9, 31)  # 假设时间步为31

        # 运行一次完整分析
        result = run_pipeline(
            X,
            self.joint_angles,
            cyl_center=cyl_center,
            cyl_radius=cyl_radius,
            cyl_height=cyl_height,
        )

        # 仅打印包含 cylinder 的结果
        for item in result:
            # link_a, link_b = item[0]
            # if 'cylinder' not in [link_a, link_b]:
            #     continue  # 只处理涉及 cylinder 的对
            link_pairs=item[0]
            dist_value = item[1]
            dist_expr = item[2]
            t_idx = item[3]

            print(link_pairs)
            print(f"最短距离（数值）: {dist_value}")
            print(f"符号距离表达式: {dist_expr}")
            print(t_idx)
            print()

        # 一旦接收到数据，取消订阅以避免不必要的回调
        self.subscription.destroy()
        self.get_logger().info("圆柱体信息已接收，停止订阅")

    def get_cylinder_info(self):
        return self.cylinder_info

def main():
    joint_angles = [0.09609847051367734, 0.266221868586861, -0.038475298984106114, 0.07489409330486552, -1.0842132629829275, 0.4215157614324569, 0.00019106326868573712, -0.6627050575913715, 0.0360991421411537,0.001]

    # joint_angles = [0.0, 1.0, -0.0,
    #                 0.0, -0.0, -0.0,
    #                 -0.0, -0.0, -0.0, 0.001]
    X = ca.SX.sym('X', 9, 31)  # 创建符号变量（9维关节角，31个时间步）

    # 初始化 ROS2 节点
    rclpy.init()
    node = CollisionSubscriberNode(joint_angles)

    # 等待圆柱体信息，只需要等一次
    while node.get_cylinder_info() is None:
        node.get_logger().info("等待圆柱体位姿数据...")
        rclpy.spin_once(node)
        break
    # # 获取圆柱体信息
    # cylinder_info = node.get_cylinder_info()
    #
    # start_time = time.time()  # 记录开始时间
    #
    # # 数值计算部分
    # min_dists, pairs = run_pipeline(joint_angles)
    #
    # # 符号计算并输出
    # result = get_symbolic_transforms_of_closest_spheres(min_dists, X)
    # # print(result)
    #
    # # 汇总每个 Link 对在每个时间步的碰撞次数
    # link_pair_time_counts = defaultdict(int)
    # for item in result:
    #     link_a, link_b = item[0]
    #     t_idx = item[3]
    #     link_pair_time_counts[(link_a, link_b, t_idx)] += 1
    #
    # # 输出详细碰撞次数
    # print("\n=== 每个 Link 对在各时间步的碰撞次数 ===")
    # for (link_a, link_b, t_idx), count in link_pair_time_counts.items():
    #     print(f"[时间步 {t_idx}] Link对: ({link_a}, {link_b}) 碰撞点数量: {count}")
    # print()
    #
    # # 输出每个碰撞点的符号表达式
    # print("=== 每个碰撞点的符号距离表达式 ===")
    # for item in result:
    #     link_a, link_b = item[0]
    #     dist_value = item[1]
    #     dist_expr = item[2]
    #     t_idx = item[3]
    #
    #     print(f"[时间步 {t_idx}] Link对: {link_a}, {link_b}")
    #     print(f"最短距离（数值）: {dist_value}")
    #     print(f"符号距离表达式: {dist_expr}")
    #     print()
    #
    # end_time = time.time()  # 记录结束时间
    # print(f"程序运行时间: {end_time - start_time:.4f} 秒")


if __name__ == "__main__":
    main()


def symbolic_collision_analysis(joint_angles, X, sphere_json_path="link_spheres_relative_v2.json"):
    """
    封装函数：输入CasADi符号变量X，返回每对Link之间的所有危险碰撞点的数量和符号距离表达式列表。

    Args:
        X (CasADi.MX): 9个关节角度和状态变量组成的CasADi符号矩阵。
        sphere_json_path (str): 球心配置文件路径

    Returns:
        tuple:
            - dict: {(link_a, link_b): 碰撞点个数}
            - list: 每项为 [ [link_a, link_b], 数值距离, 符号表达式 ]
    """
    start_time = time.time()

    # 运行您先前定义的函数来获取最短距离数据
    min_dists, _ = run_pipeline(joint_angles, sphere_json_path=sphere_json_path, output_path=None)


    # 使用新的符号表示计算碰撞点信息
    result = get_symbolic_transforms_of_closest_spheres(min_dists, X)

    # 统计每对 link 的碰撞点数目
    link_pair_counts = defaultdict(int)
    for item in result:
        link_a, link_b = item[0]
        link_pair_counts[(link_a, link_b)] += 1

    # 输出碰撞信息
    for pair, count in link_pair_counts.items():
        print(f"Link对: {pair}，碰撞点数量: {count}")

    end_time = time.time()
    print(f"symbolic_collision_analysis 执行耗时: {end_time - start_time:.4f} 秒")

    # 返回每对链接的碰撞点数目及计算的符号结果
    return link_pair_counts, result