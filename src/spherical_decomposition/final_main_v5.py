import time
from .final_symbolic_transform_v5 import get_symbolic_transforms_of_closest_spheres
from .final_output_results_v5 import run_pipeline
# from final_symbolic_transform_v5 import get_symbolic_transforms_of_closest_spheres
# from final_output_results_v5 import run_pipeline
from collections import defaultdict
import casadi as ca


def main():
    # joint_angles = [0.07940996415137255, 0.0813352815466089, -0.004847190152301571,
    #                 0.10006863051001078, -0.5007966155853011, -1.0659151586802347,
    #                 -0.7243363221368784, -1.5757282543337767, -0.7469683802905275, 0.001]
    joint_angles = [0.0, 1.0, -0.0,
                    0.0, -0.0, -0.0,
                    -0.0, -0.0, -0.0, 0.001]
    X = ca.SX.sym('X', 9, 31)  # 创建符号变量（9维关节角，31个时间步）

    start_time = time.time()  # 记录开始时间

    # 数值计算部分
    min_dists, pairs = run_pipeline(joint_angles)

    # 符号计算并输出
    result = get_symbolic_transforms_of_closest_spheres(min_dists, X)
    # print(result)
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