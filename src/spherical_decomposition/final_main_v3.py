import time
from final_symbolic_transform_v3 import get_symbolic_transforms_of_closest_spheres
from collections import defaultdict
from final_output_results_v3 import run_pipeline

def main():
    # joint_angles = [0.47940996415137255, 0.5813352815466089, -0.004847190152301571, 0.10006863051001078, -0.5007966155853011, -1.0659151586802347, -0.7243363221368784, -1.5757282543337767, -0.7469683802905275, 0.01]
    joint_angles = [0,0,0,0,0,0,0,0,0,0.001]

    start_time = time.time()  # 记录开始时间
    # 数值计算部分
    min_dists, pairs = run_pipeline(joint_angles)

    # 符号计算并输出
    result = get_symbolic_transforms_of_closest_spheres(min_dists)

    # 汇总每个 Link 对的碰撞次数
    link_pair_counts = defaultdict(int)
    for item in result:
        link_a, link_b = item[0]
        link_pair_counts[(link_a, link_b)] += 1

    # 输出碰撞信息
    for pair, count in link_pair_counts.items():
        print(f"Link对: {pair}，碰撞点数量: {count}")

    for item in result:
        print("Link对:", item[0])
        print("最短距离（数值）:", item[1])
        print()

    end_time = time.time()  # 记录结束时间
    print(f"程序运行时间: {end_time - start_time:.4f} 秒")

if __name__ == "__main__":
    main()


