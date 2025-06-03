import time
from final_symbolic_transform import get_symbolic_transforms_of_closest_spheres
from final_output_results import run_pipeline

def main():
    joint_angles = [0.5, 0.5, 0.5,  0, -1, 0 ,0, -1, 1, 0.01]


    start_time = time.time()  # 记录开始时间
    # 数值计算部分
    min_dists, pairs = run_pipeline(joint_angles)
    print(pairs)

    # 符号计算并输出
    result = get_symbolic_transforms_of_closest_spheres("output_results.json")
    end_time = time.time()  # 记录结束时间
    print(f"程序运行时间: {end_time - start_time:.4f} 秒")

    for item in result:
        print("Link对:", item[0])
        print("最短距离（数值）:", item[1])
        print("最短距离（含代数表达）:", item[2])
        print()



if __name__ == "__main__":
    main()
