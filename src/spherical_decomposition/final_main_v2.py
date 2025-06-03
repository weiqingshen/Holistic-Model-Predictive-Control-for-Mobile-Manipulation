import time
from .final_symbolic_transform_v2 import get_symbolic_transforms_of_closest_spheres

from .final_output_results_v2 import run_pipeline

def main():
    joint_angles = [0.47940996415137255, 0.5813352815466089, -0.004847190152301571, 0.10006863051001078, -0.5007966155853011, -1.0659151586802347, -0.7243363221368784, -1.5757282543337767, -0.7469683802905275, 0.01]


    start_time = time.time()  # 记录开始时间
    # 数值计算部分
    min_dists, pairs = run_pipeline(joint_angles)

    # 符号计算并输出
    result = get_symbolic_transforms_of_closest_spheres(min_dists)

    for item in result:
        print("Link对:", item[0])
        print("最短距离（数值）:", item[1])
        print("最短距离（含代数表达）:", item[2])
        print()

    end_time = time.time()  # 记录结束时间
    print(f"程序运行时间: {end_time - start_time:.4f} 秒")





if __name__ == "__main__":
    main()


def symbolic_collision_analysis(joint_angles, sphere_json_path="link_spheres_relative_v2.json"):
    """
    封装好的主函数：输入关节角，返回每对最近连杆球心的数值距离与符号表达式

    Args:
        joint_angles (list): 长度为10的关节角列表
        sphere_json_path (str): 球体相对位置的JSON路径，默认在工作目录下

    Returns:
        list: 每项是 [ [link_a, link_b], 数值最短距离, 符号最短距离表达式 ]
    """
    start_time = time.time()
    min_dists, _ = run_pipeline(joint_angles, sphere_json_path=sphere_json_path, output_path=None)
    result = get_symbolic_transforms_of_closest_spheres(min_dists)
    end_time = time.time()
    print(f"symbolic_collision_analysis 执行耗时: {end_time - start_time:.4f} 秒")
    return result