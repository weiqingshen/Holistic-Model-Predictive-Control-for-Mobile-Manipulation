import distance_count_bind
import time
# 关节角度
joint_values = [0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0]


# 记录开始时间
start_time = time.time()
# 计算关节之间的最小距离（使用 C++ 端缓存）
distances = distance_count_bind.compute_joint_distances_cached(joint_values)

# 输出结果
for d in distances:
    print(f"关节 {d.link1} ↔ {d.link2} 距离: {d.distance:.6f} m")


# 记录结束时间
end_time = time.time()

# 计算总耗时
total_time = end_time - start_time
print(f"Total Computation Time1: {total_time:.6f} seconds\n")