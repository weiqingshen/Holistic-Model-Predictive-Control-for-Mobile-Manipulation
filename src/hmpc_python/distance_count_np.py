import distance_count_bind
import time
# 关节角度
joint_values = [0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0]


# 记录开始时间
start_time = time.time()

# 获取梯度和对应的碰撞对
link_pairs, gradient = distance_count_bind.compute_distance_gradient(joint_values)

# 记录结束时间
end_time = time.time()
# 计算总耗时
total_time = end_time - start_time
print(f"Total Computation Time: {total_time:.6f} seconds\n")

# 输出格式化结果
print(f"Gradient Matrix Shape: {len(gradient)} x {len(gradient[0])}\n")

for j, link_pair in enumerate(link_pairs):
    print(f"Collision Pair: {link_pair}")
    for i, grad in enumerate(gradient):
        print(f"  ∂d({link_pair}) / ∂q{i+1} = {grad[j]:.6f}")
    print()

