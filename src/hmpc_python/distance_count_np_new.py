import distance_count_bind
import time

# 关节角度输入
joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0]

# 记录开始时间
start_time = time.time()

# **调用 C++ 计算**
link_pairs, gradient, joint_distances = distance_count_bind.compute_distance_gradient(joint_values)


# **确保先释放 C++ 资源，再 shutdown**
print("Calling distance_count_bind.release_resources()")

# 记录结束时间
end_time = time.time()
total_time = end_time - start_time
print(f"Total Computation Time: {total_time:.6f} seconds\n")

# **输出梯度矩阵形状**
print(f"Gradient Matrix Shape: {len(gradient)} x {len(gradient[0])}\n")

# **输出碰撞对和梯度**
for j, link_pair in enumerate(link_pairs):
    print(f"Collision Pair: {link_pair} | Distance: {joint_distances[j]:.6f}")
    for i, grad in enumerate(gradient):
        print(f"  ∂d({link_pair}) / ∂q{i+1} = {grad[j]:.6f}")
    print()

