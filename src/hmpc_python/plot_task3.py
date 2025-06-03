import os
import pickle
import matplotlib.pyplot as plt
import numpy as np

# 文件路径
output_dir = 'output'
file_prmstar = os.path.join(output_dir, 'trajectory_data3.pkl')
file_hmpc = os.path.join(output_dir, 'trajectory_data1.pkl')
file_hmpc_fixed = os.path.join(output_dir, 'trajectory_data2.pkl')

# 加载数据函数
def load_trajectory(file_path):
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    return data['ee_positions'], data['target_positions']

# 读取三段末端执行器轨迹和目标轨迹（任意一个文件中的目标轨迹）
ee_prmstar, target_positions = load_trajectory(file_prmstar)
ee_hmpc, _ = load_trajectory(file_hmpc)
ee_hmpc_fixed, _ = load_trajectory(file_hmpc_fixed)

# 提取目标轨迹坐标
target_x = [pos[0] for pos in target_positions]
target_y = [pos[1] for pos in target_positions]

# 提取各算法末端执行器轨迹坐标
prmstar_x = [pos[0] for pos in ee_prmstar]
prmstar_y = [pos[1] for pos in ee_prmstar]

hmpc_x = [pos[0] for pos in ee_hmpc]
hmpc_y = [pos[1] for pos in ee_hmpc]

hmpc_fixed_x = [pos[0] for pos in ee_hmpc_fixed]
hmpc_fixed_y = [pos[1] for pos in ee_hmpc_fixed]

# 开始绘图
width=3
plt.figure()
plt.plot(target_x, target_y, label='Target Path', color='black', linewidth=width)
plt.plot(prmstar_x, prmstar_y, label='PRMstar', color='#1f77b4', linewidth=width)  # 柔和蓝
plt.plot(hmpc_x, hmpc_y, label='HMPC', color='#ff7f0e', linewidth=width)           # 亮橙
plt.plot(hmpc_fixed_x, hmpc_fixed_y, label='HMPC-Fixed', color='#2ca02c', linewidth=width)  # 柔绿


plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
# plt.title('Comparison of End-Effector Trajectories')
plt.grid(True)
# plt.axis('equal')
plt.gca().set_aspect('equal', adjustable='box')  # 单位长度相同，但图像不强制正方形
# plt.xlim(-0.11, 0.25)

# 设置稀疏刻度（你可以根据实际需要调整步长）
plt.xticks(np.arange(-0.11, 0.28, 0.06))  # 每 0.1 米一个刻度
plt.yticks(np.arange(0.16, 0.36, 0.06))  # 你可以换成适合你的 y 范围

plt.legend(loc='upper right', fontsize=14)


# 保存图像
output_path = os.path.join(output_dir, 'trajectory_comparison.png')
plt.savefig(output_path, dpi=300)
plt.show()
