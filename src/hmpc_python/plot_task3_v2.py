import os
import pickle
import matplotlib.pyplot as plt
import numpy as np

# 文件路径
output_dir = 'output'
file_prmstar = os.path.join(output_dir, 'trajectory_data3_v2.pkl')
file_hmpc = os.path.join(output_dir, 'trajectory_data1_v2.pkl')
file_hmpc_fixed = os.path.join(output_dir, 'trajectory_data2_v2.pkl')

# 加载数据函数（读取时间步和目标偏差）
def load_error_data(file_path):
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    return data['time_steps'][:2000], data['target_differences'][:2000]

# 加载三种算法的数据
ts_prmstar, err_prmstar = load_error_data(file_prmstar)
ts_hmpc, err_hmpc = load_error_data(file_hmpc)
ts_hmpc_fixed, err_hmpc_fixed = load_error_data(file_hmpc_fixed)

# 添加起始点 (0, 0.176)
ts_prmstar = [-1] + ts_prmstar
err_prmstar = [0.176] + err_prmstar

ts_hmpc = [-1] + ts_hmpc
err_hmpc = [0.176] + err_hmpc

ts_hmpc_fixed = [-1] + ts_hmpc_fixed
err_hmpc_fixed = [0.176] + err_hmpc_fixed

# 转换为秒（1步 = 0.01秒）
ts_prmstar = [t * 0.01 for t in ts_prmstar]
ts_hmpc = [t * 0.01 for t in ts_hmpc]
ts_hmpc_fixed = [t * 0.01 for t in ts_hmpc_fixed]

# 开始绘图
width = 2.5
plt.figure(figsize=(10, 5))
plt.plot(ts_prmstar, err_prmstar, label='PRMstar', linewidth=width, color='#1f77b4')     # 柔和蓝
plt.plot(ts_hmpc, err_hmpc, label='HMPC', linewidth=width, color='#ff7f0e')              # 亮橙
plt.plot(ts_hmpc_fixed, err_hmpc_fixed, label='HMPC-Fixed', linewidth=width, color='#2ca02c')  # 柔绿

plt.xlabel('Time (s)', fontsize=12)

plt.ylabel('Position Error (m)', fontsize=12)
plt.title('Target Deviation Over Time (First 2000 Steps)', fontsize=13)
plt.grid(True)
plt.legend(fontsize=12)
plt.tight_layout()

# 保存图像
output_path = os.path.join(output_dir, 'error_comparison.png')
plt.savefig(output_path, dpi=300)
plt.show()
