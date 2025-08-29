#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# 读取数据文件
data = np.loadtxt('../../data/ch3/gins.txt')

# 提取位置信息
# 文件格式：timestamp px py pz qw qx qy qz vx vy vz bgx bgy bgz bax bay baz
timestamps = data[:, 0]
px = data[:, 1]  # X坐标
py = data[:, 2]  # Y坐标

# 创建图形
plt.figure(figsize=(10, 8))

# 绘制轨迹
plt.plot(px, py, 'b-', label='Trajectory')
plt.plot(px[0], py[0], 'go', label='Start')  # 起点
plt.plot(px[-1], py[-1], 'ro', label='End')  # 终点

# 设置图形属性
plt.title('Vehicle Trajectory (2D)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid(True)
plt.axis('equal')  # 保持纵横比相等
plt.legend()

# 保存图片
plt.savefig('trajectory_2d.png')
plt.show()
