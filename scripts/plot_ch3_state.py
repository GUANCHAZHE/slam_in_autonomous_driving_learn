# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker
import datetime
# 格式：t x y z qx qy qz qw vx vy vz
if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input valid file')
        exit(1)
    else:
        current_time = datetime.datetime.now()
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        plt.rcParams['figure.figsize'] = (16.0, 12.0)

        # 计算相对时间（从0开始）
        relative_time = path_data[:, 0] - path_data[0, 0]
        
        # 打印数据信息
        print(f"数据总条数: {len(path_data)}")
        print(f"时间范围: {relative_time[0]:.2f}s 到 {relative_time[-1]:.2f}s")
        print(f"采样间隔: {np.mean(np.diff(relative_time)):.4f}s")
        
        # 轨迹
        # 这个就是布局 一行两列
        #  |121|122|
        plt.subplot(121)   # 这个就是布局 一行两列
        plt.scatter(path_data[:, 1], path_data[:, 2], s=2)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid()
        plt.title('2D trajectory')

        # 姿态
        # 这个就是布局 两行两列
        #  |221|222|
        #  |223|224|
        plt.subplot(222)
        plt.plot(relative_time, path_data[:, 4], 'r')
        plt.plot(relative_time, path_data[:, 5], 'g')
        plt.plot(relative_time, path_data[:, 6], 'b')
        plt.plot(relative_time, path_data[:, 7], 'k')
        plt.xlabel('Time (s)')
        plt.ylabel('Quaternion Values')
        plt.title('Quaternion vs Time')
        plt.legend(['qw', 'qx', 'qy', 'qz'])
        plt.grid(True)
        # 设置x轴刻度间隔
        plt.gca().xaxis.set_major_locator(plt.MultipleLocator(500))  # 每500秒一个主刻度
        plt.gca().xaxis.set_minor_locator(plt.MultipleLocator(100))  # 每100秒一个次刻度

        # 速度
        # 这个就是布局 两行两列
        #  |221|222|
        #  |223|224|
        plt.subplot(224)
        plt.plot(relative_time, path_data[:, 8], 'r')
        plt.plot(relative_time, path_data[:, 9], 'g')
        plt.plot(relative_time, path_data[:, 10], 'b')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velocity vs Time')
        plt.legend(['vx', 'vy', 'vz'])
        plt.grid()

        # 调整子图之间的间距
        plt.tight_layout()
        
        # 保存图片，使用当前时间作为文件名的一部分
        timestamp = current_time.strftime('%Y%m%d_%H%M%S')  # 格式化时间字符串
        save_path = f'./scripts/trajectory_analysis_{timestamp}.png'  # 拼接文件名
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'图片已保存至: {save_path}')

        # 显示图片
        plt.show()
        exit(1)