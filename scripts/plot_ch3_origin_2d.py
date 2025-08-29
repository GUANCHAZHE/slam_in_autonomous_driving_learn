import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':

    timestamps = []
    gyro_x, gyro_y, gyro_z = [], [], []
    acc_x, acc_y, acc_z = [], [], []

    # 读取文件
    with open('./data/ch3/10_small.txt', 'r') as f:
        for line in f:
            # 只处理IMU数据行
            if line.startswith('IMU'):
                # 分割数据行
                data = line.split()
                # 提取时间戳和数据
                t = float(data[1])
                gx, gy, gz = float(data[2]), float(data[3]), float(data[4])
                ax, ay, az = float(data[5]), float(data[6]), float(data[7])
                
                # 存储数据
                timestamps.append(t)
                gyro_x.append(gx)
                gyro_y.append(gy)
                gyro_z.append(gz)
                acc_x.append(ax)
                acc_y.append(ay)
                acc_z.append(az)

    # 将所有列表转换为numpy数组
    timestamps = np.array(timestamps)
    gyro_x = np.array(gyro_x)
    gyro_y = np.array(gyro_y)
    gyro_z = np.array(gyro_z)
    acc_x = np.array(acc_x)
    acc_y = np.array(acc_y)
    acc_z = np.array(acc_z)


    # 画图
    # plt.subplot(211)
    # plt.plot(timestamps, acc_x, 'r')
    # plt.plot(timestamps, acc_y, 'g')
    # plt.plot(timestamps, acc_z, 'b')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Acce')
    # plt.title('Acce')
    # plt.legend(['acc_x','acc_y','acc_z'])
    # plt.grid(True)


    # plt.subplot(212)
    # plt.plot(timestamps, gyro_x, 'r')
    # plt.plot(timestamps, gyro_y, 'g')
    # plt.plot(timestamps, gyro_z, 'b')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Acce')
    # plt.title('Gyro')
    # plt.legend(['gyro_x','gyro_y','gyro_z'])
    # plt.grid(True)
    # # 显示照片
    # plt.show()

    timestamps_s = timestamps[:7500]
    acc_x_s = acc_x[:7500].reshape(-1,1)
    acc_y_s = acc_y[:7500].reshape(-1,1)
    acc_z_s = acc_z[:7500].reshape(-1,1)
    gyro_x_s = gyro_x[:7500].reshape(-1,1)
    gyro_y_s = gyro_y[:7500].reshape(-1,1)
    gyro_z_s = gyro_z[:7500].reshape(-1,1)
    
    acc_s = np.hstack((acc_x_s, acc_y_s, acc_z_s))
    gyro_s = np.hstack((gyro_x_s, gyro_y_s, gyro_z_s))
    print("acc_s.shape", acc_s.shape)

    # # 画图
    # plt.subplot(211)
    # plt.plot(timestamps_s, acc_s[:,0], 'r')
    # plt.plot(timestamps_s, acc_s[:,1], 'g')
    # plt.plot(timestamps_s, acc_s[:,2], 'b')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Acce')
    # plt.title('Acce')
    # plt.legend(['acc_x','acc_y','acc_z'])
    # plt.grid(True)

    # plt.subplot(212)
    # plt.plot(timestamps_s, gyro_s[:,0], 'r')
    # plt.plot(timestamps_s, gyro_s[:,1], 'g')
    # plt.plot(timestamps_s, gyro_s[:,2], 'b')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Gyro')
    # plt.title('Gyro')
    # plt.legend(['gyro_x','gyro_y','gyro_z'])
    # plt.grid(True)
    # # 显示照片
    # plt.show()


    # 计算均值和协方差
    mean_x, var_x = np.mean(acc_x_s), np.var(acc_x_s)
    mean_y, var_y = np.mean(acc_y_s), np.var(acc_y_s)
    mean_z, var_z = np.mean(acc_z_s), np.var(acc_z_s)
    # 组合成向量
    mean_acce = np.array([mean_x, mean_y, mean_z])
    var_acce = np.array([var_x, var_y, var_z])
    print("抵消重力前的acce mean_acce, var_acce", mean_acce, var_acce)

    gravity = np.zeros(3)
    gravity = - mean_acce / np.linalg.norm(mean_acce,ord=1) *9.8
    print("np.linalg.norm(mean_acce,ord=1)", np.linalg.norm(mean_acce,ord=1))
    print("gravity", gravity)

    acc_s += gravity
    mean_acce = np.array([np.mean(acc_s[:,0]), np.mean(acc_s[:,1]), np.mean(acc_s[:,2])])
    var_acce = np.array([np.var(acc_s[:,0]), np.var(acc_s[:,1]), np.var(acc_s[:,2])])
    print("抵消重力后的acce mean_acce, var_acce", mean_acce, var_acce)

    mean_gyro = np.array([np.mean(gyro_s[:,0]), np.mean(gyro_s[:,1]), np.mean(gyro_s[:,2])])
    var_gyro = np.array([np.var(gyro_s[:,0]), np.var(gyro_s[:,1]), np.var(gyro_s[:,2])])
    print("gyro mean_gyro, var_gyro", mean_gyro, var_gyro)



    # # 画图
    # plt.subplot(211)
    # plt.plot(timestamps_s, acc_s[:,0], 'r')
    # plt.plot(timestamps_s, acc_s[:,1], 'g')
    # plt.plot(timestamps_s, acc_s[:,2], 'b')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Acce')
    # plt.title('Acce')
    # plt.legend(['acc_x','acc_y','acc_z'])
    # plt.grid(True)


    # plt.subplot(212)
    # plt.plot(timestamps_s, gyro_s[:,0], 'r')
    # plt.plot(timestamps_s, gyro_s[:,1], 'g')
    # plt.plot(timestamps_s, gyro_s[:,2], 'b')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Gyro')
    # plt.title('Gyro')
    # plt.legend(['gyro_x','gyro_y','gyro_z'])
    # plt.grid(True)
    # # 显示照片
    # plt.show()