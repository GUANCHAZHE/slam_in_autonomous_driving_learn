# -*- coding: utf-8 -*-
# 简化版本，只使用 Open3D
from ctypes import pointer
from math import inf
from traceback import print_tb
import open3d as o3d
import argparse
import logging
import os
import numpy as np
import cv2
import time


min_z = 0.2
max_z = 2.5
image_path = "./bev11111.png"
range_image_path = "./my_range_image.png"
default_pcd_path='./data/ch5/map_example.pcd'

azimuth_resolution_deg = 0.3      # 方位角的分辨率，水平方向分辨率
elevation_range = 15              # 俯仰角的上下范围度数
elevation_rows = 16               # 俯仰角对应的行数
lidar_height = 1.128              # 雷达的高度 单位米

def pcd_to_bird(points_np, resolution = 0.1):
    min_x, min_y, _= np.min(points_np, axis=0)
    max_x, max_y, _ = np.max(points_np, axis=0)
  
    # 能使用乘法就是用乘法 效率高
    inv_r = 1.0 / resolution

    # 计算图像的尺寸大小
    image_rows = ((max_y - min_y) * inv_r).astype(int) 
    image_cols = ((max_x - min_x) * inv_r).astype(int)

    # 计算图像和点云的中心
    x_center_point = 0.5 * (max_x + min_x)
    y_center_point = 0.5 * (max_y + min_y) 
    x_center_image = 0.5 * image_cols 
    y_center_image = 0.5 * image_rows

    image = np.full((image_rows, image_cols, 3), (255,255,255), dtype=np.uint8)

    x_coords = ((points_np[:, 0] - x_center_point) * inv_r + x_center_image).astype(int)
    y_coords = ((points_np[:, 1] - y_center_point) * inv_r + y_center_image).astype(int)

    # 设置有效点位
    valid_mask = (
        (x_coords >= 0) & (x_coords < image_cols) &
        (y_coords >= 0) & (y_coords < image_rows) &
        (points_np[:, 2] >= min_z) & (points_np[:,2] <= max_z)
    )

    # 修正像素值
    image[y_coords[valid_mask], x_coords[valid_mask]] = [227, 143, 79]

    cv2.imwrite(image_path, image)
    print(f"完成图像存储位于{image_path}")
    return image

def scan_to_range_image(point_np):
    """
    将扫描到的点云转换为距离图，可以检测地形，比如台阶之类的
    """
    image_cols = int( 360 / azimuth_resolution_deg)   # 1200
    image_rows = int( elevation_rows)                 # 16
    print(f"range iamge : {image_rows} * {image_cols}")

    # 生成hsv图像更好的显示图像
    image = np.zeros((image_rows, image_cols, 3), dtype=np.uint8)

    # elevation 分辨率
    ele_resolution = (elevation_range * 2) / image_rows

    for i in range(len(point_np)):
        pt= point_np[i]
        px = pt[0]
        py = pt[1]
        pz = pt[2]
        # 这里的range 并非 三维视图的 x^2 + y^2 + z^2 ，而是地上的线
        range_aval = np.sqrt(px * px + py * py)
        if range_aval < 1e-6:
            continue;     # 过滤数值较小的点
        azimuth = np.arctan2(py, px) * 180 / np.pi  # 度

        ratio = (pz - lidar_height) /range_aval
        ratio = np.clip(ratio, -1.0, 1.0)           # 考虑范围 ，将它传递到 (-1, 1)
        # elevation = np.arctan2(pz - lidar_height, range_aval) * 180 / np.pi  # 这个就是标准的空间几何定义
        elevation = np.arcsin(ratio) * 180 / np.pi  # 这个貌似是转换视图的定义 是 

        # print(f"elevation_tan {elevation_tan} elevation_sin {elevation_sin}")

        if azimuth < 0:
            azimuth += 360

        x = int(azimuth / azimuth_resolution_deg)                       # 行
        y = int((elevation + elevation_range) / ele_resolution + 0.5)   # 列

        if 0 <= x  < image_cols and 0 <= y < image_rows:
            image[y, x] = [int(range_aval / 100 * 255.0), 255, 127]
    
    # 将y向上翻转
    image_flipped = np.flip(image, axis=0)

    # hsv 转 bgr
    image_rbg = cv2.cvtColor(image_flipped, cv2.COLOR_HSV2BGR)
    cv2.imwrite(range_image_path, image_rbg)
    print(f"图像存储于 {range_image_path}")
    return image_rbg

        


def bfnn(target_point, point_np):
    """
    暴力匹配，计算src到目标点云的距离三维距离，找出其中最近的一个
    """
    distance = []
    for i in range(len(point_np)):
        dis = np.linalg.norm( target_point - point_np[i] )
        distance.append(dis)
    np.sort(distance)

    return distance

def bfnn_cloud(point_np_1, point_np_2):
    """
    两个点云之间的匹配，计算他们之间的相对误差
    返回结果为matches
    """
    matches = []

    for i in range(len(point_np_2)):
        dis = bfnn(point_np_1[i], point_np_2)
        matches.append(dis)
    
    return matches

def bfnn_cloud_mt(point_np_1 = None, point_np_2 = None):
    """
    多线程最近邻匹配
    返回与 cloud2 相同长度的 matches 数组
    每个元素为 (idx1, idx2)
    """

    matches = [None] * len(point_np_2)

    # with THrea

def load_and_vis_pcd(pcd_path, is_vis):
   # 加载点云
    cloud = o3d.io.read_point_cloud(pcd_path)
    print(f"点云点数: {len(cloud.points)}")
    
    # 可视化
    if is_vis == True:
        o3d.visualization.draw_geometries([cloud])

    cloud_points_np = np.asarray(cloud.points)

    return cloud, cloud_points_np












#####################  TEST CODE  ##################
def test_bfnn_cloud():
    point_np_1 = np.array([[1,1,1], [2,2,2], [3,3,3]])
    point_np_2 = np.array([[1.2,1.2,1.2], [2.2,2.2,2.2], [3.2,3.2,3.2]])
    t1 = time.time()
    matcehs = bfnn_cloud(point_np_1, point_np_2)
    t2 = time.time()
    t = t2 - t1

    for i in range(len(matcehs)):
        print(matcehs[i])
    print(f"测试完成时间 {t}, 测试成功")


def test_scan_to_range_image(point_np=None):
    _, point_np =load_and_vis_pcd(default_pcd_path, False)
    t1 = time.time()
    scan_to_range_image(point_np)
    t2 = time.time()
    t = t2 - t1
    print(f"完成scan_to_range_image 测试 消耗时间 t{t}")







def test_brute_force():
    target = [0, 0, 0]
    point_np = np.array([[1,1,1], [2,2,2], [3,3,3]])
    dis = bfnn(target, point_np)
    print(dis[0])
    print("brute force 测试通过")

def test_brute_force_defalut_pcd():

    _, point_np =load_and_vis_pcd(default_pcd_path, False)
    target = [0, 0, 0]
    t1 = time.time()
    dis = bfnn(target, point_np)
    t2 = time.time()

    print(f"距离最近的点是 dis[0] {dis[0]}")

    t = t2 - t1

    print(f"测试完成时间 {t}, 测试成功")

    print("brute force 测试通过")



def main():
    parser = argparse.ArgumentParser(description='点云可视化')
    parser.add_argument('--pcd_path', type=str, default='./data/ch5/map_example.pcd', 
                       help='点云文件路径')
    args = parser.parse_args()
    
    if not os.path.exists(args.pcd_path):
        logging.error(f"文件不存在: {args.pcd_path}")
        return -1
    
    # cloud, cloud_points_np = load_and_vis_pcd(args.pcd_path, False)

    # pcd_to_bird(cloud_points_np)



    #   TEST 
    # test_brute_force()
    # test_bfnn_cloud()
    # test_brute_force_defalut_pcd()
    test_scan_to_range_image()

if __name__ == "__main__":
    main()
    print("程序全部结束")