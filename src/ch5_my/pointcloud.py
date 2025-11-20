# -*- coding: utf-8 -*-
# 简化版本，只使用 Open3D
from ctypes import pointer
from math import inf
import open3d as o3d
import argparse
import logging
import os
import numpy as np
import cv2

import image_rotate

min_z = 0.2
max_z = 2.5
image_path = "./bev11111.png"

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


def brute_force(src, dst):
    """
    暴力匹配，计算src到目标点云的距离，找出其中最近的一个
    """

    distance = []

def load_and_vis_pcd(pcd_path):
   # 加载点云
    cloud = o3d.io.read_point_cloud(pcd_path)
    print(f"点云点数: {len(cloud.points)}")
    
    # 可视化
    o3d.visualization.draw_geometries([cloud])

    cloud_points_np = np.asarray(cloud.points)

    return cloud, cloud_points_np


def main():
    parser = argparse.ArgumentParser(description='点云可视化')
    parser.add_argument('--pcd_path', type=str, default='./data/ch5/map_example.pcd', 
                       help='点云文件路径')
    args = parser.parse_args()
    
    if not os.path.exists(args.pcd_path):
        logging.error(f"文件不存在: {args.pcd_path}")
        return -1
    
    cloud, cloud_points_np = load_and_vis_pcd(args.pcd_path)

    pcd_to_bird(cloud_points_np)

if __name__ == "__main__":
    main()
    print("程序全部结束")