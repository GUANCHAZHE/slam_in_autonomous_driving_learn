# -*- coding: utf-8 -*-
# 简化版本，只使用 Open3D，已重构为类
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
from concurrent.futures import ThreadPoolExecutor


# default_pcd_path = './data/ch5/map_example.pcd'
default_pcd_path = './data/ch5/first.pcd'

class PointCloudProcessor:
    """点云处理器，封装常用的点云 -> 图像 / 最近邻等方法，便于测试与复用。

    主要方法：
    - pcd_to_bird: 生成鸟瞰图并保存
    - scan_to_range_image: 生成激光的 range image 并保存
    - bfnn / bfnn_cloud: 暴力最近邻匹配（用于测试/基线）
    - load_and_vis_pcd: 读取并可视化点云
    """

    def __init__(self,
                 min_z=0.2,
                 max_z=2.5,
                 image_path="./bev11111.png",
                 range_image_path="./my_range_image.png",
                 azimuth_resolution_deg=0.3,
                 elevation_range=15,
                 elevation_rows=16,
                 lidar_height=1.128):
        self.min_z = min_z
        self.max_z = max_z
        self.image_path = image_path
        self.range_image_path = range_image_path
        self.azimuth_resolution_deg = azimuth_resolution_deg
        self.elevation_range = elevation_range
        self.elevation_rows = elevation_rows
        self.lidar_height = lidar_height

    def pcd_to_bird(self, points_np, resolution=0.1):
        """把点云投影成鸟瞰图并保存为图片。

        返回生成的 BGR 图像 numpy 数组。
        """
        min_x, min_y, _ = np.min(points_np, axis=0)
        max_x, max_y, _ = np.max(points_np, axis=0)

        inv_r = 1.0 / resolution

        image_rows = max(1, int(((max_y - min_y) * inv_r)))
        image_cols = max(1, int(((max_x - min_x) * inv_r)))

        x_center_point = 0.5 * (max_x + min_x)
        y_center_point = 0.5 * (max_y + min_y)
        x_center_image = 0.5 * image_cols
        y_center_image = 0.5 * image_rows

        image = np.full((image_rows, image_cols, 3), (255, 255, 255), dtype=np.uint8)

        x_coords = ((points_np[:, 0] - x_center_point) * inv_r + x_center_image).astype(int)
        y_coords = ((points_np[:, 1] - y_center_point) * inv_r + y_center_image).astype(int)

        valid_mask = (
            (x_coords >= 0) & (x_coords < image_cols) &
            (y_coords >= 0) & (y_coords < image_rows) &
            (points_np[:, 2] >= self.min_z) & (points_np[:, 2] <= self.max_z)
        )

        image[y_coords[valid_mask], x_coords[valid_mask]] = [227, 143, 79]

        cv2.imwrite(self.image_path, image)
        print(f"完成图像存储位于{self.image_path}")
        return image

    def scan_to_range_image(self, point_np):
        """将扫描到的点云转换为距离图（range image）。

        返回 BGR 图像。
        """
        image_cols = int(360 / self.azimuth_resolution_deg)
        image_rows = int(self.elevation_rows)
        print(f"range iamge : {image_rows} * {image_cols}")

        image = np.zeros((image_rows, image_cols, 3), dtype=np.uint8)

        ele_resolution = (self.elevation_range * 2) / image_rows

        for i in range(len(point_np)):
            pt = point_np[i]
            px, py, pz = pt[0], pt[1], pt[2]
            range_aval = np.sqrt(px * px + py * py)
            if range_aval < 1e-6:
                continue
            azimuth = np.arctan2(py, px) * 180 / np.pi

            ratio = (pz - self.lidar_height) / range_aval
            ratio = np.clip(ratio, -1.0, 1.0)
            elevation = np.arcsin(ratio) * 180 / np.pi

            if azimuth < 0:
                azimuth += 360

            x = int(azimuth / self.azimuth_resolution_deg)
            y = int((elevation + self.elevation_range) / ele_resolution + 0.5)

            if 0 <= x < image_cols and 0 <= y < image_rows:
                image[y, x] = [int(range_aval / 100 * 255.0), 255, 127]

        image_flipped = np.flip(image, axis=0)
        image_bgr = cv2.cvtColor(image_flipped, cv2.COLOR_HSV2BGR)
        cv2.imwrite(self.range_image_path, image_bgr)
        print(f"图像存储于 {self.range_image_path}")
        return image_bgr

    def bfnn(self, target_point, point_np):
        """暴力最近邻：返回每个点到 target_point 的距离列表（未排序的返回值原样）。"""
        target = np.asarray(target_point)
        distances = np.linalg.norm(point_np - target, axis=1)
        # 返回 numpy 数组，调用方可自行排序或取最小值
        return distances

    def bfnn_cloud(self, point_np_1, point_np_2):
        """对两个点云做暴力匹配，返回一个列表，其中每个元素是 distances（numpy array）。"""
        matches = []
        for i in range(len(point_np_1)):
            dis = self.bfnn(point_np_1[i], point_np_2)
            matches.append(dis)
        return matches

    def bfnn_cloud_mt(self, point_np_1=None, point_np_2=None):
        """占位：多线程最近邻匹配（当前未实现），返回与 cloud2 相同长度的 matches 列表。"""
        if point_np_2 is None:
            return []
        if point_np_1 is None:
            return []

        point_np_1 = np.asarray(point_np_1)
        point_np_2 = np.asarray(point_np_2)

        def _nn(idx_point):
            idx, pt = idx_point
            # 计算到所有点的欧氏距离并取最小
            dists = np.linalg.norm(point_np_2 - pt, axis=1)
            min_idx = int(np.argmin(dists))
            min_dist = float(dists[min_idx])
            return (int(idx), min_idx, min_dist)

        matches = []
        # 使用线程池并行计算每个点的最近邻
        with ThreadPoolExecutor() as exe:
            # map 保持输入顺序
            for res in exe.map(_nn, enumerate(point_np_1)):
                matches.append(res)

        return matches

    def load_and_vis_pcd(self, pcd_path, is_vis=True):
        """加载点云并可选可视化，返回 (cloud, points_np)。"""
        cloud = o3d.io.read_point_cloud(pcd_path)
        print(f"点云点数: {len(cloud.points)}")
        if is_vis:
            o3d.visualization.draw_geometries([cloud])
        cloud_points_np = np.asarray(cloud.points)
        return cloud, cloud_points_np

    def voxelize_pcd(self, pcd, voxel_size=0.1):
        """使用 Open3D 体素化点云"""
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
        
        # 获取体素中心点（近似代表点）
        voxels = voxel_grid.get_voxels()
        points = []
        for voxel in voxels:
            pt = voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
            points.append(pt)
        voxel_pcd = o3d.geometry.PointCloud()
        voxel_pcd.points = o3d.utility.Vector3dVector(np.asarray(points))

        return voxel_pcd, voxel_grid


    def visualize_voxel_and_original(self, original_pcd, voxel_size=0.1):
        voxel_pcd, voxel_grid = self.voxelize_pcd(original_pcd, voxel_size)

        # 可视化：原始点云（灰色）+ 体素（彩色）
        original_pcd.paint_uniform_color([0.8, 0.8, 0.8])  # 灰色
        voxel_pcd.paint_uniform_color([1, 0, 0])           # 红色

        # o3d.visualization.draw_geometries([original_pcd, voxel_pcd], window_name="Original vs Voxelized")

        # 或直接显示体素网格（带立方体）
        # o3d.visualization.draw_geometries([voxel_grid], window_name="Voxel Grid (with cubes)")

    def test_voxel_vs_original(self, pcd_path=default_pcd_path):
        cloud, points_np = self.load_and_vis_pcd(pcd_path, is_vis=False)
        
        # 体素化
        voxel_pcd, voxel_grid = self.voxelize_pcd(cloud, voxel_size=0.1)
        
        # 可视化网格（带立方体）
        print("显示体素网格（带立方体）...")
        # o3d.visualization.draw_geometries([voxel_grid])
        
        # 可视化点对比
        cloud.paint_uniform_color([0.7, 0.7, 0.7])
        voxel_pcd.paint_uniform_color([1, 0, 0])
        print("显示原始点云（灰） vs 体素中心点（红）...")
        o3d.visualization.draw_geometries([cloud, voxel_pcd])

    # 将原有测试函数改为方法，便于在单元测试中调用
    def test_bfnn_cloud(self):
        point_np_1 = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3]])
        point_np_2 = np.array([[1.2, 1.2, 1.2], [2.2, 2.2, 2.2], [3.2, 3.2, 3.2]])
        t1 = time.time()
        matches = self.bfnn_cloud(point_np_1, point_np_2)
        t2 = time.time()
        t = t2 - t1
        for i in range(len(matches)):
            print(matches[i])
        print(f"测试完成时间 {t}, 测试成功")

    def test_scan_to_range_image(self, pcd_path=default_pcd_path):
        _, point_np = self.load_and_vis_pcd(pcd_path, False)
        t1 = time.time()
        self.scan_to_range_image(point_np)
        t2 = time.time()
        print(f"完成scan_to_range_image 测试 消耗时间 {t2 - t1}")

    def test_brute_force(self):
        target = [0, 0, 0]
        point_np = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3]])
        dis = self.bfnn(target, point_np)
        print(dis.min())
        print("brute force 测试通过")

    def test_brute_force_default_pcd(self, pcd_path=default_pcd_path):
        _, point_np = self.load_and_vis_pcd(pcd_path, False)
        t1 = time.time()
        dis = self.bfnn([0, 0, 0], point_np)
        t2 = time.time()
        print(f"距离最近的点是 dis.min() {dis.min()}")
        print(f"测试完成时间 {t2 - t1}, 测试成功")


def main():
    parser = argparse.ArgumentParser(description='点云可视化')
    parser.add_argument('--pcd_path', type=str, default=default_pcd_path,
                        help='点云文件路径')
    parser.add_argument('--vis', action='store_true', help='是否可视化点云')
    args = parser.parse_args()

    if not os.path.exists(args.pcd_path):
        logging.error(f"文件不存在: {args.pcd_path}")
        return -1

    processor = PointCloudProcessor()
    cloud, cloud_points_np = processor.load_and_vis_pcd(args.pcd_path, is_vis=args.vis)

    # 示例：若需要生成鸟瞰图或 range image，可取消下面注释
    # processor.pcd_to_bird(cloud_points_np)
    # processor.scan_to_range_image(cloud_points_np)
    processor.test_voxel_vs_original()
    # 测试代码


if __name__ == "__main__":
    main()
    print("程序全部结束")