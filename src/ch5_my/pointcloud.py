# -*- coding: utf-8 -*-
# 简化版本，只使用 Open3D
import open3d as o3d
import argparse
import logging
import os

def main():
    parser = argparse.ArgumentParser(description='点云可视化')
    parser.add_argument('--pcd_path', type=str, default='./data/ch5/map_example.pcd', 
                       help='点云文件路径')
    args = parser.parse_args()
    
    if not os.path.exists(args.pcd_path):
        logging.error(f"文件不存在: {args.pcd_path}")
        return -1
    
    # 加载点云
    cloud = o3d.io.read_point_cloud(args.pcd_path)
    print(f"点云点数: {len(cloud.points)}")
    
    # 可视化
    o3d.visualization.draw_geometries([cloud])

if __name__ == "__main__":
    main()
    print("test")