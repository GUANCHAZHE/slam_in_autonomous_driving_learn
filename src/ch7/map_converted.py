import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt

def point_cloud_to_floor_plan(pcd_path, output_image_path, voxel_size=0.05):
    # 1. 读取点云
    print(f"正在加载点云: {pcd_path}...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    # 2. 预处理：降采样 (为了加快处理速度)
    pcd = pcd.voxel_down_sample(voxel_size=0.01)
    

    points = np.asarray(pcd.points)
    y_values = points[:, 1] # 取出高度

    print(f"=== 点云高度分布诊断 ===")
    print(f"总点数: {len(y_values)}")
    print(f"最低点: {y_values.min():.4f}, 最高点: {y_values.max():.4f}")

    # 将高度分为 10 层，看点都在哪里
    hist, bin_edges = np.histogram(y_values, bins=10)
    print("\n分层统计:")
    for i in range(len(hist)):
        print(f"层 {i+1}: {bin_edges[i]:.4f} 米 到 {bin_edges[i+1]:.4f} 米 --> 有 {hist[i]} 个点")
    print("=======================")
    # 假设 Z轴 是高度 (根据你的SLAM坐标系调整，有的可能是Y轴)
    # 建议截取墙壁最清晰的高度，例如 0.8m 到 1.8m 之间
    min_height = -2.0
    max_height = -0.08
    
    # 过滤掉不在范围内点
    mask = (points[:, 1] > min_height) & (points[:, 1] < max_height)
    slice_points = points[mask]
    
    if len(slice_points) == 0:
        print("错误：截取高度后没有剩余点，请检查点云坐标系（是Y轴向上还是Z轴向上？）")
        return

    # 4. 投影到 2D 平面 (XY平面)
    # 提取 X 和 Y 坐标
    x_points = slice_points[:, 0]
    y_points = slice_points[:, 2]
    
    # 计算地图边界
    min_x, max_x = np.min(x_points), np.max(x_points)
    min_y, max_y = np.min(y_points), np.max(y_points)
    
    # 设定像素分辨率 (例如 5cm 一个像素)
    resolution = 0.01 
    width = int((max_x - min_x) / resolution) + 10
    height = int((max_y - min_y) / resolution) + 10
    
    # 创建空白图像 (黑色背景)
    map_img = np.zeros((height, width), dtype=np.uint8)
    
    # 将点映射到像素坐标
    pixel_x = ((x_points - min_x) / resolution).astype(int)
    pixel_y = ((y_points - min_y) / resolution).astype(int)
    
    # 将有墙的地方设为白色 (255)
    # 这里加一点 margin 防止边缘溢出
    map_img[pixel_y, pixel_x] = 255
    
    # 图片通常是上下颠倒的，翻转一下方便看
    map_img = cv2.flip(map_img, 0)
    
    print("原始投影完成，开始图像后处理...")

    # 5. 图像后处理 (OpenCV Magic)
    # 这一步是将"粗糙地图"变成"用户友好地图"的关键
    
    # A. 闭运算 (Closing): 先膨胀后腐蚀，用于连接断裂的墙壁，填补玻璃造成的空洞
    kernel_close = np.ones((5, 5), np.uint8) 
    processed_img = cv2.morphologyEx(map_img, cv2.MORPH_CLOSE, kernel_close)
    
    # B. 开运算 (Opening): 先腐蚀后膨胀，用于去除独立的噪点（比如残留的杂物）
    kernel_open = np.ones((3, 3), np.uint8)
    processed_img = cv2.morphologyEx(processed_img, cv2.MORPH_OPEN, kernel_open)
    
    # C. (可选) 高斯模糊 + 二值化，让边缘更平滑
    processed_img = cv2.GaussianBlur(processed_img, (5, 5), 0)
    _, processed_img = cv2.threshold(processed_img, 127, 255, cv2.THRESH_BINARY)
    
    # D. 轮廓提取 (只保留大的轮廓，去除小的碎片)
    contours, _ = cv2.findContours(processed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    final_map = np.zeros_like(processed_img)
    
    # 比如只保留面积大于一定阈值的轮廓（过滤掉小椅子腿留下的点）
    for cnt in contours:
        if cv2.contourArea(cnt) > 50: # 阈值根据分辨率调整
             # approxPolyDP 可以把弯弯曲曲的线拉直成直线
            epsilon = 0.01 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            cv2.drawContours(final_map, [approx], -1, 255, -1) # -1 填充实心，2 表示画线宽

    # 6. 显示与保存
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.title("Raw Projection")
    plt.imshow(map_img, cmap='gray')
    plt.axis('off')
    
    plt.subplot(1, 2, 2)
    plt.title("Processed (Cleaned)")
    plt.imshow(final_map, cmap='gray')
    plt.axis('off')
    plt.show()
    
    cv2.imwrite(output_image_path, final_map)
    print(f"处理完成，图片已保存至 {output_image_path}")

# 使用示例
# 请将 'your_point_cloud.pcd' 替换为你的实际文件路径
# point_cloud_to_floor_plan('your_point_cloud.pcd', 'clean_map.png')


def main():
    point_cloud_to_floor_plan(
        pcd_path="/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_output_cloud.pcd",
        output_image_path="/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_output_cloud.png"
    )

if __name__ == "__main__":
    main()
_