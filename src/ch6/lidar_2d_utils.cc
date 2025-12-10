//
// Created by xiang on 2022/3/15.
//

#include "ch6/lidar_2d_utils.h"
#include <opencv2/imgproc.hpp> // 包含 OpenCV 图像处理函数，如 cv::circle

namespace sad {

/**
 * @brief 将 2D 激光扫描数据可视化到图像上
 * @param scan        2D 激光扫描数据指针 (包含角度、距离等信息)
 * @param pose        该扫描数据在世界坐标系中的位姿 (SE2: x, y, theta)
 * @param image       输出图像 (CV_8UC3, BGR 格式)
 * @param color       绘制颜色 (Vec3b: B, G, R)
 * @param image_size  图像的边长 (假设为正方形)
 * @param resolution  图像的分辨率 (像素/米)
 * @param pose_submap 子地图坐标系相对于世界坐标系的位姿 (默认为单位变换)
 */
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap) {
    
    // 1. 初始化图像
    // 如果传入的 image 是空的 (data == nullptr)，则创建一个指定大小的白色图像。
    // CV_8UC3 表示 8 位无符号整数，3 通道 (BGR)。
    // cv::Vec3b(255, 255, 255) 表示白色 (B=255, G=255, R=255)。
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    // 2. 遍历扫描数据中的每个点
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        
        // 3. 距离过滤
        // 检查当前射线的距离值是否在有效范围内 (range_min, range_max)。
        // 如果距离无效 (如小于最小值或大于最大值)，则跳过该点。
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }

        // 4. 计算真实角度
        // 每个距离值 ranges[i] 对应一个角度。
        // 真实角度 = 起始角度 + 索引 * 角度增量
        double real_angle = scan->angle_min + i * scan->angle_increment;
        
        // 5. 计算激光点在雷达坐标系下的坐标
        // 使用极坐标到直角坐标的转换公式：
        // x = 距离 * cos(角度)
        // y = 距离 * sin(角度)
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        // 6. 角度范围过滤 (可选)
        // 这个 if 语句过滤掉扫描数据两端各 30 度范围内的点。
        // scan->angle_min + 30 * M_PI / 180.0 是起始角度后 30 度
        // scan->angle_max - 30 * M_PI / 180.0 是结束角度前 30 度
        // 这通常是为了避免雷达自身或车辆前方/后方的遮挡物。
        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        // 7. 坐标变换：从雷达坐标系 -> 世界坐标系 -> 子地图坐标系
        // pose * Vec2d(x, y): 将雷达坐标系下的点 (x, y) 变换到世界坐标系下。
        // pose_submap.inverse() * (...): 将世界坐标系下的点变换到子地图坐标系下。
        // psubmap 是点在子地图坐标系下的坐标。
        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(x, y));

        // 8. 坐标变换：从子地图坐标系 -> 图像像素坐标系
        // psubmap[0] * resolution: 将米单位的 x 坐标转换为像素单位。
        // + image_size / 2: 将子地图坐标系的原点 (通常在中心) 映射到图像的中心像素。
        int image_x = int(psubmap[0] * resolution + image_size / 2);
        int image_y = int(psubmap[1] * resolution + image_size / 2);

        // 9. 图像边界检查
        // 检查计算出的像素坐标是否在图像的有效范围内。
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            // 10. 绘制点
            // 如果在范围内，则在图像的对应像素位置设置颜色。
            image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
            // 注意：这里再次用 cv::Vec3b 构造 color，可能多余，直接用 color 即可。
            // image.at<cv::Vec3b>(image_y, image_x) = color; 是等价的。
        }
    }

    // 11. 绘制位姿点
    // 计算雷达位姿 (pose.translation() 是位姿的平移部分，即 x, y 坐标) 在子地图坐标系下的位置。
    // 然后将其转换为图像像素坐标。
    Vec2d pose_in_image =
        pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    
    // 在图像上画一个圆圈，表示雷达的位置。
    // cv::Point2f(pose_in_image[0], pose_in_image[1]): 圆心坐标
    // 5: 圆的半径 (像素)
    // cv::Scalar(color[0], color[1], color[2]): 圆的颜色
    // 2: 圆的线宽 (像素)
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);
}

}  // namespace sad