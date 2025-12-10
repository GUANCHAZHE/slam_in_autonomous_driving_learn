//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ch6/icp_2d.h"
#include "ch6/lidar_2d_utils.h"
#include "common/io_utils.h"

DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor1.bag", "数据包路径");
DEFINE_string(method, "point2point", "2d icp方法：point2point/point2plane");

/// 测试从rosbag中读取2d scan并plot的结果
/// 通过选择method来确定使用点到点或点到面的ICP
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

    SE2 global_pose;   // 世界坐标系下的位姿
    std::vector<SE2> trajectory;  // 存储轨迹

    /// 我们将上一个scan与当前scan进行配准
    rosbag_io
        .AddScan2DHandle("/pavo_scan_bottom",
                         [&](Scan2d::Ptr scan) {
                             current_scan = scan;

                             if (last_scan == nullptr) {
                                 last_scan = current_scan;
                                 return true;
                             }

                             sad::Icp2d icp;
                             icp.SetTarget(last_scan);
                             icp.SetSource(current_scan);

                             SE2 pose;
                             if (fLS::FLAGS_method == "point2point") {
                                 icp.AlignGaussNewton(pose);
                             } else if (fLS::FLAGS_method == "point2plane") {
                                 icp.AlignGaussNewtonPoint2Plane(pose);
                             }

                            // 更新全局位姿：T_world_current = T_world_last * T_last_current
                            // 更新全局位姿
                            global_pose = global_pose * pose;
                            trajectory.push_back(global_pose);

                            // 显示 scan（原本）
                            cv::Mat image;
                            sad::Visualize2DScan(last_scan, SE2(), image, Vec3b(255, 0, 0));
                            sad::Visualize2DScan(current_scan, pose, image, Vec3b(0, 0, 255));
                            cv::imshow("scan", image);

                            // === 轨迹显示 ===
                            static cv::Mat traj_image = cv::Mat(900, 1600, CV_8UC3, cv::Scalar(255, 255, 255));

                            double x = global_pose.translation().x();
                            double y = global_pose.translation().y();

                            cv::Point2d pt(1500 - x * 5, 150 - y * 5);
                            cv::circle(traj_image, pt, 3, cv::Scalar(0, 0, 255), -1);

                            // if (trajectory.size() > 1) {
                            //     const SE2& last_pose = trajectory[trajectory.size() - 2];
                            //     cv::Point2d last_pt(800 - last_pose.translation().x() * 20,
                            //                         450 - last_pose.translation().y() * 20);
                            //     cv::line(traj_image, last_pt, pt, cv::Scalar(0, 0, 0), 1);
                            // }

                            cv::imshow("trajectory", traj_image);
                            cv::waitKey(20);

                             last_scan = current_scan;
                             return true;
                         })
        .Go();

    return 0;
}