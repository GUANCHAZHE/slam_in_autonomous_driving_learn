// #include <memory>
// #include <string>
// #include <iostream>

// // ROS 2 核心
// #include "rclcpp/rclcpp.hpp"
// // 消息类型
// #include "sensor_msgs/msg/point_cloud2.hpp"

// // PCL 库 (ROS 2 转换库)
// #include "pcl_conversions/pcl_conversions.h"
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>

// class BagFrameSaver : public rclcpp::Node
// {
// public:
//     BagFrameSaver() : Node("bag_frame_saver_node")
//     {
//         // 目标话题名称
//         std::string topic_name = "/camera1_SD0140820L0057/points2";

//         // 设置 QoS: SensorData (Best Effort)
//         // 这对于读取 rosbag 或 传感器数据至关重要，否则可能连不上
//         rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

//         // 创建订阅
//         sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             topic_name,
//             qos_profile,
//             std::bind(&BagFrameSaver::topic_callback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "等待接收话题: %s ...", topic_name.c_str());
//         RCLCPP_INFO(this->get_logger(), "请在另一个终端播放 rosbag (ros2 bag play ...)");
//     }

// private:
//     void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         if (has_saved_) return; // 防止重复保存

//         RCLCPP_INFO(this->get_logger(), "收到数据帧！正在转换...");

//         // 1. 将 ROS 消息转换为 PCL 点云
//         pcl::PointCloud<pcl::PointXYZ> cloud;
//         pcl::fromROSMsg(*msg, cloud);

//         // 2. 构造文件名 (保存到当前运行目录下)
//         std::string filename = "frame_first.pcd";

//         // 3. 保存文件 (使用二进制模式，速度快体积小)
//         try {
//             if (cloud.empty()) {
//                 RCLCPP_WARN(this->get_logger(), "收到的点云为空，跳过保存。");
//                 return;
//             }
//             pcl::io::savePCDFile(filename, cloud);
            
//             RCLCPP_INFO(this->get_logger(), "成功保存第一帧到: %s (点数: %lu)", filename.c_str(), cloud.size());
            
//             // 4. 任务完成，关闭节点
//             has_saved_ = true;
//             rclcpp::shutdown();

//         } catch (const std::exception &e) {
//             RCLCPP_ERROR(this->get_logger(), "保存失败: %s", e.what());
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
//     bool has_saved_ = false;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<BagFrameSaver>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }