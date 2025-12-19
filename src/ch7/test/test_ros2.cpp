#include <memory>
#include <string>

// ROS 2 核心头文件
#include "rclcpp/rclcpp.hpp"
// 点云消息头文件
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL 库及转换头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

class PcdSubscriber : public rclcpp::Node
{
public:
    PcdSubscriber() : Node("pcd_subscriber_node")
    {
        // 1. 设置 QoS (Quality of Service)
        // 这一点非常重要！
        // 激光雷达或深度相机通常使用 "Best Effort" (传感器数据模式) 发布数据。
        // 如果订阅者使用默认的 "Reliable"，由于不兼容，可能会收不到数据。
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        // 或者手动配置：qos_profile.keep_last(10).best_effort().durability_volatile();

        // 2. 创建订阅者
        // 这里的 topic 名字请改为你实际的话题，例如 "/camera/depth/points"
        std::string topic_name = "/camera/depth/points";
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name,
            qos_profile,
            std::bind(&PcdSubscriber::topic_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "已启动订阅节点，监听话题: %s", topic_name.c_str());
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        // 3. 将 ROS 消息转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 使用 pcl_conversions 进行转换
        // 如果你的点云包含颜色，可以使用 pcl::PointXYZRGB
        pcl::fromROSMsg(*msg, *cloud);

        // 4. 简单的处理/打印
        size_t point_count = cloud->points.size();
        
        if (point_count > 0) {
            // 获取第一个点的坐标用于验证
            float x = cloud->points[0].x;
            float y = cloud->points[0].y;
            float z = cloud->points[0].z;

            RCLCPP_INFO(this->get_logger(), 
                "收到一帧点云 | 宽度: %d, 高度: %d | 总点数: %lu | 第一个点: [%.2f, %.2f, %.2f]",
                msg->width, msg->height, point_count, x, y, z);
            
            // --- 在这里可以调用你的 ICP 算法 ---
            // process_icp(cloud);
        } else {
            RCLCPP_WARN(this->get_logger(), "收到空点云帧");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdSubscriber>());
    rclcpp::shutdown();
    return 0;
}