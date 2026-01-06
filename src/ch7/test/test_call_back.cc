// --- 添加开始 ---
#include <pthread.h>
// 强制修复 Ubuntu 22.04 下 Boost 库的 PTHREAD_STACK_MIN 错误
#ifdef PTHREAD_STACK_MIN
  #undef PTHREAD_STACK_MIN
#endif
#define PTHREAD_STACK_MIN 16384
// --- 添加结束 ---

#include <iostream>
#include <functional> // for std::function
#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <algorithm>
#include <chrono>
#include <dirent.h>
#include <sys/stat.h>
#include <thread>
#include <cmath>
#include <filesystem>

// PCL头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// log 日志
#include <glog/logging.h>
// 命令行参数解析
#include <gflags/gflags.h>

// 包含系统中的点类型
#include "common/point_types.h"
#include "ch7/ndt_3d.h"
#include "common/point_cloud_utils.h"
#include "common/math_utils.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"
#include "common/timer/timer.h"
#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "tools/pcl_map_viewer.h"

// ROS消息头文件
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/message_instance.h>

// ROS2 的文件
// #include "rclcpp/rclcpp.hpp"

// // ros 头文件
// 原始的ULHK数据集配置
DEFINE_string(bag_path, "./dataset/sad/ulhk/test3.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "./config/velodyne_ulhk.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

// 自定义数据集配置 (My_dataset - CS30)
// DEFINE_string(custom_bag_path, "./dataset/sad/ulhk/cs30_ros1_converted.bag", "path to custom rosbag");
DEFINE_string(custom_bag_path, "./dataset/sad/ulhk/cs30_ros1_converted-04.bag", "path to custom rosbag");

DEFINE_string(custom_dataset_type, "CUSTOM", "Custom dataset type");           // 自定义数据集类型
DEFINE_string(custom_config, "./config/velodyne_ulhk.yaml", "path of custom config yaml");
DEFINE_string(pointcloud_topic, "/camera1_SD0140820L0057/points2", "PointCloud2 topic name");
DEFINE_string(imu_topic, "/imu/data", "IMU topic name");
DEFINE_bool(use_custom_dataset, true, "use custom dataset?");


//  创建C++的订阅文件


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_XYZ;

// 模拟IMU数据
struct IMUData {
    double timestamp;
    double gyro_x, gyro_y, gyro_z;  // 陀螺仪数据
    double accel_x, accel_y, accel_z; // 加速度计数据

    IMUData(double t, double gx, double gy, double gz, double ax, double ay, double az)
        : timestamp(t), gyro_x(gx), gyro_y(gy), gyro_z(gz), accel_x(ax), accel_y(ay), accel_z(az) {}
};

// 模拟激光雷达点云数据
struct PointCloud {
    double timestamp;
    std::vector<std::pair<double, double>> points; // 简化的点云数据 (x, y)

    PointCloud(double t) : timestamp(t) {}

    void addPoint(double x, double y) {
        points.emplace_back(x, y);
    }
};

// 同步后的测量组
struct MeasurementGroup {
    double lidar_begin_time = 0;
    double lidar_end_time = 0;
    std::shared_ptr<PointCloud> lidar_data = nullptr;
    std::queue<std::shared_ptr<IMUData>> imu_data;

    MeasurementGroup() {
        lidar_data = std::make_shared<PointCloud>(0);
    }
};

// 回调函数类型定义
using ProcessCallback = std::function<void(const MeasurementGroup&)>;

/**
 * 模拟MessageSync类，用于同步IMU和激光雷达数据
 */
class MessageSync {
public:
    using Callback = ProcessCallback;

    MessageSync(Callback cb) : callback_(cb) {}

    // 处理IMU数据
    void ProcessIMU(std::shared_ptr<IMUData> imu) {
        double timestamp = imu->timestamp;
        if (timestamp < last_timestamp_imu_) {
            std::cout << "IMU时间回退，清空缓冲区" << std::endl;
            while (!imu_buffer_.empty()) {
                imu_buffer_.pop();
            }
        }

        last_timestamp_imu_ = timestamp;
        imu_buffer_.push(imu);
    }

    // 处理激光雷达点云数据
    void ProcessPointCloud(std::shared_ptr<PointCloud> cloud) {
        if (cloud->timestamp < last_timestamp_lidar_) {
            std::cout << "激光雷达时间回退，清空缓冲区" << std::endl;
            while (!lidar_buffer_.empty()) {
                lidar_buffer_.pop();
            }
        }

        last_timestamp_lidar_ = cloud->timestamp;
        lidar_buffer_.push(cloud);

        // 尝试同步数据
        Sync();
    }

private:
    /**
     * 尝试同步IMU与激光雷达数据，成功时调用回调函数
     */
    bool Sync() {
        if (lidar_buffer_.empty() || imu_buffer_.empty()) {
            return false;
        }

        if (!lidar_pushed_) {
            measures_.lidar_data = lidar_buffer_.front();
            measures_.lidar_begin_time = measures_.lidar_data->timestamp;
            measures_.lidar_end_time = measures_.lidar_data->timestamp + 0.1; // 假设激光雷达扫描持续0.1秒
            lidar_pushed_ = true;
        }

        // 确保有足够的IMU数据覆盖整个激光雷达扫描周期
        if (last_timestamp_imu_ < measures_.lidar_end_time) {
            return false;
        }

        // 提取时间范围内的IMU数据
        while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp <= measures_.lidar_end_time) {
            measures_.imu_data.push(imu_buffer_.front());
            imu_buffer_.pop();
        }

        lidar_buffer_.pop();
        lidar_pushed_ = false;

        // 调用回调函数处理同步后的数据
        if (callback_) {
            std::cout << "\n=== 数据同步完成，触发回调函数 ===" << std::endl;
            callback_(measures_); // 调用ProcessMeasurements
        }

        return true;
    }

    Callback callback_;                             // 回调函数
    std::queue<std::shared_ptr<PointCloud>> lidar_buffer_;  // 激光雷达数据缓冲
    std::queue<std::shared_ptr<IMUData>> imu_buffer_;       // IMU数据缓冲
    double last_timestamp_imu_ = -1.0;              // 最近IMU时间
    double last_timestamp_lidar_ = 0;               // 最近激光雷达时间
    bool lidar_pushed_ = false;
    MeasurementGroup measures_;
};

/**
 * 模拟LooselyLIO类，实现类似LIO的回调处理
 */
class SimulationLIO {
public:
    SimulationLIO() {
        // 初始化时设置回调函数
        sync_ = std::make_shared<MessageSync>([this](const MeasurementGroup &meas) {
            ProcessMeasurements(meas);
        });
    }

    // 激光雷达回调函数
    void PointCloudCallback(std::shared_ptr<PointCloud> cloud) {
        std::cout << "接收激光雷达数据，时间戳: " << cloud->timestamp
                  << ", 点数: " << cloud->points.size() << std::endl;
        sync_->ProcessPointCloud(cloud);
    }

    // IMU回调函数
    void IMUCallback(std::shared_ptr<IMUData> imu) {
        std::cout << "接收IMU数据，时间戳: " << imu->timestamp
                  << ", 陀螺: (" << imu->gyro_x << ", " << imu->gyro_y << ", " << imu->gyro_z << ")" << std::endl;
        sync_->ProcessIMU(imu);
    }

private:
    // 处理同步后的数据
    void ProcessMeasurements(const MeasurementGroup &meas) {
        std::cout << "进入ProcessMeasurements函数..." << std::endl;
        std::cout << "激光雷达数据时间范围: " << meas.lidar_begin_time << " - " << meas.lidar_end_time << std::endl;
        std::cout << "同步的IMU数据数量: " << meas.imu_data.size() << std::endl;

        // 模拟预测步骤
        Predict(meas);

        // 模拟点云去畸变
        // Undistort(meas);

        // 模拟配准和更新
        Align(meas);

        std::cout << "处理完成，返回主流程\n" << std::endl;
    }

    void Predict(const MeasurementGroup &meas) {
        std::cout << "  - 执行预测步骤，使用 " << meas.imu_data.size() << " 个IMU数据进行状态预测" << std::endl;
        // 这里可以模拟EKF的预测过程
    }

    void Undistort(const MeasurementGroup &meas) {
        std::cout << "  - 执行点云去畸变，处理激光雷达时间范围内的点云数据" << std::endl;
        // 这里可以模拟点云去畸变过程
    }

    void Align(const MeasurementGroup &meas) {
        std::cout << "  - 执行配准和状态更新" << std::endl;
        // 这里可以模拟点云配准和EKF更新过程
    }

    std::shared_ptr<MessageSync> sync_ = nullptr;  // 消息同步器
};

std::string Frame_pcd_dir = "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames_pcd";
std::string Frame_pcd_dir_ros2 = "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames_pcd_ros2";

std::string frame_0 = "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames_pcd/frame_000000.pcd";
std::string output_cloud_path = "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/output_cloud.pcd";


/**
 * 两帧雷达间的imu积分，只采取陀螺仪积分角度，再加上之前的世界位姿，推算下一时刻的世界位姿
 */
SE3 IntegrateIMU(std::vector<IMUPtr> &imu_range, const SE3& last_pose_world)
{
    // 初始化增量
    Eigen::Vector3d P_wb = last_pose_world.translation();
    Eigen::Quaterniond Q_wb = last_pose_world.unit_quaternion();
    // Eigen::Vector3d V_w = current_velocity; // 上一时刻的世界坐标系速度

    double dt = 0.01;  // 时间间隔假设为 0.01
    for (const auto & imu : imu_range)
    {
        // 旋转的增量
        Eigen::Vector3d gyro_dt = imu->gyro_ * dt;
        Eigen::Quaterniond dq(1, gyro_dt.x() * 0.5, gyro_dt.y() * 0.5, gyro_dt.z() * 0.5);
        dq.normalize();
        Q_wb = Q_wb * dq;
        Q_wb.normalize();

        // 平移的增量
        // imu速度转换到世界坐标系下
        Eigen::Vector3d acc_w = Q_wb * imu->acce_;
        // 计算位置增量
        P_wb = P_wb + acc_w * dt * dt + 0.5 * acc_w * dt * dt;

    }
    return SE3(Q_wb, P_wb);
}

void ReadandShowFrame()
{
    // 打印测试的相关pcd 点云
    PointCloud_XYZ::Ptr cloud(new PointCloud_XYZ);
    pcl::io::loadPCDFile(frame_0, *cloud);
    LOG(INFO) << "点云的大小" << cloud->width * cloud->height;

    // 创建可视化容器
    pcl::visualization::PCLVisualizer viewer("PCD viewer");
    viewer.setBackgroundColor(0,0,0);
    viewer.addPointCloud(cloud, "cloud");
    // viewer.initCameraParameters();
    
    while ((!viewer.wasStopped()))
    {
        viewer.spinOnce(100);
    }
}


void PlayFrames(const std::string& folder, std::vector<sad::CloudPtr> &frames_sad, int start_id, int count, bool &is_vis)
{
    LOG(INFO) << "开始加载点云文件";

    // 装载点云缓存，提高播放效率
    frames_sad.reserve(count);

    for (int i = 0; i < count; i++)
    {
        int frame_id = start_id + i;

        char filename[256];
        sprintf(filename, "%s/frame_%06d.pcd", folder.c_str(), frame_id);

        // 先用PCL加载点云
        PointCloud_XYZ::Ptr pcl_cloud(new PointCloud_XYZ);
        if (pcl::io::loadPCDFile(filename, *pcl_cloud) != 0) {
            std::cerr << "无法加载: " << filename << std::endl;
            continue;
        }

        // 转换为sad::CloudPtr格式，便于NDT使用
        sad::CloudPtr sad_cloud(new sad::PointCloudType);
        sad_cloud->resize(pcl_cloud->size());
        for (size_t j = 0; j < pcl_cloud->size(); ++j) {
            sad::PointType pt;
            pt.x = pcl_cloud->points[j].x;
            pt.y = pcl_cloud->points[j].y;
            pt.z = pcl_cloud->points[j].z;
            pt.intensity = 1e-6;  // 采取统一的默认值
            sad_cloud->points[j] = pt;
        }
        sad_cloud->width = pcl_cloud->width;
        sad_cloud->height = pcl_cloud->height;
        sad_cloud->is_dense = pcl_cloud->is_dense;

        frames_sad.push_back(sad_cloud);
        LOG(INFO) << "加载成功：" << filename << " 点数: " << sad_cloud->size();
    }

    LOG(INFO) << "结束加载点云文件，共加载 " << frames_sad.size() << " 个点云";

    if (frames_sad.empty()) {
        std::cerr << "没有有效的帧，无法播放!" << std::endl;
        return;
    }

    // 可视化部分（可选）
    LOG(INFO) << "开始可视化点云";
    pcl::visualization::PCLVisualizer viewer("PCD Sequence Player");
    viewer.setBackgroundColor(0, 0, 0);

    // 用于重复更新的对象名
    const std::string cloud_id = "cloud";

    // 将当前sad格式的点云转换为PCL格式以进行可视化
    std::vector<PointCloud_XYZ::Ptr> vis_clouds(frames_sad.size());
    for (size_t idx = 0; idx < frames_sad.size(); ++idx) {
        vis_clouds[idx] = PointCloud_XYZ::Ptr(new PointCloud_XYZ);
        vis_clouds[idx]->resize(frames_sad[idx]->size());
        for (size_t i = 0; i < frames_sad[idx]->size(); ++i) {
            pcl::PointXYZ pt;
            pt.x = frames_sad[idx]->points[i].x;
            pt.y = frames_sad[idx]->points[i].y;
            pt.z = frames_sad[idx]->points[i].z;
            vis_clouds[idx]->points[i] = pt;
        }
        vis_clouds[idx]->width = frames_sad[idx]->width;
        vis_clouds[idx]->height = frames_sad[idx]->height;
        vis_clouds[idx]->is_dense = frames_sad[idx]->is_dense;
    }


    viewer.addPointCloud(vis_clouds[0], cloud_id);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_id);

    // 循环播放显示点云
    int idx = 0;
    LOG(INFO) << "点云开始播放";
    while (!viewer.wasStopped() && idx < frames_sad.size() && is_vis)
    {
        viewer.spinOnce(100);

        // 500 ms 切换下一帧
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 更新点云内容
        viewer.updatePointCloud(vis_clouds[idx], cloud_id);
        viewer.setWindowName("PCD Player - Frame " + std::to_string(start_id + idx) + ", Points: " + std::to_string(vis_clouds[idx]->size()));

        idx = idx + 1;
        // LOG(INFO) << "已显示第 " << idx << " 帧";
    }
    LOG(INFO) << "点云播放完毕";
}



SE3 last_kf_pose = SE3();
float leafsize = 0.3;


 // 检测是否为关键帧
 bool IsKeyframe(const SE3& current_pose)
 {
    SE3 delta = last_kf_pose.inverse() * current_pose;
    // return delta.translation().norm() > 0.5 || delta.so3().log().norm() > 10 * sad::math::kDEG2RAD;  // 度转化为弧度 deg -> rad    室外的参数
    return delta.translation().norm() > 0.1 || delta.so3().log().norm() > 3 * sad::math::kDEG2RAD;  // 度转化为弧度 deg -> rad   室内的参数

}
 void loadRosbagImuData(  std::vector<IMUPtr>  &imu_data_buffer)
{
    // 加载rosbag中的imu数据
    sad::RosbagIO rosbag_io(FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));

    LOG(INFO) << "开始加载imu的数据" ;
    rosbag_io
        .AddImuHandle([&](IMUPtr imu) {
            imu_data_buffer.emplace_back(imu);

            // LOG(INFO) << "11111" ;
            // std::cout << " 正在读取相关的imu数据" << std::endl;
            // 显示IMU数据
            // std::cout << "IMU数据 - 时间戳: " << imu->timestamp_
            //           << ", 陀螺仪: (" << imu->gyro_.x() << ", " << imu->gyro_.y() << ", " << imu->gyro_.z() << ")"
            //           << ", 加速度: (" << imu->acce_.x() << ", " << imu->acce_.y() << ", " << imu->acce_.z() << ")"
            //           << std::endl;

            return true;
        })
        .Go();
    LOG(INFO) << "结束加载imu的数据" ;
    std::cout << "总共读取了 " << imu_data_buffer.size() << " 条IMU数据" << std::endl;
}

/**
 * @brief 从自定义rosbag加载IMU数据 (支持自定义话题名称)
 * @param imu_data_buffer IMU数据缓冲区
 */
void loadCustomRosbagImuData(std::vector<std::shared_ptr<sad::IMU>>& imu_data_buffer)
{
    // 加载自定义rosbag中的imu数据，使用MY_dataset类型
    sad::RosbagIO rosbag_io(FLAGS_custom_bag_path, sad::DatasetType::CS_30);

    LOG(INFO) << "开始加载自定义rosbag的IMU数据，话题: " << FLAGS_imu_topic;
    
    // 使用AddHandle通用处理函数处理自定义话题的IMU数据
    rosbag_io.AddHandle(FLAGS_imu_topic, [&](const rosbag::MessageInstance &m) -> bool {
        auto msg = m.instantiate<sensor_msgs::Imu>();
        if (msg == nullptr) {
            return false;
        }
        
        // 创建IMU对象
        auto imu = std::make_shared<sad::IMU>(
            msg->header.stamp.toSec(),
            Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)
        );
        imu_data_buffer.emplace_back(imu);
        return true;
    }).Go();
    
    LOG(INFO) << "结束加载自定义rosbag的IMU数据";
    std::cout << "总共读取了 " << imu_data_buffer.size() << " 条IMU数据" << std::endl;
}

/**
 * @brief 从自定义rosbag加载点云数据 (支持自定义话题名称)
 * @param cloud_data_buffer 点云数据缓冲区
 */
void loadCustomRosbagPointCloudData(std::vector<sad::CloudPtr>& cloud_data_buffer)
{
    // 加载自定义rosbag中的点云数据，使用MY_dataset类型
    sad::RosbagIO rosbag_io(FLAGS_custom_bag_path, sad::DatasetType::CS_30);

    LOG(INFO) << "开始加载自定义rosbag的点云数据，话题: " << FLAGS_pointcloud_topic;
    
    // 使用AddPointCloud2Handle处理PointCloud2消息
    rosbag_io.AddPointCloud2Handle(FLAGS_pointcloud_topic, [&](sensor_msgs::PointCloud2::Ptr msg) -> bool {
        // 将PointCloud2消息转换为sad::CloudPtr格式
        sad::FullCloudPtr full_cloud(new sad::FullPointCloudType);
        pcl::fromROSMsg(*msg, *full_cloud);
        
        // 转换为sad::CloudPtr (PointType格式)
        sad::CloudPtr cloud(new sad::PointCloudType);
        cloud->resize(full_cloud->size());
        for (size_t i = 0; i < full_cloud->size(); ++i) {
            cloud->points[i].x = full_cloud->points[i].x;
            cloud->points[i].y = full_cloud->points[i].y;
            cloud->points[i].z = full_cloud->points[i].z;
            cloud->points[i].intensity = full_cloud->points[i].intensity;
        }
        cloud->width = full_cloud->width;
        cloud->height = full_cloud->height;
        cloud->is_dense = full_cloud->is_dense;
        
        cloud_data_buffer.emplace_back(cloud);
        return true;
    }).Go();
    
    LOG(INFO) << "结束加载自定义rosbag的点云数据";
    std::cout << "总共读取了 " << cloud_data_buffer.size() << " 帧点云数据" << std::endl;
}


//  TODO 
// 书写一个ros1的读取path下的rosbag内的imu的数据，然后再将他存到buffer
void LoadImuDataFromRosbag1(std::string path , std::string topic, std::vector<IMUPtr>  &imu_data_buffer) 
{

}

// 读取范围内的imu数据
bool GetIMUsInTimeRange(const std::vector<IMUPtr>& buffer, double start_time, double end_time, 
                        std::vector<IMUPtr>& output_imus, size_t& current_imu_idx) 
{
     std::cout<< "start_time: " << start_time << std::endl;    
    // 简单遍历，实际可以使用二分查找优化
    while (current_imu_idx < buffer.size()) {
        if (buffer[current_imu_idx]->timestamp_ < start_time) {
            current_imu_idx++;
            continue;
        }
        if (buffer[current_imu_idx]->timestamp_ > end_time) {
            break; // 超出范围
        }
        output_imus.push_back(buffer[current_imu_idx]);
        current_imu_idx++;
    }
    return !output_imus.empty();
}

void GetIMUData(const std::vector<IMUPtr>& buffer, std::vector<IMUPtr>& output_imus, 
                size_t start, size_t last)
{
    start = start * 10;
    
    for (size_t i = start; i <= start + last && i < buffer.size(); ++i) {
        output_imus.push_back(buffer[i]);
    }
}

 void test_templocal_lo(bool & is_vis)
 {
    // 是否可视化加载的数据
    is_vis = false;
    int start_frame = 900;
    int last_frame = 300;
    double scan_interval = 0.1; 

    // 雷达和imu的外参
    std::vector<double> ext_t = {0.0, 0.0, -0.28};
    std::vector<double> ext_r = {
                    2.67949e-08, -1.0, 0.0,    // 第一行
                    1.0, 2.67949e-08, 0.0,     // 第二行  
                    0.0, 0.0, 1.0              // 第三行
                };
    Vec3d Lidar_T_wrt_IMU = sad::math::VecFromArray(ext_t);
    Mat3d Lidar_R_wrt_IMU = sad::math::MatFromArray(ext_r);
    SE3 T_IL = SE3(Lidar_R_wrt_IMU, Lidar_T_wrt_IMU);

    std::vector<sad::CloudPtr> frames_sad;  // 使用sad::CloudPtr格式的点云容器
    std::vector<IMUPtr> imu_data_buffer;     // 加载相关的imu的数据

    // 加载所有的imu数据
    loadRosbagImuData(imu_data_buffer);

    // 当前第一帧的时间为

    // 进行imu的初始化
    sad::StaticIMUInit imu_init;
    sad::StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;
    imu_init = sad::StaticIMUInit(imu_init_options);

    // 创建eskf的滤波器
    sad::ESKFD eskf;
    sad::ESKFD::Options eskf_options;
    
    // -----------------加载数据进行imu的初始化--------------
    for (int i = 0; i < 3000; ++i) {
        imu_init.AddIMU(*imu_data_buffer[i]);
    }
    
    if( imu_init.InitSuccess()) {
        // 从初始化中读取参数
        eskf_options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);   // 假设各向同性，只读取第一个，代表三个轴的数值
        eskf_options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
        eskf.SetInitialConditions(eskf_options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());    
        LOG(INFO) << "eskf 初始化成功";
    }

    LOG(INFO) << "测试里程计程序启动" ;
    // ReadandShowFrame();


    // 读取相关的点云数据
    // 现在直接加载为sad::CloudPtr格式，便于NDT使用
    PlayFrames(Frame_pcd_dir, frames_sad, start_frame, last_frame, is_vis);  // 
    LOG(INFO) << "加载完成 " << frames_sad.size() << " 个点云";

    if (frames_sad.size() < 2) {
        LOG(WARNING) << "点云数量不足，无法进行NDT配准";
        LOG(INFO) << "测试程序结束" ;
        return;
    }

    LOG(INFO) << "开始使用NDT算法进行点云配准";

    // 创建NDT对象
    sad::Ndt3d::Options ndt_options;
    ndt_options.voxel_size_ = 0.5;      // 设置体素大小为0.5米
    ndt_options.max_iteration_ = 30;     // 最大迭代次数
    ndt_options.min_effective_pts_ = 5;  // 最小有效点数
    sad::Ndt3d ndt(ndt_options);

    sad::CloudPtr output_cloud(new sad::PointCloudType);    // 输出点云

    // 取出第0个和第1个点云进行NDT配准
    sad::CloudPtr target_cloud = sad::CloudPtr(new sad::PointCloudType);    // 目标点云
    sad::CloudPtr scan = sad::CloudPtr(new sad::PointCloudType);            // 源点云
    sad::CloudPtr local_map = sad::CloudPtr(new sad::PointCloudType);    // 输出点云
    sad::CloudPtr scan_world = sad::CloudPtr(new sad::PointCloudType);      // 变换后的源点云
    sad::CloudPtr scan_0 = sad::CloudPtr(new sad::PointCloudType);      // 变换后的源点云

    std::vector<SE3> estimated_poses;
    std::deque<sad::CloudPtr> scan_wolrd_in_local_map;
    SE3 guess = SE3();

    // 创建体素化的的相关对象，优化地图
    pcl::VoxelGrid<sad::PointType> voxel_grid;
    voxel_grid.setLeafSize(leafsize, leafsize, leafsize);
    
    pcl::transformPointCloud(*frames_sad[0], *scan_0, T_IL.matrix());
    frames_sad[0] = scan_0;

    // 地0帧加入局部地图
    scan_wolrd_in_local_map.emplace_back(frames_sad[0]);
    ndt.SetTarget(frames_sad[0]);  // 设置初始目标点云


    // 添加imu的相关配置文件
    double last_scan_time = start_frame * 10;   // imu的当前的时间戳
    size_t current_imu_idx = 0;

    // 开始遍历所有的点云
    for(int i = 1; i < frames_sad.size(); i++)
    {

        double current_sacn_time = last_scan_time + scan_interval;
        std::cout << " -------" <<"当前是第几 ：" <<i << "-------" <<std::endl;

        // ----------- IMU数据 --------------------
        // ----------  开始预测
        // 现在默认一帧的雷达读取相关的10帧的imu数据
        std::vector<IMUPtr> range_imus;
        // GetIMUsInTimeRange(imu_data_buffer, start_frame * 10 + i, start_frame * 10 + 10* i, range_imus, current_imu_idx);
        GetIMUData(imu_data_buffer, range_imus, start_frame + i -1, 10);
        for (auto& imu : range_imus) {
            eskf.Predict(*imu);
        }


        // ----------- 开始Ndt配准 ------------------------------
        // ----------- 读取雷达数据 -----------------
        sad::CloudPtr frames_sad_voxel = sad::CloudPtr(new sad::PointCloudType);          // 体素化后的点云
        sad::CloudPtr frames_sad_voxel_trans_imu = sad::CloudPtr(new sad::PointCloudType);    // 体素化后的点云_变换到imu坐标系

        // 体素化
        std::cout << "当前点云大小: " << frames_sad[i]->size() << std::endl;
        voxel_grid.setInputCloud(frames_sad[i]);
        voxel_grid.filter(*frames_sad_voxel);
        std::cout << "体素化后点云大小: " << frames_sad_voxel->size() << std::endl;

        // 当前点云体素化之后的点云转换到imu坐标系
        pcl::transformPointCloud(*frames_sad_voxel, *frames_sad_voxel_trans_imu, T_IL.matrix());
        frames_sad_voxel = frames_sad_voxel_trans_imu;

        ndt.SetSource(frames_sad_voxel);
        // ndt.SetTarget(frames_sad[i-1]);


        // NDT 匹配
        // 1 恒速模型配准开始ndt的配准
        // if ( estimated_poses.size() < 2) {
        //     // 第一次迭代时，使用初始猜测
        //     ndt.AlignNdt(guess);
        // // } else {
        // //     // 采取恒速模型预测相关的初始数值
        // //     SE3 T1 = estimated_poses[estimated_poses.size() - 1];  // 上一帧的估计位姿
        // //     SE3 T2 = estimated_poses[estimated_poses.size() - 2];  // 上上一帧的估计位姿
        // //     guess = T1 * (T2.inverse() * T1 );  // 初始猜测为上一帧的逆变换
        // //     ndt.AlignNdt(guess);
        // } else {
        //     ndt.AlignNdt(guess);
        //     std::cout << "N修正前位姿:\n " << guess.matrix() << std::endl;
        // }


        // ----------- 开始配准------------
        // 从 imu 获取相关的姿态
        SE3 pose_guess = eskf.GetNominalSE3();
        LOG(INFO) <<  "修正前位姿 pose_guess:\n " << pose_guess.matrix();

        // 2 使用imu得到的数据配准
        if (estimated_poses.size() < 2)
        {
            SE3 pose;
            ndt.AlignNdt(pose);
        } else {
            ndt.AlignNdt(pose_guess);
        }
        // 加入到估计位姿
        estimated_poses.emplace_back(pose_guess);
        
        // 变换当前帧到局部地图坐标系
        // sad::CloudPtr scan_world = sad::CloudPtr(new sad::PointCloudType);      // 变换后的源点云
        scan_world.reset(new sad::PointCloudType);
        pcl::transformPointCloud(*frames_sad_voxel, *scan_world, pose_guess.matrix().cast<float>());

        SE3 pose_of_lo = pose_guess;
        // 将结果修正观测
        eskf.ObserveSE3(pose_of_lo, 1e-2, 1e-2);

        // 修正之后的结果
        SE3 pose_fuesed = eskf.GetNominalSE3();

        // 打印修正后的位姿
        std::cout << "修正后的位姿:\n " << pose_fuesed.matrix() << std::endl;


        if (IsKeyframe(pose_fuesed))
        {
            last_kf_pose = pose_fuesed;

            // 加入到局部地图
            scan_wolrd_in_local_map.emplace_back(scan_world);
            if (scan_wolrd_in_local_map.size() > 30) {
                scan_wolrd_in_local_map.pop_front();
            }

            local_map.reset(new sad::PointCloudType);

            for (auto& scan : scan_wolrd_in_local_map) {
                *local_map += *scan;
            }


            // 打印出来当前的点云大小
            LOG(INFO) << "当前第 " << i << " 帧, local_map大小: " << local_map->size();
            ndt.SetTarget(local_map);  // 设置新的目标点云

            // *output_cloud += *local_map;
            *output_cloud += *scan_world;

        }
        last_scan_time = current_sacn_time;
    }
    LOG(INFO) << "imu_data_buffer[0]->timestamp_" <<     imu_data_buffer[0]->timestamp_;
    LOG(INFO) << "开始存储地图, 地图大小: " << local_map->size();
    // 保存最终的点云结果
    sad::CloudPtr output_voxel = sad::CloudPtr(new sad::PointCloudType);

    if (local_map->size() > 60000) {
        voxel_grid.setLeafSize(leafsize, leafsize, leafsize);
        LOG(INFO) << "地图过大，进行体素化";
        LOG(INFO) << " 体素化前地图大小: " << output_cloud->size();
        voxel_grid.setInputCloud(output_cloud);
        // output_cloud.reset(new sad::PointCloudType);
        voxel_grid.filter(*output_voxel);
        LOG(INFO) << " 体素化后地图大小: " << output_voxel->size();
    }
    sad::SaveCloudToFile(output_cloud_path, *output_voxel);



    LOG(INFO) << "测试里程计程序结束" ;
 }



 void test_Ndt_LO(bool &is_vis)
 {
    is_vis = false;

    //  自身的配置文件
    // int start_frame = 0;
    // int last_frame = 500;
    // double voxel_size = 0.05;
    // int num_kfs_in_local_map = 30;   // 组成局部地图的关键帧数量
 

    // ULHK
    int start_frame = 800;
    int last_frame = 500;
    double voxel_size = 1.0;
    int num_kfs_in_local_map = 30;   // 组成局部地图的关键帧数量
    bool use_consistent_model = false;
    bool use_imu_guess = false;
    bool use_hybird_guess = true;


    // ---------- 加载所有的点云数据
    std::vector<sad::CloudPtr> frames_sad;
    // PlayFrames(Frame_pcd_dir_ros2, frames_sad, start_frame, last_frame, is_vis);      // 使用自己的数据集
    PlayFrames(Frame_pcd_dir, frames_sad, start_frame, last_frame, is_vis);           // 使用ulhk的数据集

    LOG(INFO) << "加载完成 " << frames_sad.size() << " 帧点云";

    // 开始imu的相关的数据
    double scan_interval = 0.1;              // 扫描时间的内容
    std::vector<IMUPtr> imu_data_buffer;     // 加载相关的imu的数据
    
    // 开始加载相关的imu数据 到 buffer
    loadRosbagImuData(imu_data_buffer);      

    // ---------- 创建ndt的配准对
    sad::Ndt3d::Options ndt_options;
    // ndt_options.voxel_size_ = 0.15;          // 室内数据集的参数
    ndt_options.voxel_size_ = 0.5;          // 室内数据集的参数

    ndt_options.max_iteration_ = 30;
    ndt_options.min_effective_pts_ = 5;
    sad::Ndt3d ndt(ndt_options);

    sad::CloudPtr target_cloud(new sad::PointCloudType);  // ndt 匹配的target目标点云
    sad::CloudPtr source_cloud(new sad::PointCloudType);  // ndt 匹配的source点云
    sad::CloudPtr loacl_map(new sad::PointCloudType);     // 局部点云地图
    sad::CloudPtr output_cloud(new sad::PointCloudType);  // 保存的结果点云

    std::vector<SE3> estimated_pose;                      // 保存估计的位姿，用于后续的恒速模型
    std::deque<sad::CloudPtr> scan_world_local;

    // 开始处理第一帧的数据
    ndt.SetTarget(frames_sad[0]);
    scan_world_local.emplace_back(frames_sad[0]);

    // 体素化相关的体积
    pcl::VoxelGrid<sad::PointType> voxel_grid;
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);

    // 正式开始遍历点云
    for (int i = 1; i < frames_sad.size(); ++i)
    {
        LOG(INFO) << "---当前处理的第几--- " << i ;

        // 将当前的每一帧体素化
        voxel_grid.setInputCloud(frames_sad[i]);
        LOG(INFO) << "体素之前的大小为" << frames_sad[i]->size();
        voxel_grid.filter(*source_cloud);
        LOG(INFO) << "体素之后的大小为" << source_cloud->size();

        // ndt 设置当前的帧
        ndt.SetSource(source_cloud);

        // ----------- 开始配准
        // ndt 匹配的初始数值选择

        //------------- 进行imu的积分，估计这段时间的位姿
        std::vector<IMUPtr> range_imus;
        GetIMUData(imu_data_buffer, range_imus, start_frame + i -1, 8);
        LOG(INFO) << "当前缓存的imu积分的数据为" <<range_imus.size();

        // 获取当前时刻的位置
        SE3 last_pose = estimated_pose.empty() ? SE3() : estimated_pose.back();
        

        SE3 guess;
        if ( estimated_pose.size() < 2 ) {
            ndt.AlignNdt(guess);
        } else if (use_consistent_model){
            // 采用恒速模型进行预测
            SE3 T1 = estimated_pose[estimated_pose.size() - 1];
            SE3 T2 = estimated_pose[estimated_pose.size() - 2];
            guess = T1 * (T2.inverse() * T1);
            ndt.AlignNdt(guess);
        } else if (use_imu_guess){
        
            // 开始积分得到imu递推的这段时间的位置
            SE3 pose_imu = IntegrateIMU(range_imus, last_pose);
            LOG(INFO) << "imu 预测的位姿为pose_imu \n" << pose_imu.matrix();

            ndt.AlignNdt(pose_imu);
            guess = pose_imu;
        } else if (use_hybird_guess) {
            // 恒速模型的数据的位移
            SE3 T1 = estimated_pose[estimated_pose.size() - 1];
            SE3 T2 = estimated_pose[estimated_pose.size() - 2];
            guess = T1 * (T2.inverse() * T1);
            Eigen::Vector3d P_delta = guess.translation();

            // imu的旋转角度
            SE3 pose_imu = IntegrateIMU(range_imus, last_pose);
            Eigen::Quaterniond Q_delta = pose_imu.unit_quaternion();
            // SE3 

            // 拼接得到整体的位移位置猜测
            SE3 guess = SE3(Q_delta, P_delta);
            ndt.AlignNdt(guess);
        }
        SE3 pose = guess;
        estimated_pose.emplace_back(pose);
        LOG(INFO) << "当前的位姿为pose \n" << pose.matrix();

        // 将当前帧转换到世界坐标系下
        sad::CloudPtr source_cloud_world(new sad::PointCloudType);  // ndt 匹配的source点云
        pcl::transformPointCloud(*source_cloud, * source_cloud_world, pose.matrix());

        if (IsKeyframe(pose)) {
            LOG(INFO) << " 检测到关键帧";
            // 保存当前的关键帧
            last_kf_pose = pose;

            // 加入到局部地图缓存队列
            scan_world_local.emplace_back(source_cloud_world);
            if ( scan_world_local.size() > num_kfs_in_local_map ){
                scan_world_local.pop_front();
            }

            // 当前的局部地图大小
            loacl_map.reset(new sad::PointCloudType);   // 重置缓存大小
            for (auto& scan : scan_world_local) {
                *loacl_map += *scan;
            }

            LOG(INFO) << "设置当前的local_map的大小" << loacl_map->size();
            // 将新的local_map设置为target
            ndt.SetTarget(loacl_map);

            // 保存结果点云
            *output_cloud += *source_cloud_world;
            // *output_cloud += *loacl_map ;
        }
    }

    LOG(INFO) << " 开始存储点云地图 地图大小为" << output_cloud->size();

    if (output_cloud->size() > 100000)
    {
        sad::CloudPtr output_voxel(new sad::PointCloudType);
        voxel_grid.setInputCloud(output_cloud);
        voxel_grid.filter(*output_voxel);
        LOG(INFO) << "输出点云过大，进行体素化，体素化后大小为: " << output_voxel->size();
        sad::SaveCloudToFile(output_cloud_path, *output_voxel);
    } else {
        sad::SaveCloudToFile(output_cloud_path, *output_cloud);
    }

 }


void test_rotate()
{
    Eigen::Matrix3d R;
    R << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "原始矩阵 R:\n" << R << std::endl;


    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxis rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));  // 围绕z轴旋转45
    std::cout.precision(3);
    
    Eigen::Vector3d v(1, 0, 0);
    std::cout << "旋转之前的向量 v:\n" << v.transpose() << std::endl;

    rotation_matrix = rotation_vector.toRotationMatrix();
    std::cout<< "rotation_vector.matrix() = \n" << rotation_vector.matrix() << std::endl;
    std::cout<< "rotation_vector.toRotationMatrix() \n" << rotation_vector.toRotationMatrix() << std::endl;

    Eigen::Vector3d v_rotated = rotation_vector * v;
    std::cout << "旋转后的向量 v_rotated:\n" << v_rotated.transpose() << std::endl;

    // euler角
    Eigen::Vector3d euler_angels = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序
    std::cout << "Euler角 (ZYX顺序):\n" << euler_angels.transpose() << std::endl;

    // 齐次变换
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(1, 2, 3));
    std::cout << "齐次变换矩阵 T:\n" << T.matrix() << std::endl;

    // 四元数
    Eigen::Quaterniond q(rotation_vector);
    std::cout << "四元数 q:\n" << q.coeffs().transpose() << std::endl; // coeffs顺序为 (x, y, z, w)  虚部在前，实部在后

    q = Eigen::Quaterniond(rotation_matrix);
    std::cout << "由旋转矩阵构造的四元数 q:\n" << q.coeffs().transpose() << std::endl;

    v_rotated = q * v;
    std::cout << "用四元数旋转后的向量 v_rotated:\n" << v_rotated.transpose() << std::endl;

    // so3李群
    Sophus::SO3d SO3_R(rotation_matrix);
    Sophus::SO3d SO3_q(q);
    std::cout << "Sophus SO(3) from rotation matrix:\n" << SO3_R.matrix() << std::endl;
    std::cout << "Sophus SO(3) from quaternion:\n" << SO3_q.matrix() << std::endl;
    
    // so3李代数
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << "so3" << so3.transpose() << std::endl;

    // se3 李群
    Eigen::Vector3d t(1, 0, 0);
    Eigen::Matrix3d R_ = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SE3d SE3_Rt(R_, t);
    std::cout << "Sophus SE(3) from R,t:\n" << SE3_Rt.matrix() << std::endl;

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    std::cout << "se3: " << se3.transpose() << std::endl;

    // 演示查看如何跟新
    Vector6d updated_se3 = Vector6d::Zero();
    // 只更新位移第一个元素，第0行0列，而非第一个元素
    updated_se3(0, 0) = 0.0001;
    cout << "updated_se3: " << updated_se3.transpose() << std::endl;  
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(updated_se3) * SE3_Rt;
    cout << "SE3 updated = \n" << SE3_updated.matrix() << std::endl; 
}


void test_transfomr()
{
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    q1.normalize();
    q2.normalize();
    Eigen::Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.4, 0.2);
    Vec3d p1(0.5, 0, 0.2);

    Eigen::Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    Vec3d p2 = T2w*T1w.inverse()* p1;
    Eigen::Isometry3d T21 = T2w * T1w.inverse();
    std::cout <<" p2" << p2.transpose() << std::endl;
    std::cout <<" T1w \n" << T1w.matrix() << std::endl;
    std::cout <<" T2w \n" << T2w.matrix() << std::endl;
    std::cout <<" T21 \n" << T21.matrix() << std::endl;
    // T21.matrix().eulerAngles(2,1,0);
    std::cout << "T21 euler angles: \n" << T21.rotation().eulerAngles(2,1,0).transpose() << std::endl;
} 


void test_eskf_imu(std::vector<IMUPtr> & imu_data_buffer)
{
    loadRosbagImuData(imu_data_buffer);

    sad::ESKFD eskf;
    sad::ESKFD::Options eskf_options;

    // 初始化eskf 专门的类
    sad::StaticIMUInit imu_init;
    // 初始化imu初始化的相关配置
    sad::StaticIMUInit::Options imu_init_options;
    // 书写配置 不使用轮速计
    imu_init_options.use_speed_for_static_checking_ = false;
    // 写入相关的配置
    imu_init = sad::StaticIMUInit(imu_init_options);

    for (int i = 0; i < 3000; ++i) {
        imu_init.AddIMU(*imu_data_buffer[i]);
    }
    if (imu_init.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        eskf_options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
        eskf_options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
        eskf.SetInitialConditions(eskf_options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());

        LOG(INFO) << "gyro_var_:  " << eskf_options.gyro_var_;
        LOG(INFO) << "acce_var_:  " << eskf_options.acce_var_;
        LOG(INFO) << "InitBg:  " << imu_init.GetInitBg().transpose();
        LOG(INFO) << "InitBa:  " << imu_init.GetInitBa().transpose();
        LOG(INFO) << "Gravity:  " << imu_init.GetGravity().transpose();
        
        LOG(INFO) << "IMU静止结束成功";
    }

    eskf_options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
    eskf_options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
    eskf.SetInitialConditions(eskf_options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());

    LOG(INFO) << "IMU初始化成功";

    

}

/**
 * @brief 使用自定义CS30数据集进行NDT+IMU融合的里程计测试
 * @param is_vis 是否进行可视化
 */
void test_Ndt_LO_CustomDataset(bool& is_vis)
{
    is_vis = false;
    double voxel_size = 0.05;
    int num_kfs_in_local_map = 30;   // 组成局部地图的关键帧数量
    bool use_guess = true;           // 是否使用恒速模型进行预测
    bool use_imu_prediction = true;  // 是否使用IMU预测的位姿作为初始猜测
    
    LOG(INFO) << "========== 自定义数据集 (CS30) NDT+IMU 里程计测试开始 ==========";
    LOG(INFO) << "Rosbag路径: " << FLAGS_custom_bag_path;
    LOG(INFO) << "点云话题: " << FLAGS_pointcloud_topic;
    LOG(INFO) << "IMU话题: " << FLAGS_imu_topic;
    
    // ---------- 加载所有的点云数据
    std::vector<sad::CloudPtr> frames_sad;
    loadCustomRosbagPointCloudData(frames_sad);
    LOG(INFO) << "加载完成 " << frames_sad.size() << " 帧点云";

    if (frames_sad.size() < 2) {
        LOG(WARNING) << "点云数据不足，无法进行NDT配准";
        return;
    }

    // ---------- 加载IMU数据
    std::vector<std::shared_ptr<sad::IMU>> imu_data_buffer;
    loadCustomRosbagImuData(imu_data_buffer);

    LOG(INFO) << "加载完成 " << imu_data_buffer.size() << " 条IMU数据";

    if (imu_data_buffer.empty()) {
        LOG(WARNING) << "IMU数据为空，无法进行IMU初始化";
        return;
    }

    // ---------- 进行IMU初始化
    sad::StaticIMUInit imu_init;
    sad::StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false;
    imu_init = sad::StaticIMUInit(imu_init_options);

    // 使用前3000条IMU数据进行静止初始化
    int init_imu_count = std::min(3000, (int)imu_data_buffer.size());
    for (int i = 0; i < init_imu_count; ++i) {
        imu_init.AddIMU(*imu_data_buffer[i]);
    }

    LOG(INFO) << " imu的第一帧的数据为gyro_ \n" << imu_data_buffer[0]->gyro_ << "\n acce_ \n"<< imu_data_buffer[0]->acce_; 

    // sad::ESKFD eskf;
    // sad::ESKFD:GetIMUData:Options eskf_options;

    // if (imu_init.InitSuccess()) {
    //     eskf_options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
    //     eskf_options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
    //     eskf.SetInitialConditions(eskf_options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
    //     LOG(INFO) << "IMU初始化成功";
    // } else {
    //     LOG(WARNING) << "IMU初始化失败，使用默认配置";
    // }

    // ---------- 创建ndt的配准对象
    sad::Ndt3d::Options ndt_options;
    ndt_options.voxel_size_ = 0.15;
    ndt_options.max_iteration_ = 30;
    ndt_options.min_effective_pts_ = 5;
    sad::Ndt3d ndt(ndt_options);

    sad::CloudPtr target_cloud(new sad::PointCloudType);  // ndt 匹配的target目标点云
    sad::CloudPtr source_cloud(new sad::PointCloudType);  // ndt 匹配的source点云
    sad::CloudPtr loacl_map(new sad::PointCloudType);     // 局部点云地图
    sad::CloudPtr output_cloud(new sad::PointCloudType);  // 保存的结果点云

    std::vector<SE3> estimated_pose;
    std::deque<sad::CloudPtr> scan_world_local;

    // ---------- 处理第一帧数据
    ndt.SetTarget(frames_sad[0]);
    scan_world_local.emplace_back(frames_sad[0]);

    // 体素化相关的体积
    pcl::VoxelGrid<sad::PointType> voxel_grid;
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);

    // ---------- 正式开始遍历点云
    for (int i = 1; i < frames_sad.size(); ++i)
    {
        // LOG(INFO) << "---处理第 " << i << " 帧点云---";

        // 将当前的每一帧体素化
        voxel_grid.setInputCloud(frames_sad[i]);
        // LOG(INFO) << "体素化前大小: " << frames_sad[i]->size();
        voxel_grid.filter(*source_cloud);
        // LOG(INFO) << "体素化后大小: " << source_cloud->size();

        // ndt 设置当前的帧
        ndt.SetSource(source_cloud);

        // ----------- 开始配准
        // ndt 匹配的初始数值选择
        SE3 guess;
        if (estimated_pose.size() < 2) {
            ndt.AlignNdt(guess);
        } else if (use_guess){
            // 采用恒速模型进行预测
            SE3 T1 = estimated_pose[estimated_pose.size() - 1];
            SE3 T2 = estimated_pose[estimated_pose.size() - 2];
            guess = T1 * (T2.inverse() * T1);
            ndt.AlignNdt(guess);
        }else if (use_imu_prediction) {
            // 使用IMU预测的位姿作为初始猜测
            // SE3 imu_pose = eskf.GetNominalSE3();
            // LOG(INFO) << "使用IMU预测位姿作为初始猜测:\n" << imu_pose.matrix();
            // ndt.AlignNdt(imu_pose);
        } else {
            ndt.AlignNdt(guess);
        }
        SE3 pose = guess;
        estimated_pose.emplace_back(pose);
        // LOG(INFO) << "NDT配准后位姿:\n" << pose.matrix();

        // 将当前帧转换到世界坐标系下
        sad::CloudPtr source_cloud_world(new sad::PointCloudType);
        pcl::transformPointCloud(*source_cloud, *source_cloud_world, pose.matrix());

        // 检测是否为关键帧
        if (IsKeyframe(pose)) {
            // LOG(INFO) << "检测到关键帧";
            last_kf_pose = pose;

            // 加入到局部地图缓存队列
            scan_world_local.emplace_back(source_cloud_world);
            if (scan_world_local.size() > num_kfs_in_local_map) {
                scan_world_local.pop_front();
            }

            // 更新局部地图
            loacl_map.reset(new sad::PointCloudType);
            for (auto& scan : scan_world_local) {
                *loacl_map += *scan;
            }

            // LOG(INFO) << "局部地图大小: " << loacl_map->size();
            ndt.SetTarget(loacl_map);

            // 保存结果点云
            *output_cloud += *source_cloud_world;
        }
    }

    // ---------- 保存结果
    LOG(INFO) << "开始存储点云地图，地图大小: " << output_cloud->size();

    std::string output_path = "./dataset/sad/ulhk/cs30_output_cloud.pcd";
    if (output_cloud->size() > 100000) {
        sad::CloudPtr output_voxel(new sad::PointCloudType);
        voxel_grid.setInputCloud(output_cloud);
        voxel_grid.filter(*output_voxel);
        LOG(INFO) << "输出点云过大，进行体素化后大小: " << output_voxel->size();
        sad::SaveCloudToFile(output_path, *output_voxel);
    } else {
        sad::SaveCloudToFile(output_path, *output_cloud);
    }

    LOG(INFO) << "========== 自定义数据集 (CS30) NDT+IMU 里程计测试完成 ==========";
}

int main(int argc, char ** argv) {
    bool is_vis = true;
    // 启用日志系统
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    // 解析命令行参数
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "主程序启动";

    // 根据标志选择使用的数据集
    if (FLAGS_use_custom_dataset) {
        LOG(INFO) << "使用自定义数据集 (CS30)";
        test_Ndt_LO_CustomDataset(is_vis);
    } else {
        LOG(INFO) << "使用原始ULHK数据集";
        test_Ndt_LO(is_vis);
    }

    LOG(INFO) << "主程序结束";

    return 0;
}