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

// // ros 头文件
DEFINE_string(bag_path, "./dataset/sad/ulhk/test3.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "./config/velodyne_ulhk.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

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
std::string frame_0 = "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames_pcd/frame_000000.pcd";
std::string output_cloud_path = "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/output_cloud.pcd";
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
        viewer.spinOnce(10);

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
    return delta.translation().norm() > 0.5 || delta.so3().log().norm() > 10 * sad::math::kDEG2RAD;  // 度转化为弧度 deg -> rad
 }
 void loadRosbagImuData(  std::vector<IMUPtr>  &imu_data_buffer)
{
    // 加载rosbag中的imu数据
    sad::RosbagIO rosbag_io(FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));

    LOG(INFO) << "开始加载imu的数据" ;
    rosbag_io
        .AddImuHandle([&](IMUPtr imu) {
            imu_data_buffer.emplace_back(imu);

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

// 读取范围内的imu数据
bool GetIMUsInTimeRange(const std::vector<IMUPtr>& buffer, double start_time, double end_time, 
                        std::vector<IMUPtr>& output_imus, size_t& current_imu_idx) {
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

 void test_templocal_lo(bool & is_vis)
 {
    // 是否可视化加载的数据
    is_vis = false;
    int start_frame = 900;
    int last_frame = 200;

    std::vector<sad::CloudPtr> frames_sad;  // 使用sad::CloudPtr格式的点云容器
    std::vector<IMUPtr> imu_data_buffer;     // 加载相关的imu的数据

    // 加载所有的imu数据
    loadRosbagImuData(imu_data_buffer);

    // 进行imu的初始化
    
    LOG(INFO) << "测试里程计程序启动" ;
    // ReadandShowFrame();


    // 读取相关的点云数据
    // 现在直接加载为sad::CloudPtr格式，便于NDT使用
    PlayFrames(Frame_pcd_dir, frames_sad, start_frame, last_frame, is_vis);  // 从 900 开始加载 10 个点云
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

    std::vector<SE3> estimated_poses;
    std::deque<sad::CloudPtr> scan_wolrd_in_local_map;
    SE3 guess = SE3();

    // 创建体素化的的相关对象，优化地图
    pcl::VoxelGrid<sad::PointType> voxel_grid;
    voxel_grid.setLeafSize(leafsize, leafsize, leafsize);
    
    ndt.SetTarget(frames_sad[0]);  // 设置初始目标点云


    // 添加imu的相关配置文件
    double last_scan_time = start_frame * 10;   // imu的当前的时间戳
    size_t current_imu_idx = 0;

    // 开始遍历所有的点云
    for(int i = 1; i < frames_sad.size(); i++)
    {

        // ----------- 读取雷达数据 -----------------
        sad::CloudPtr frames_sad_voxel = sad::CloudPtr(new sad::PointCloudType);    // 体素化后的点云

        // 体素化
        std::cout << "当前点云大小: " << frames_sad[i]->size() << std::endl;
        voxel_grid.setInputCloud(frames_sad[i]);
        voxel_grid.filter(*frames_sad_voxel);
        std::cout << "体素化后点云大小: " << frames_sad_voxel->size() << std::endl;

        ndt.SetSource(frames_sad_voxel);
        // ndt.SetTarget(frames_sad[i-1]);

        // ----------- IMU数据
        // std::vector<>

        // NDT 匹配
        // 1 恒速模型配准开始ndt的配准
        if ( estimated_poses.size() < 2) {
            // 第一次迭代时，使用初始猜测
            ndt.AlignNdt(guess);
        } else {
            // 采取恒速模型预测相关的初始数值
            SE3 T1 = estimated_poses[estimated_poses.size() - 1];  // 上一帧的估计位姿
            SE3 T2 = estimated_poses[estimated_poses.size() - 2];  // 上上一帧的估计位姿
            guess = T1 * (T2.inverse() * T1 );  // 初始猜测为上一帧的逆变换
            ndt.AlignNdt(guess);
        }

        // // 1 使用直接配准的方法
        // ndt.AlignNdt(guess);

        std::cout << "NDT配准结果:\n " << guess.matrix() << std::endl;
        // 加入到估计位姿
        estimated_poses.emplace_back(guess);
        
        
        // 变换当前帧到局部地图坐标系
        // sad::CloudPtr scan_world = sad::CloudPtr(new sad::PointCloudType);      // 变换后的源点云
        scan_world.reset(new sad::PointCloudType);
        pcl::transformPointCloud(*frames_sad_voxel, *scan_world, guess.matrix().cast<float>());

        if (IsKeyframe(guess))
        {
            last_kf_pose = guess;

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

            *output_cloud += *local_map;
        }
    }
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

int main(int argc, char ** argv) {




    bool is_vis = true;
    // 启用日志系统
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    // 解析命令行参数
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "主程序启动";

    // 主要的相关的里程计测试代码
    // test_templocal_lo(is_vis);
    // test_rotate();

    // test_transfomr();

    // 测试读取rosbag的代码
    // std::deque<IMUPtr> imu_data_buffer;
    // loadRosbagImuData(imu_data_buffer);

    // 测试eskf的相关程序
    test_eskf_imu();
    LOG(INFO) << "主程序结束";



    // // 创建LIO模拟器

    // SimulationLIO lio_system;

    // std::cout << "\n开始模拟数据流..." << std::endl;

    // // 5 帧的雷达 10hz 
    // //     每个间隔内得到10帧的 imu 数据， 100hz
    // // 模拟传感器数据流（类似ROSBag播放）
    // for (int i = 0; i < 5; ++i) {
    //     double timestamp = i * 0.2; // 每0.2秒一个激光雷达帧

    //     // 创建模拟激光雷达点云数据
    //     auto cloud = std::make_shared<PointCloud>(timestamp);
    //     cloud->addPoint(1.0 + i, 2.0 + i);
    //     cloud->addPoint(1.5 + i, 2.5 + i);
    //     cloud->addPoint(2.0 + i, 3.0 + i);

    //     // 模拟在激光雷达扫描期间接收多个IMU数据
    //     for (int j = 0; j < 10; ++j) {
    //         double imu_timestamp = timestamp + j * 0.01; // IMU数据频率更高
    //         auto imu = std::make_shared<IMUData>(
    //             imu_timestamp,
    //             0.1 + j*0.01, 0.2 + j*0.01, 0.3 + j*0.01,  // 陀螺仪数据
    //             9.8, 0.1, 0.2  // 加速度计数据
    //         );

    //         // 触发IMU回调函数
    //         lio_system.IMUCallback(imu);
    //     }

    //     // 触发激光雷达回调函数
    //     std::cout << "\n--- 激光雷达帧 " << i << " 到达 ---" << std::endl;
    //     lio_system.PointCloudCallback(cloud);
    // }

    // std::cout << "\n=== 回调函数机制演示完成 ===" << std::endl;
    // std::cout << "回调函数实现的关键点:" << std::endl;
    // std::cout << "1. MessageSync类保存回调函数，在数据同步完成时调用" << std::endl;
    // std::cout << "2. 使用lambda表达式捕获this指针，调用成员函数" << std::endl;
    // std::cout << "3. 事件驱动模式：数据到达 -> 同步 -> 触发回调 -> 处理数据" << std::endl;

    return 0;
}