#include <iostream>
#include <functional> // for std::function
#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <algorithm>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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


void ReadAndShowFrame()
{
    
}

int main() {




    std::cout << "=== 模拟LIO系统的回调函数机制 ===" << std::endl;

    // 创建LIO模拟器
    SimulationLIO lio_system;

    std::cout << "\n开始模拟数据流..." << std::endl;

    // 5 帧的雷达 10hz 
    //     每个间隔内得到10帧的 imu 数据， 100hz
    // 模拟传感器数据流（类似ROSBag播放）
    for (int i = 0; i < 5; ++i) {
        double timestamp = i * 0.2; // 每0.2秒一个激光雷达帧

        // 创建模拟激光雷达点云数据
        auto cloud = std::make_shared<PointCloud>(timestamp);
        cloud->addPoint(1.0 + i, 2.0 + i);
        cloud->addPoint(1.5 + i, 2.5 + i);
        cloud->addPoint(2.0 + i, 3.0 + i);

        // 模拟在激光雷达扫描期间接收多个IMU数据
        for (int j = 0; j < 10; ++j) {
            double imu_timestamp = timestamp + j * 0.01; // IMU数据频率更高
            auto imu = std::make_shared<IMUData>(
                imu_timestamp,
                0.1 + j*0.01, 0.2 + j*0.01, 0.3 + j*0.01,  // 陀螺仪数据
                9.8, 0.1, 0.2  // 加速度计数据
            );

            // 触发IMU回调函数
            lio_system.IMUCallback(imu);
        }

        // 触发激光雷达回调函数
        std::cout << "\n--- 激光雷达帧 " << i << " 到达 ---" << std::endl;
        lio_system.PointCloudCallback(cloud);
    }

    std::cout << "\n=== 回调函数机制演示完成 ===" << std::endl;
    std::cout << "回调函数实现的关键点:" << std::endl;
    std::cout << "1. MessageSync类保存回调函数，在数据同步完成时调用" << std::endl;
    std::cout << "2. 使用lambda表达式捕获this指针，调用成员函数" << std::endl;
    std::cout << "3. 事件驱动模式：数据到达 -> 同步 -> 触发回调 -> 处理数据" << std::endl;

    return 0;
}