//
// Created by xiang on 23-2-2.
//
#include "measure_sync.h"

namespace sad {

// 它以激光雷达扫描的开始和结束时间作为参考时间窗口。
// 它收集所有时间戳落在该窗口内的IMU数据。
// 它等待直到有足够的IMU数据覆盖了整个激光雷达扫描周期。
// 最终输出一个包含单帧激光雷达数据和对应时间段内所有IMU数据的同步数据包。
// 通过不断处理缓冲区中的数据，实现了连续的数据流同步。
bool MessageSync::Sync() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        lidar_end_time_ = measures_.lidar_begin_time_ + measures_.lidar_->points.back().time / double(1000);

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    double imu_time = imu_buffer_.front()->timestamp_;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp_;
        if (imu_time > lidar_end_time_) {
            break;
        }
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    if (callback_) {
        callback_(measures_);
    }

    return true;
}

void MessageSync::Init(const std::string& yaml) { conv_->LoadFromYAML(yaml); }

}  // namespace sad