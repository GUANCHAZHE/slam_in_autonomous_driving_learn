
#pragma once

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/odom.h"

#include <deque>

namespace sad{
/**
 * IMU水平静止状态下初始化器
 * 使用方法：调用AddIMU, AddOdom添加数据，使用InitSuccess获取初始化是否成功
 * 成功后，使用各Get函数获取内部参数
 *
 * 初始化器在每次调用AddIMU时尝试对系统进行初始化。在有odom的场合，初始化要求odom轮速读数接近零；没有时，假设车辆初期静止。
 * 初始化器收集一段时间内的IMU读数，按照书本3.5.4节估计初始零偏和噪声参数，提供给ESKF或者其他滤波器
 */

class StaticIMUInit {
    public:
    struct Options {
        Options() {}
        double init_time_seconds = 10.0;
        int init_imu_queue_max_size_ = 2000;
        int static_odom_pulse_ = 5;
        double max_static_gyro_var_ = 0.5;
        double max_static_acce_var_ = 0.05;
        double gravity_norm_ = 9.81;
        bool use_speed_for_static_chaecking_ = true;
    };

    // 构造函数
    StaticIMUInit(Options options = Options()) : options_(options) {}

    // 添加IMU 数据
    bool AddIMU(const IMU& imu);
    //添加轮数据
    bool AddOdom(const Odom& odom);

    // 判断是否成功初始化
    bool InitSuccess() const {return init_success_;}

    // 获取各个Cov, bias, gravity
    Vec3d GetCovGyro() const {return cov_gyro_; }
    Vec3d GetCovAcce() const {return cov_acce_; }
    Vec3d GetInitBg() const {return init_bg_; }
    Vec3d GetInitBa() const {return init_ba_; }
    Vec3d GetGravity() const {return gravity_; }


    private:
    // 尝试对系统初始化
    bool TryInit();

    Options options_;                 // 选项信息
    bool init_success_ = false;       // 初始化是否成功
    Vec3d cov_gyro_ = Vec3d::Zero();  // 陀螺测量噪声协方差（初始化时评估）
    Vec3d cov_acce_ = Vec3d::Zero();  // 加计测量噪声协方差（初始化时评估）
    Vec3d init_bg_ = Vec3d::Zero();   // 陀螺初始零偏
    Vec3d init_ba_ = Vec3d::Zero();   // 加计初始零偏
    Vec3d gravity_ = Vec3d::Zero();   // 重力  由于不确定重力初始方向，选择三维重力方向
    bool is_static_ = false;          // 标志车辆是否静止
    std::deque<IMU> init_imu_deque_;  // 初始化用的数据
    double current_time_ = 0.0;       // 当前时间
    double init_start_time_ = 0.0;    // 静止的初始时间
};
}