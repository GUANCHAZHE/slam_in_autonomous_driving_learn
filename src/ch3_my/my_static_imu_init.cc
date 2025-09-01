#include "ch3_my/my_static_imu_init.h"
#include "common/math_utils.h"

#include <glog/logging.h>
namespace sad {

bool StaticIMUInit::AddIMU(const IMU& imu) {
    if (init_success_){
        return true;
    }

    if (options_.use_speed_for_static_chaecking_ && !is_static_) {
        LOG(WARNING) << "等待车辆静止";
        init_imu_deque_.clear();
        return false;
    }

    if (init_imu_deque_.empty()) {
        // 记录开始静止的时间
        init_start_time_ = imu.timestamp_;
    }

    // 计入初始化队列
    init_imu_deque_.push_back(imu);

    double init_time = imu.timestamp_ - init_start_time_; // 记录初始化的时间
    if (init_time > options_.init_time_seconds){
        TryInit();
    }

    while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
        init_imu_deque_.pop_front();
    }

    current_time_ = imu.timestamp_;
    return false;
}

bool StaticIMUInit::AddOdom(const Odom& odom) {
    if(init_success_) {
        return true;
    }

    if (odom.left_pulse_ < options_.static_odom_pulse_ && odom.right_pulse_ < options_.static_odom_pulse_) {
        is_static_ = true;
    } else {
        is_static_ = false;
    }

    current_time_ = odom.timestamp_;
    return true;
}

// 
bool StaticIMUInit::TryInit() {
    if (init_imu_deque_.size() < 10 ) {
        return false;
    }

    Vec3d mean_gryo, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gryo, cov_gyro_, [](const IMU& imu) {return imu.gyro_;});
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [](const IMU& imu) {return imu.acce_;});

    // 以acc均值为方向，取9.8长度为重力 ？？ 这部分的代码是为啥？
    LOG(INFO) <<"meadn acce: " << mean_acce.transpose();  // mean_acce的转置 也就是[0,0,-9.8]
    // 这时候 mean_acce.norm() 就是求解范数 平方和开根号 的结果就是 9.8
    // 由于不确定重力一开始哪个方向，所以选择将三个方向都考虑进去(x,y,z)
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_; 


    // 重新计算加速度计的协方差  去除重力影响 
    // 这里需要this，是因为访问了类内变量gravity_ 所以需要this 指针
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                        [this](const IMU& imu) { return imu.acce_ + gravity_;});
    
    
    // 检查IMU噪声
    // 这个cov_gyro的数据到底是什么样的，是多少？
    // xyz方向的误差求和，他只求解了主对角线的影响
    if (cov_gyro_.norm() > options_.max_static_gyro_var_) {
        LOG(ERROR) << "陀螺仪测量噪声太大" <<cov_gyro_.norm() << " > " << options_.max_static_gyro_var_;
        return false;
    }
    
    if (cov_acce_.norm() > options_.max_static_acce_var_) {
        LOG(ERROR) << "加速度计测量噪声太大" <<cov_acce_.norm() << " > " << options_.max_static_acce_var_;
        return false;
    }

    // 估计测量噪声和零偏
    init_bg_ = mean_gryo;
    init_ba_ = mean_acce;

    LOG(INFO) << "IMU 初始化成功， 初始化时间=" << current_time_ - init_start_time_ <<", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose() <<", gyro sq = " << cov_gyro_.transpose()
              << ", acce sq = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
              << ", norm: " << gravity_.norm();
    LOG(INFO) << "mean gyro: " << mean_gryo.transpose() << " acce: " << mean_acce.transpose();
    init_success_ = true;
    return true;    
}

}
