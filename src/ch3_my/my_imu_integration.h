#pragma once

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/nav_state.h"

namespace sad {
    class IMUIntegration{
        public:
        IMUIntegration(const Vec3d& gravity, const Vec3d& init_bg, const Vec3d& init_ba)
        : gravity_(gravity), bg_(init_bg), ba_(init_ba) {}


        void AddIMU(const IMU& imu) {
            // IMU的时间读取差值
            double dt = imu.timestamp_ - timestamp_;
            if (dt >0 && dt < 0.1) {
                p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt + 0.5*(R_* (imu.acce_ - ba_)) * dt *dt;
                v_ = v_ + R_ * (imu.acce_ - ba_ ) * dt + gravity_ * dt;
                R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);
            }

            timestamp_ = imu.timestamp_;
        }

        NavStated GetNavState() const {return NavStated(timestamp_, R_, p_, v_, bg_, ba_);}

        SO3 GetR() const {return R_;}
        Vec3d GetV() const {return v_;}
        Vec3d GetP() const {return p_;}

        
        private:
        SO3 R_;
        Vec3d v_ = Vec3d::Zero();
        Vec3d p_ = Vec3d::Zero();

        double timestamp_ = 0.0;

        Vec3d ba_ = Vec3d::Zero();
        Vec3d bg_ = Vec3d::Zero();

        Vec3d gravity_ = Vec3d(0,0, -9.8);

    };
}