//
// Created by xiang on 22-12-29.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 本节程序演示一个正在作圆周运动的车辆
/// 车辆的角速度与线速度可以在flags中设置

DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    SE3 pose;                                                                    // TWB表示的位姿 也就是 齐次的[R|t;0 1]
    Vec3d omega(0, angular_velocity_rad, 0);                                     // 角速度矢量  围绕这自身的z轴旋转
    // Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    Vec3d v_body(0, 0, 0);                                                      // 本体系速度  x向右为正 Y向上为正 Z向前为正 右手坐标系
    const double dt = 0.05;                                                      // 每次更新的时间
    Vec3d a_world(0, -9.8, 0);  // 世界坐标系下的加速度
    
    while (ui.ShouldQuit() == false) {
        // 更新自身位置
        v_body = v_body + a_world * dt;           // 在本体系下加速度更新
        Vec3d v_world = pose.so3() * v_body;      // 在自身坐标系下速度更新
        pose.translation() += (v_world ) * dt;    // 将这个更新转换到世界坐标系下

        //// 更新自身旋转
        if (FLAGS_use_quaternion) {
            // 对应的 公式 -2.78
            // 四元数更新qExp[w] = q[1, 1/2 w] = q[1, 0.5wx, 0.5wy, 0.5wz] * dt 
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();  // 归一化四元数 四元数是单位四元数 多次计算完完成之后可能不是单位四元数，所以需要归一化
            pose.so3() = SO3(q);  // 将四元数转换为SO3
        } else {
            // 公式2.55 旋转矩阵R(t) = R(t-1) * exp(omega * dt)
            // 将自身坐标系下的旋转（omega）转换到全局坐标系
            Vec3d omega_world = pose.so3() * omega;  // 变换到世界坐标系
            pose.so3() = pose.so3() * SO3::exp(omega_world * dt);
            
            // pose.so3() = pose.so3() * SO3::exp(omega * dt);  // 更新自身的旋转代码 原始代码
        }

        LOG(INFO) << "pose:T " << pose.translation().transpose();
        // 输出欧拉角（ZYX顺序：yaw, pitch, roll）
        Eigen::Vector3d euler_angles = pose.so3().matrix().eulerAngles(2, 1, 0); // ZYX顺序
        // LOG(INFO) << "pose:R " << pose.rotationMatrix();
        LOG(INFO) << "pose:euler (yaw, pitch, roll): " << euler_angles.transpose();
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);
    }

    ui.Quit();
    return 0;
}