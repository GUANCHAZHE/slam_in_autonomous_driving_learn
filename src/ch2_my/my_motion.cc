#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char ** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if(ui.Init() == false) {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;
    SE3 pose;
    Vec3d omega(0, angular_velocity_rad, 0);  // 角速度矢量 围绕这自身的z轴旋转

    Vec3d v_body(0,0,0);
    const double dt = 0.05;
    Vec3d a_world(0, -0.98,0);

    while(ui.ShouldQuit() == false) {
        v_body = v_body +a_world * dt;
        Vec3d v_world = pose.so3() * v_body;  // 在自身坐标系下速度更新
        pose.translation() += (v_world) * dt;  // 将这个更新转换到

        Vec3d omeg_world = pose.so3() * omega;  // 变换到世界坐标系
        pose.so3() = pose.so3() * SO3::exp(omeg_world * dt);


        LOG(INFO) << "pose:T " << pose.translation().transpose();

        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);  // 微秒级别的延时
    }

    ui.Quit();
    return 0;
}