#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "common/eigen_types.h"
#include "common/math_utils.h"

DEFINE_int32(num_tested_points_plane, 10, "number of tested points in plane fitting");
DEFINE_int32(num_tested_points_line, 100, "number of tested points in line fitting");
DEFINE_double(noise_sigma, 0.01, "noise of generated samples");

void PlaneFittingTest();
void LineFittingTest();

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "testing plane fitting";
    PlaneFittingTest();

    LOG(INFO) << "testing line fitting";
    LineFittingTest();
}

void PlaneFittingTest() {
    // 1. 定义一个真值平面的系数向量 [a, b, c, d]，代表平面方程 ax + by + cz + d = 0
    Vec4d true_plane_coeffs(0.1, 0.2, 0.3, 0.4);
    true_plane_coeffs.normalize();

    // 2. 将该系数向量归一化（使其 L2 范数为 1）。
    //    归一化后的 [a, b, c] 是平面的单位法向量，d 是原点到平面的距离（带符号）。
    //    这有助于后续计算和比较。
    std::vector<Vec3d> points;

    // 随机生成仿真平面点
    cv::RNG rng;

    // 5. 循环生成指定数量 (FLAGS_num_tested_points_plane) 的测试点
    for (int i = 0; i < FLAGS_num_tested_points_plane; ++i) {
        // 先生成一个随机点，计算第四维，增加噪声，再归一化
        // a. 生成一个在 [0, 1) x [0, 1) x [0, 1) 立方体内的随机点 p = [x, y, z]
        Vec3d p(rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0));

        // b. 计算该点在真值平面上对应的 "齐次坐标第四维" n4。
        //    对于平面 ax + by + cz + d = 0，齐次坐标 [x, y, z, w] 满足 ax + by + cz + dw = 0。
        //    如果我们有 [x, y, z, 1]，那么 w = -(ax + by + cz) / d。
        //    这里 true_plane_coeffs.head<3>() 是 [a, b, c]，p 是 [x, y, z]。
        //    所以 n4 = -(a*x + b*y + c*z) / d。
        //    n4 = -p * plane / d
        double n4 = -p.dot(true_plane_coeffs.head<3>()) / true_plane_coeffs[3];

        // c. 将点 p 归一化到平面上。
        //    将 p 除以 n4，得到齐次坐标 [x/n4, y/n4, z/n4, 1]，这对应于欧几里得坐标 [x/n4, y/n4, z/n4]。
        //    这个点现在精确地位于由 true_plane_coeffs 定义的平面上。
        //    std::numeric_limits<double>::min() 是一个极小的正数，加在分母上是为了防止 n4 为 0 时的除零错误。
        p = p / (n4 + std::numeric_limits<double>::min());  // 防止除零

        // d. 为该点添加高斯噪声。
        //    每个坐标 (x, y, z) 都加上一个均值为 0，标准差为 FLAGS_noise_sigma 的高斯随机数。
        //    这模拟了实际传感器数据中存在的测量误差。
        p += Vec3d(rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma));

        points.emplace_back(p);

        // 验证在平面上
        LOG(INFO) << "res of p: " << p.dot(true_plane_coeffs.head<3>()) + true_plane_coeffs[3];
    }

    Vec4d estimated_plane_coeffs;
    if (sad::math::FitPlane(points, estimated_plane_coeffs)) {
        LOG(INFO) << "estimated coeffs: " << estimated_plane_coeffs.transpose()
                  << ", true: " << true_plane_coeffs.transpose();
    } else {
        LOG(INFO) << "plane fitting failed";
    }
}

void LineFittingTest() {
    // 直线拟合参数真值   点和它的向量参数
    Vec3d true_line_origin(0.1, 0.2, 0.3);
    Vec3d true_line_dir(0.4, 0.5, 0.6);
    true_line_dir.normalize();

    // 随机生成直线点，利用参数方程
    std::vector<Vec3d> points;
    cv::RNG rng;
    for (int i = 0; i < fLI::FLAGS_num_tested_points_line; ++i) {
        double t = rng.uniform(-1.0, 1.0);
        Vec3d p = true_line_origin + true_line_dir * t;
        p += Vec3d(rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma));

        points.emplace_back(p);
    }

    Vec3d esti_origin, esti_dir;
    if (sad::math::FitLine(points, esti_origin, esti_dir)) {
        LOG(INFO) << "estimated origin: " << esti_origin.transpose() << ", true: " << true_line_origin.transpose();
        LOG(INFO) << "estimated dir: " << esti_dir.transpose() << ", true: " << true_line_dir.transpose();
    } else {
        LOG(INFO) << "line fitting failed";
    }
}
