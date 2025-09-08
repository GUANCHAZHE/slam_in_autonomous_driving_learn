#pragma once

#include "common/eigen_types.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/nav_state.h"
#include "common/odom.h"

#include <glog/logging.h>
#include <iomanip>

namespace sad {
/**
 * 书本第3章介绍的误差卡尔曼滤波器
 * 可以指定观测GNSS的读数，GNSS应该事先转换到车体坐标系
 *
 * 本书使用18维的ESKF，标量类型可以由S指定，默认取double
 * 变量顺序：p, v, R, bg, ba, grav，与书本对应x
 * @tparam S    状态变量的精度，取float或double
 */

template <typename S = double>
class ESKF {
    public:
    // 类型定义
    using SO3 = Sophus::SO3<S>;                    // 旋转变量类型 李群表示
    using VecT = Eigen::Matrix<S, 3, 1>;           // 向量向量
    using Vec18T = Eigen::Matrix<S, 18, 1>;        // 18维向量类型 3*6 对应x
    using Mat3T = Eigen::Matrix<S, 3, 3>;          // 3*3矩阵了类型
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>; // 运动噪声类型
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;     // 里程计噪声类型 ?
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;     // GNSS噪声类型
    using Mat18T = Eigen::Matrix<S, 18,18>;        // 18维方差类型  ?
    using NavStateT = NavState<S>;                 // 整体名义状态变量类型


    struct Options {
        Options() = default;

        /// IMU 测量与零偏参数
        double imu_dt_ = 0.01;         // IMU测量间隔  100hz
        // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
        double gyro_var_ = 1e-5;       // 陀螺测量方差     σg   p53
        double acce_var_ = 1e-2;       // 加计测量方差     σa   p53
        double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差  σbg  p54
        double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差  σba  p54

        /// 里程计参数 里程计
        double odom_var_ = 0.5;        // 轮速计的噪声误差 z = h(x) + V
        double odom_span_ = 0.1;       // 里程计测量间隔

        // 轮式编码器
        double wheel_radius_ = 0.155;  // 轮子半径   单位是米
        double circle_pulse_ = 1024.0; // 编码器每圈脉冲数

        /// RTK 观测参数
        double gnss_pos_noise_ = 0.1;                   // GNSS位置噪声
        double gnss_height_noise_ = 0.1;                // GNSS高度噪声
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS旋转噪声

        /// 其他配置
        bool update_bias_gyro_ = true;  // 是否更新陀螺bias
        bool update_bias_acce_ = true;  // 是否更新加计bias
    };

    ESKF(Options option = Options()) : options_(option) {BuildNoise(option); }

    /**
     * 设置初始条件
     * @param options 噪声项配置
     * @param init_bg 初始零偏 陀螺
     * @param init_ba 初始零偏 加计
     * @param gravity 重力
     */
    void SetInitialConditions( Options options, const VecT& init_bg, const VecT& init_ba,
                                const VecT& gravity = VecT(0, 0, -9.8)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
        cov_ = Mat18T::Identity() * 1e-4;   // ？？ 这里为什么还需要 * 1e-4？？？
    }

    // 运动方程和观测方程
    // 这部分的代码放在整体类的外部实现
    /// 使用IMU递推
    bool Predict(const IMU& imu);

    /// 使用轮速计观测
    bool ObserveWheelSpeed(const Odom& odom);

    /// 使用GPS观测
    bool ObserveGps(const GNSS& gnss);

    /**
     * 使用SE3进行观测
     * @param pose  观测位姿
     * @param trans_noise 平移噪声
     * @param ang_noise   角度噪声
     * @return
     */
    bool ObserveSE3(const SE3& pose, double trans_noise = 0.1, double ang_noise = 1.0 * math::kDEG2RAD);

    /// accessors
    /// 获取全量状态
    NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

    /// 获取重力
    Vec3d GetGravity() const {return g_; }

    private:
    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev;  // * ev;
        double et2 = et;  // * et;
        double eg2 = eg;  // * eg;
        double ea2 = ea;  // * ea;

        // 设置过程噪声
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;
        // diagonal 获取对角线元素

        // 设置里程计噪声
        double o2 = options.odom_var_ * options.odom_var_;    //  ? 这部分还是需要再看一下， 他是如何进行 噪声相乘的，方差和标准差之间的关系
        odom_noise_.diagonal() << o2, o2, o2;

        // 设置GNSS状态
        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;   // 注意这里的z是高度误差，和xy不一样
    }

    /// 更新名义状态变量，重置error state  公式3.55 和公式
    void UpdateAndReset() {
        // 1 带入误差数值  （3.55）
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (options_.update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(9, 0);
        }

        if (options_.update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }

        g_ += dx_.template block<3, 1>(15, 0);

        // 2 重置ESKF 分为重置均值和协方差部分
        ProjectCov();   // (3.63) 协方差矩阵线性变化
        dx_.setZero();  // 误差置为0  （3.57）
    }

    /// 对P阵进行投影，参考式(3.63)
    void ProjectCov() {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0)); // (3.62)
        cov_ = J * cov_ * J.transpose();  // (3.63)
    }

        // 成员变量
    double current_time_ = 0.0;  // 当前时间

    /// 名义状态 就是测量数据
    VecT p_ = VecT::Zero();
    VecT v_ = VecT::Zero();
    SO3  R_;
    VecT bg_ = VecT::Zero();
    VecT ba_ = VecT::Zero();
    VecT g_{0, 0, -9.8};

    /// 误差状态 
    Vec18T dx_ = Vec18T::Zero();    //  公式3.43  18*1

    /// 协方差阵
    Mat18T cov_ = Mat18T::Identity();    //  公式3.48b 中的P 18*1

    /// 噪声阵
    MotionNoiseT Q_ = MotionNoiseT::Zero();     //  公式3.45  18*18
    OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

    /// 标志位
    bool first_gnss_ = true;  // 是否为第一个gnss数据

    /// 配置项
    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    assert(imu.timestamp_ >= current_time_);

    double dt = imu.timestamp_ - current_time_;
    if (dt > (5 * options_.imu_dt_) || dt < 0) {
        // 时间间隔不对，可能是第一个IMU数据，没有历史信息
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }

    // nominal state 递推
    // 这部分每次运行都会根据之前的数据生成一次，不会保留之前的数据，所以需要更新状态
    // 3-41a p(t+1) = p(t) + v*dt + 1/2(R(a~ - ba))* dt^2 + 1/2*g*dt^2 
    VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    // 3-41b v(t+1) = v(t) + R(t)(a~ - ba) * dt + g * dt
    VecT new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
    // 3-41c R(t+1) = R(t) * Exp(w~ - bg) * dt
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    // 更新预测状态 xpred  
    // 每次状态都不会创建一个新的，所以需要保留之前的数据 ？？？
    // TODO 这里的时间切换还不是很懂，在代码里如何实现的，只明白一个大概，具体的实现还需要再看看
    R_ = new_R;
    v_ = new_v;
    p_ = new_p;
    // 其余状态维度不变

    // error state 递推
    // 计算运动过程雅可比矩阵 F，见(3.47)
    // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式，这里为了教学方便，使用矩阵形式
    // 它的相关的()内的坐标采取的是开始的位置，然后往后计算三个
    // | 子块                            | 对应    | 物理意义                      |
    // | ------------------------------ | ------  | ----------------------      |
    // | F(0:3,3:6) = I\*dt             | p 对 v  | 位置随速度变化（p = p + v\*dt） |
    // | F(3:6,6:9) = -R\*hat(a-ba)\*dt | v 对 θ  | 速度受姿态旋转影响              |
    // | F(3:6,12:15) = -R\*dt          | v 对 ba | 加速度计偏置影响速度积分         |
    // | F(3:6,15:18) = I\*dt           | v 对 g  | 重力影响速度积分               |
    // | F(6:9,6:9) = SO3::exp(...)     | θ 对 θ  | 姿态随角速度更新               |
    // | F(6:9,9:12) = -I\*dt           | θ 对 bg | 陀螺仪偏置对姿态影响            |

    Mat18T F = Mat18T::Identity();                                                 // 主对角线
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                         // p 对 v               [1,2]
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;  // v对theta (R)         [2,3]
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                             // v 对 ba              [2,5]
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                        // v 对 g               [2,6]
    F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();     // theta(R) 对 theta(R) [3,3]
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                        // theta(R) 对 bg       [3,4]

    // mean and cov prediction
    dx_ = F * dx_;  // 公式对应的 3.48a  这行其实没必要算，dx_在重置之后应该为零，因此这步可以跳过，但F需要参与Cov部分计算，所以保留
    cov_ = F * cov_.eval() * F.transpose() + Q_;    // 公式对应的 3.48b  cov_ = Ppred 估计协方差矩阵
    // std::cout << cov_ << std::endl;
    // std::cout << Q_ << std::endl;
    current_time_ = imu.timestamp_;   // 更新时间
    return true;
}

template <typename S>
bool ESKF<S>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);
    // odom 修正以及雅可比
    // 使用三维的轮速观测，H为3*18, 大部分为零，其中3为v速度
    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    H.template block<3, 3>(0, 3) = Mat3T::Identity();

    // 卡尔曼增益
    //  cov_ = Ppred (3.51a)
    Eigen::Matrix<S, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse(); 

    // velocity obs  （3.76）
    // odom.left_pulse_ / options_.circle_pulse_             p/n 就是弧度
    // odom.left_pulse_ / options_.circle_pulse_ * 2 *M_PI   (p/n) *2π 角度
    // odom.left_pulse_ / options_.circle_pulse_ * 2 *M_PI / options_.odom_span_ (p/n) *2π /t 角速度w
    // odom.left_pulse_ / options_.circle_pulse_ * 2 *M_PI / options_.odom_span_ * options_.wheel_radius_   (p/n)*2π /t * r v=wr 线速度   
    double velo_l = options_.wheel_radius_ * (odom.left_pulse_ / options_.circle_pulse_) * 2 * M_PI / options_.odom_span_;
    double velo_r = options_.wheel_radius_ * (odom.right_pulse_ / options_.circle_pulse_) * 2 * M_PI / options_.odom_span_;
    double average_vel = 0.5 * (velo_l + velo_r);

    VecT vel_odom(average_vel, 0.0, 0.0);  // (vx, vy, vz)
    VecT vel_world = R_ * vel_odom;       // 世界坐标下的轮速观测  3.73

    dx_ = K * (vel_world - v_);    // (3.51b) 更新误差
    
    cov_ = (Mat18T::Identity() - K * H) * cov_;   // 更新方差  (3.51d)

    UpdateAndReset();   // 误差状态后处理 (3.51c)
    return true;
}

// GNSS 观测修正
template <typename S>
bool ESKF<S>::ObserveGps(const GNSS& gnss) {
    // GNSS 观测修正 确保读取最新的数据
    assert(gnss.unix_time_ >= current_time_);
    
    // 这个记录初始的R，P？？？有啥用 标牌世界远点么？
    if (first_gnss_) {
        R_ = gnss.utm_pose_.so3();
        p_ = gnss.utm_pose_.translation();
        first_gnss_ = false;
        current_time_ = gnss.unix_time_;
        return true;
    }

    assert(gnss.heading_valid_);
    ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise_, options_.gnss_ang_noise_);
    current_time_ = gnss.unix_time_;

    return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
    /// se3 pose既有旋转，也有平移
    /// 观测状态变量中的p, R，H为6x18，其余为零，定义如下 p,v,r,bg,ba,g,他的协方差矩阵就是18*18的方阵，每行列分别求导
    /// 他的大致形状为横向长条 
    /// |I3,03,03,03,03,03|
    /// |03,I3,03,03,03,03|
    ///   
    ///
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();  // P部分（3.70)
    H.template block<3, 3>(3, 6) = Mat3T::Identity();  // R部分（3.66)

    // 卡尔曼增益和更新过程
    Vec6d noise_vec;       // 噪声部分的V z = h(x) + v, v~(0,V)
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;
    Mat6d V = noise_vec.asDiagonal();    // 将行向量转换成一个对角阵

    Eigen::Matrix<S, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();  // (3.51a) 卡尔曼增益K

    // 更新x和cov
    Vec6d innov = Vec6d::Zero();                                  
    innov.template head<3>() = (pose.translation() - p_);          // 平移部分  pgnss - p
    innov.template tail<3>() = (R_.inverse() * pose.so3()).log();  // 旋转部分(3.67) LOG(R^T*Rgnss)

    dx_ = K * innov;   // (3.51b) 更新误差 
    cov_ = (Mat18T::Identity() - K * H) * cov_; // (3.51d) 更新协方差矩阵

    UpdateAndReset();    // 将误差叠加当前数值 && 重置误差
    return true;
}

}