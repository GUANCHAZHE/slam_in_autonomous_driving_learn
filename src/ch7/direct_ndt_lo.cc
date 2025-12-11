//
// Created by xiang on 2022/7/18.
//

#include "ch7/direct_ndt_lo.h"
#include "common/math_utils.h"
#include "tools/pcl_map_viewer.h"

#include <pcl/common/transforms.h>

namespace sad {

void DirectNDTLO::AddCloud(CloudPtr scan, SE3& pose) {
    // 第一帧添加到局部地图，然后开始建立体素
    if (local_map_ == nullptr) {
        // 第一个帧，直接加入local map
        local_map_.reset(new PointCloudType);
        // operator += 用来拼接点云
        *local_map_ += *scan;
        pose = SE3();
        last_kf_pose_ = pose;

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.setInputTarget(local_map_);
        } else {
            ndt_.SetTarget(local_map_);  // 建立体素
        }

        return;
    }

    // 计算scan相对于local map的位姿
    // 这里的得到的是相对于起始点的位姿，也就是世界坐标系下的位置
    pose = AlignWithLocalMap(scan);
    
    // 显示当前的估计的pose
    Eigen::Vector3d t = pose.translation();
    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d euler = R.eulerAngles(0, 1, 2);
    std::cout << "--------- SE3f/SE3d 打印 --------- \n";
    std::cout << "平移 t = " << t.transpose() << "\n";
    std::cout << "旋转矩阵 R = \n" << R << "\n";
    std::cout << "欧拉角(rad) roll-pitch-yaw = " << euler.transpose() << "\n";
    std::cout << "欧拉角(deg)                = " << (euler * 180 / M_PI).transpose() << "\n";


    CloudPtr scan_world(new PointCloudType);
    // 利用和全局地图配准得到的pose，将scan转换到scan_world，也就是世界坐标系下面去
    pcl::transformPointCloud(*scan, *scan_world, pose.matrix().cast<float>());

    // 只选择相关的关键帧拼接为局部地图
    if (IsKeyframe(pose)) {
        std::cout << "22关键帧,加入局部地图" << std::endl;
        // 显示当前的估计的pose
        Eigen::Vector3d t = pose.translation();
        Eigen::Matrix3d R = pose.rotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(0, 1, 2);
        std::cout << "22--------- SE3f/SE3d 打印 --------- \n";
        std::cout << "22平移 t = " << t.transpose() << "\n";
        std::cout << "22旋转矩阵 R = \n" << R << "\n";
        std::cout << "22欧拉角(rad) roll-pitch-yaw = " << euler.transpose() << "\n";
        std::cout << "22欧拉角(deg)                = " << (euler * 180 / M_PI).transpose() << "\n";

        last_kf_pose_ = pose;

        // 重建local map
        scans_in_local_map_.emplace_back(scan_world);
        if (scans_in_local_map_.size() > options_.num_kfs_in_local_map_) {
            scans_in_local_map_.pop_front();
        }
        std::cout << "22局部地图大小: " << local_map_->size() << std::endl;
        local_map_.reset(new PointCloudType);
        for (auto& scan : scans_in_local_map_) {
            *local_map_ += *scan;
        }

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.setInputTarget(local_map_);
        } else {
            ndt_.SetTarget(local_map_);
        }
    }

    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
}

bool DirectNDTLO::IsKeyframe(const SE3& current_pose) {
    // 只要与上一帧相对运动超过一定距离或角度，就记关键帧
    // 假设当前的位置P1w P2w，从1移动到2的变换为 T21
    // T21 * P1w = P2w  位置的增量也就是状态的变换
    // T21 = P2w * P1w^-1
    // T12^-1 = T12^T = P1w^-1 * P2w 得到如下结果
    //  其实反向也没有太大的问题，我们需要的模长和角度都是相同的
    SE3 delta = last_kf_pose_.inverse() * current_pose;

    bool distance_ok = delta.translation().norm() > options_.kf_distance_;
    bool angle_ok = delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD; 

    // 打印出当前近的距离和角度
    // 将他的矩阵形式的平移和旋转打印出来
    Vec3d t = delta.translation();
    Mat3d R = delta.rotationMatrix();
    std::cout << "当前的变换矩阵 delta.matrix() = \n" << delta.matrix() << "\n";
    std::cout << "当前的平移增量 t = " << t.transpose() << "\n";
    std::cout << "当前的旋转增量 R = \n" << R << "\n";

    std::cout << "当前的距离增量:delta.translation().norm() " << delta.translation().norm() << std::endl;
    std::cout << "当前的角度增量:delta.so3().log().norm() " << delta.so3().log().norm() << std::endl;

    return distance_ok || angle_ok;
}

SE3 DirectNDTLO::AlignWithLocalMap(CloudPtr scan) {
    if (options_.use_pcl_ndt_) {
        ndt_pcl_.setInputSource(scan);
    } else {
        ndt_.SetSource(scan);
    }

    CloudPtr output(new PointCloudType());

    SE3 guess;
    bool align_success = true;
    // 前两帧之间的位置
    if (estimated_poses_.size() < 2) {
        if (options_.use_pcl_ndt_) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);
        }
    } else {
        // 从最近两个pose来推断
        // 利用恒速模型估计现在这个时刻的状态，将估计得到的状态传递给ndt，作为初值
        // T2 ----> T1 ----> T3?
        SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
        SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
        // 位移增量 Δ = T2^{-1} T1
        // 按恒速预测 T3 = T1 * Δ   将Δ视为点的增量 并且Δ是机器人坐标系下的增量
        guess = T1 * (T2.inverse() * T1);  

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);   // 将target和source匹配，初值为guess
        }
    }

    // pose: [x y z], [qx qy qz qw]
    LOG(INFO) << "pose: " << guess.translation().transpose() << ", "    // 平移的xyz
              << guess.so3().unit_quaternion().coeffs().transpose();    
              // so3()旋转李代数
              // unit_quaternion()然后转为单位四元数，
              // coeffs()之后再获取，
              // transpose()最后转置输出

    if (options_.use_pcl_ndt_) {
        LOG(INFO) << "trans prob: " << ndt_pcl_.getTransformationProbability();
    }

    estimated_poses_.emplace_back(guess);
    return guess;
}

void DirectNDTLO::SaveMap(const std::string& map_path) {
    if (viewer_) {
        viewer_->SaveMap(map_path);
    }
}

}  // namespace sad