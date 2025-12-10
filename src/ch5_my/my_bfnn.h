
#ifndef SLAM_IN_AUTO_DRIVING_BFNN_H
#define SLAM_IN_AUTO_DRIVING_BFNN_H


#include "common/eigen_types.h"
#include "common/point_types.h"

#include <thread>

namespace sad{
    /**
     * BF NN
     * @param cloud 点云
     * @param point 待查找点
     * @return 找到最近点索引
     */
    int bfnn_point(CloudPtr cloud, const Vec3f& point);
     
    /**
     * k近邻
     * @param cloud 点云
     * @param point 待查找的带你
     * @param k 近邻数
     * @return 找到的最近邻的索引
     */
    std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point ,int k = 5);
    /**
     * 对点云进行BF最近邻 单线程版本
     * @param cloud1
     * @param cloud2
     * @param matches
     */
    void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>> matches);

    /**
     * 对点云进行BF最近邻 多线程版本
     * @param cloud1
     * @param cloud2
     * @param matches
     */
    void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches);

}

#endif