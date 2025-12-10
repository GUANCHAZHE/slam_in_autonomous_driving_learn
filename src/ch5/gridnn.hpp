//
// Created by xiang on 2021/8/25.
//

#ifndef SLAM_IN_AUTO_DRIVING_GRID2D_HPP
#define SLAM_IN_AUTO_DRIVING_GRID2D_HPP

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "common/point_types.h"

#include <glog/logging.h>
#include <execution>
#include <map>

namespace sad {

/**
 * 栅格法最近邻
 * @tparam dim 模板参数，使用2D或3D栅格
 */
template <int dim>
class GridNN {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;

    enum class NearbyType {
        CENTER,  // 只考虑中心
        // for 2D
        NEARBY4,  // 上下左右
        NEARBY8,  // 上下左右+四角

        // for 3D
        NEARBY6,  // 上下左右前后
    };

    /**
     * 构造函数
     * @param resolution 分辨率
     * @param nearby_type 近邻判定方法
     */
    explicit GridNN(float resolution = 0.1, NearbyType nearby_type = NearbyType::NEARBY4)
        : resolution_(resolution), nearby_type_(nearby_type) {
        inv_resolution_ = 1.0 / resolution_;

        // check dim and nearby
        if (dim == 2 && nearby_type_ == NearbyType::NEARBY6) {
            LOG(INFO) << "2D grid does not support nearby6, using nearby4 instead.";
            nearby_type_ = NearbyType::NEARBY4;
        } else if (dim == 3 && (nearby_type_ != NearbyType::NEARBY6 && nearby_type_ != NearbyType::CENTER)) {
            LOG(INFO) << "3D grid does not support nearby4/8, using nearby6 instead.";
            nearby_type_ = NearbyType::NEARBY6;
        }

        GenerateNearbyGrids();
    }

    /// 设置点云，建立栅格
    bool SetPointCloud(CloudPtr cloud);

    /// 获取最近邻
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx);

    /// 对比两个点云
    bool GetClosestPointForCloud(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);
    bool GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);

   private:
    /// 根据最近邻的类型，生成附近网格
    void GenerateNearbyGrids();

    /// 空间坐标转到grid
    KeyType Pos2Grid(const PtType& pt);

    float resolution_ = 0.1;       // 分辨率
    float inv_resolution_ = 10.0;  // 分辨率倒数

    NearbyType nearby_type_ = NearbyType::NEARBY4;
    std::unordered_map<KeyType, std::vector<size_t>, hash_vec<dim>> grids_;  //  栅格数据
    CloudPtr cloud_;

    std::vector<KeyType> nearby_grids_;  // 附近的栅格
};

// 实现
template <int dim>
// 将空间中相邻的点都放入到一个栅格grid中
bool GridNN<dim>::SetPointCloud(CloudPtr cloud) {
    // 建立所有点云的索引，防止重复
    // [0, 1, 2, ..., N-1]
    std::vector<size_t> index(cloud->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    // 在建立的索引中进行网格化
    std::for_each(index.begin(), index.end(), [&cloud, this](const size_t& idx) {
        auto pt = cloud->points[idx];
        auto key = Pos2Grid(ToEigen<float, dim>(pt));
        // 如果这个栅格内没有点，构建栅格，插入点
        if (grids_.find(key) == grids_.end()) {
            grids_.insert({key, {idx}});
        } else {
        // 栅格内存在点，将这个点索引插入栅格
            grids_[key].emplace_back(idx);
        }
    });
    
    // 最终结果假设如下：
    // 假设点云 8 个点，被分成 3 个格子：
    // grids_ = {
    // (0, 0, 1): [0, 2, 5],
    // (1, 3, 0): [1, 3],
    // (-1, 7, 2): [4, 6, 7]
    // }
    cloud_ = cloud;
    LOG(INFO) << "grids: " << grids_.size();
    return true;
}

template <int dim>
// 输入相关的点，经过四舍五入和转换为int类型的数值 返回得到grid
Eigen::Matrix<int, dim, 1> GridNN<dim>::Pos2Grid(const Eigen::Matrix<float, dim, 1>& pt) {
    return pt.array().template round().template cast<int>();
    // Eigen::Matrix<int, dim, 1> ret;
    // for (int i = 0; i < dim; ++i) {
    //     ret(i, 0) = round(pt[i] * inv_resolution_);
    // }
    // return ret;
}

template <>
void GridNN<2>::GenerateNearbyGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY4) {
        nearby_grids_ = {Vec2i(0, 0), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1)};
    } else if (nearby_type_ == NearbyType::NEARBY8) {
        nearby_grids_ = {
            Vec2i(0, 0),   Vec2i(-1, 0), Vec2i(1, 0),  Vec2i(0, 1), Vec2i(0, -1),
            Vec2i(-1, -1), Vec2i(-1, 1), Vec2i(1, -1), Vec2i(1, 1),
        };
    }
}

template <>
void GridNN<3>::GenerateNearbyGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

template <int dim>
// 1. 查找目标点的最近栅格
// 2. 根据最近邻的定义，查找附近的栅格
// 3. 暴力匹配这些栅格的点
bool GridNN<dim>::GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx) {
    // 在pt栅格周边寻找最近邻
    std::vector<size_t> idx_to_check;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));

    // 按照之前设置的几个近邻搜索，NEARBY4/6/8
    // 2d的 4个 上下左右
    // 3d的 6个 上下左右前后
    // 这个 delta到底是如何确定的，同时的并行化搜索不会出现问题么？
    // 并行化搜索四个方向
    // 1. 通过栅格化搜索缩小范围，找到最近的栅格范围内
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &idx_to_check, this](const KeyType& delta) {
        auto dkey = key + delta;
        // 这里搜索得到最近邻的所有点，也就是搜索到的栅格的所有点
        auto iter = grids_.find(dkey);    
        if (iter != grids_.end()) {
            // 将iter的所有变量全都加到idx_to_check的末尾
            idx_to_check.insert(idx_to_check.end(), iter->second.begin(), iter->second.end());
        }
    });

    if (idx_to_check.empty()) {
        return false;
    }

    // 2. 通过反索引得到最近栅格对应的点云数据，标记新点云的索引
    // brute force nn in cloud_[idx]
    CloudPtr nearby_cloud(new PointCloudType);
    std::vector<size_t> nearby_idx;
    for (auto& idx : idx_to_check) {
        // 这是在模版编程，需要显示指明point的类型，然后再emplace_back，这是语法提示符
        nearby_cloud->points.template emplace_back(cloud_->points[idx]);  // 添加最近的点
        nearby_idx.emplace_back(idx);                                     // 添加最近的点的索引
    }

    // 3. 跟新点云的索引，bf匹配新点云团
    size_t closest_point_idx = bfnn_point(nearby_cloud, ToVec3f(pt));
    idx = nearby_idx.at(closest_point_idx);
    closest_pt = cloud_->points[idx];

    return true;
}

template <int dim>
bool GridNN<dim>::GetClosestPointForCloud(CloudPtr ref, CloudPtr query,
                                          std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    std::vector<size_t> index(query->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    std::for_each(index.begin(), index.end(), [this, &matches, &query](const size_t& idx) {
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            matches.emplace_back(cp_idx, idx);
        }
    });

    return true;
}

template <int dim>
bool GridNN<dim>::GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query,
                                            std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    // 与串行版本基本一样，但matches需要预先生成，匹配失败时填入非法匹配
    std::vector<size_t> index(query->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    matches.resize(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &matches, &query](const size_t& idx) {
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            matches[idx] = {cp_idx, idx};
        } else {
            matches[idx] = {math::kINVALID_ID, math::kINVALID_ID};
        }
    });

    return true;
}

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_GRID2D_HPP
