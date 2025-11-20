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
     * @param dim 模版参数 2D或者3D
     */
    template <int dim>
    class GridNN {
        public:
        using KeyType = Eigen::Matrix<int, dim, 1>;
        using PtType = Eigen::Matrix<float, dim, 1>;

        enum class NearbyType {
            CENTER,
            // for 2d
            NEARBY4, // 上下左右
            NEARBY8, // 上下左右 + 四角

            // for 3d
            NEARBY6, // 上下左右前后
        };

        /**
         * 构造函数
         * @param resolution 分辨率
         * @param nearby_type 近邻判定方法
         */
        explicit GridNN(float resolution = 0.1, NearbyType nearby_type_ = NearbyType::NEARBY4)
            : resolution_(resolution), nearby_type_(nearby_type) {
                inv_resolution_ = 1.0 / resolution_;

                // chekc dim and nearby
                if ( dim == 2 && nearby_type_ == NearbyType::NEARBY6) {
                    LOG(INFO) << "2D grid does not support nearby6, using nearby4 instead.";
                    nearby_type_ = NearbyType::NEARBY4;
                } else if ( dim == 3 && (nearby_type_ != NearbyType::NEARBY6 && nearby_type_ != NearbyType::CENTER)) {
                    LOG(INFO) << "3D grid does not support nearby4/8, using nearby6 instead.";
                    nearby_type_ = NearbyType::NEARBY6; 
                }

                GenerateNearbyGrids();
            }
    
        /// 获取最近邻
        bool GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx);


        private:
        /// 根据最近邻的类型，生成附近网格
        void GenerateNearbyGrids();

        // 空间坐标转到grid
        KeyType Pos2Grid(const PtType& pt);

        float resolution_ = 0.1;
        float inv_resolution_ = 10.0;

        NearbyType nearby_type_ = NearbyType::NEARBY4;
        std::unordered_map<KeyType, std::vector<size_t>, hash_vec<dim>> grids_;  // 栅格数据
        CloudPtr cloud_;

        std::vector<KeyType> nearby_grids_;  // 附近的栅格
    };

    // // 模版的具体实现
    // template <int dim>
    // bool GridNN<dim>

    template<int dim>
    Eigen::Matrix<int, dim, 1> GridNN<dim>::Pos2Grid(const Eigen::Matrix<float, dim, 1>& pt) {
        return pt.array().template round().template cast<int>();
    }
    
    template<>
    void GridNN<2>::GenerateNearbyGrids() {
        if (nearby_type_ == NearbyType::CENTER) {
            nearby_grids_ .emplace_back(KeyType::Zero());
        } else if (nearby_type_ == NearbyType::NEARBY4) {
            nearby_grids_ = {Vec2i(0, 0 ), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1)};
        } else if (nearby_type_ == NearbyType::NEARBY8 ) {
            nearby_grids_ = {
                Vec2i(0, 0),   Vec2i(-1, 0), Vec2i(1, 0 ), Vec2i( 0, -1), Vec2i(0, 1),
                Vec2i(-1, -1), Vec2i(-1, 1), Vec2i(1, -1), Vec2i(1, 1),
            };
        }
    }
    template<>
    void GridNN<3>::GenerateNearbyGrids() {
        if (nearby_type_ == NearbyType::CENTER) {
            nearby_grids_.emplace_back(KeyType::Zero());
        } else if (nearby_type_ == NearbyType::NEARBY6 ) {
            nearby_grids_ = {
                Vec2i(0, 0, 0),   
                Vec2i(-1, 0, 0), Vec2i(1, 0 , 0), 
                Vec2i( 0, -1, 0), Vec2i(0, 1, 0),
                Vec2i(0, 0, -1),  Vec2i(0,  0, 1)};
        }
    }

    template <int dim>
    bool GridNN<dim>::GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx){
        // 在pt栅格周边寻找最近邻
        std::vector<size_t> idx_to_check;
        auto key = Pos2Grid(ToEigen<float, dim>(pt));

        std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &idx_to_check, this](const KeyType& delta) {
            auto deky - key + delta;
            auot iter = grids_.find(dkey);
            if (iter != grids_.find(dkey)){
                idx_to_check.insert(idx_to_check.end(), iter->second.begin(), iter->second.end());
            }
        });

        if (idx_to_check.empty()) {
            return false;
        }

        // brute force nn in cloud_[idx]
        CloudPtr nearby_cloud(new PointCloudType);
        std::vector<size_t> nearby_idx;
        for (auto & idx: idx_to_chekc){
            nearby_cloud->points.template emplace_back(clud_=>points[idx]);
            nearby_idx.emplac_back(idx);
        }

        size_t closeet_point_idx = bfnn_point()
    }

}
#endif  // SLAM_IN_AUTO_DRIVING_GRID2D_HPP
