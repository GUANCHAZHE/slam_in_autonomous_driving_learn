
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "ch5_my/bfnn.h"
#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "common/sys_utils.h"

DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

TEST(CH5__Test, BFNN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()){
        LOG(ERROR) << " can not load cloud";
        FAIL();
    }

    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    LOG(INFO) <<" points:" << first->size() << ", " << second->size();

    // 评价单线程和多线程的暴力匹配版本
    sad::evaluate_and_call(
        [&first, & second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            sad::bfnn_cloud(first,second, matches);
        },
        "暴力匹配（单线程）",5
    );
    sad::evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t,size_t>> matches;
            sad::bfnn_cloud_mt(first,second, matches);
        },
        "暴力匹配(多线程)", 5
    );
    
    SUCCEED();
}