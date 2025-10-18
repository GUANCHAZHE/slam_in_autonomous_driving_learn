#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");

int main(int argc, char ** argv) {
    // 加载日志程序 和 相关的标志位程序
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << ("stat program started");
    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }


    // // 读取点云
    // PointCloudType::Ptr cloud(new PointCloudType);
    // pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);
}