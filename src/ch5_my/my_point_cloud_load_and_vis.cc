<<<<<<< HEAD
=======

>>>>>>> 5185ab465f9db083b803ccd7477c8a0567b1a7ba
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
<<<<<<< HEAD
=======

>>>>>>> 5185ab465f9db083b803ccd7477c8a0567b1a7ba
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");

<<<<<<< HEAD
int main(int argc, char ** argv) {
    // 加载日志程序 和 相关的标志位程序
=======

/// 本程序可用于显示单个点云，演示PCL的基本用法
int main(int argc, char** argv){
>>>>>>> 5185ab465f9db083b803ccd7477c8a0567b1a7ba
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

<<<<<<< HEAD
    LOG(INFO) << ("stat program started");
    if (FLAGS_pcd_path.empty()) {
=======
    if ( FLAGS_pcd_path.empty()) {
>>>>>>> 5185ab465f9db083b803ccd7477c8a0567b1a7ba
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

<<<<<<< HEAD

    // // 读取点云
    // PointCloudType::Ptr cloud(new PointCloudType);
    // pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);
=======
    // 读取点云
    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);

    if (cloud->empty()) {
        LOG(ERROR) << " cannot load cloud file";
        return -1;
    }

    LOG(INFO) << "cloud points:" << cloud->size();

    // visualize
    pcl::visualization::PCLVisualizer viewer("cloud viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> handle(cloud, "z"); // z as color
    viewer.addPointCloud<PointType>(cloud, handle);
    viewer.spin();

    return 0;
>>>>>>> 5185ab465f9db083b803ccd7477c8a0567b1a7ba
}