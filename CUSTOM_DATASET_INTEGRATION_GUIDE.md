# CS30自定义数据集集成说明

## 概述

我已经在 `test_call_back.cc` 中成功添加了对自定义CS30数据集的支持。此修改允许程序从自定义rosbag文件中加载点云和IMU数据，并进行NDT+IMU融合的里程计测试。

## 修改内容详解

### 1. 新增命令行参数 (lines 59-64)

```cpp
// 自定义数据集配置 (My_dataset - CS30)
DEFINE_string(custom_bag_path, "./dataset/sad/ulhk/cs30_ros1_converted.bag", "path to custom rosbag");
DEFINE_string(custom_dataset_type, "CUSTOM", "Custom dataset type");
DEFINE_string(custom_config, "./config/velodyne_ulhk.yaml", "path of custom config yaml");
DEFINE_string(pointcloud_topic, "/camera1_SD0140820L0057/points2", "PointCloud2 topic name");
DEFINE_string(imu_topic, "/imu/data", "IMU topic name");
DEFINE_bool(use_custom_dataset, false, "use custom dataset?");
```

**说明**:
- `custom_bag_path`: 指定自定义rosbag文件的完整路径
- `pointcloud_topic`: 点云数据的ROS话题名称
- `imu_topic`: IMU数据的ROS话题名称
- `use_custom_dataset`: 标志是否使用自定义数据集

### 2. 新增头文件 (lines 49-51)

```cpp
// ROS消息头文件
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/message_instance.h>
```

这些头文件支持处理ROS消息格式的IMU和PointCloud2数据。

### 3. 新增函数 1: `loadCustomRosbagImuData()`

**功能**: 从自定义rosbag加载IMU数据

**参数**: 
- `imu_data_buffer`: IMU数据缓冲区引用

**实现要点**:
- 使用 `sad::DatasetType::MY_dataset` 数据集类型
- 使用 `AddHandle()` 通用处理函数来处理自定义话题名称的IMU数据
- 手动解析 `sensor_msgs::Imu` 消息并创建 `sad::IMU` 对象
- 返回加载的IMU数据数量

```cpp
void loadCustomRosbagImuData(std::vector<std::shared_ptr<sad::IMU>>& imu_data_buffer)
{
    sad::RosbagIO rosbag_io(FLAGS_custom_bag_path, sad::DatasetType::MY_dataset);
    LOG(INFO) << "开始加载自定义rosbag的IMU数据，话题: " << FLAGS_imu_topic;
    
    rosbag_io.AddHandle(FLAGS_imu_topic, [&](const rosbag::MessageInstance &m) -> bool {
        auto msg = m.instantiate<sensor_msgs::Imu>();
        if (msg == nullptr) {
            return false;
        }
        
        auto imu = std::make_shared<sad::IMU>(
            msg->header.stamp.toSec(),
            Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)
        );
        imu_data_buffer.emplace_back(imu);
        return true;
    }).Go();
    
    LOG(INFO) << "结束加载自定义rosbag的IMU数据";
    std::cout << "总共读取了 " << imu_data_buffer.size() << " 条IMU数据" << std::endl;
}
```

### 4. 新增函数 2: `loadCustomRosbagPointCloudData()`

**功能**: 从自定义rosbag加载点云数据

**参数**:
- `cloud_data_buffer`: 点云数据缓冲区引用

**实现要点**:
- 使用 `AddPointCloud2Handle()` 处理PointCloud2消息
- 将ROS格式的PointCloud2转换为PCL格式
- 再转换为 `sad::PointType` 格式

```cpp
void loadCustomRosbagPointCloudData(std::vector<sad::CloudPtr>& cloud_data_buffer)
{
    sad::RosbagIO rosbag_io(FLAGS_custom_bag_path, sad::DatasetType::MY_dataset);
    LOG(INFO) << "开始加载自定义rosbag的点云数据，话题: " << FLAGS_pointcloud_topic;
    
    rosbag_io.AddPointCloud2Handle(FLAGS_pointcloud_topic, [&](sensor_msgs::PointCloud2::Ptr msg) -> bool {
        sad::FullCloudPtr full_cloud(new sad::FullPointCloudType);
        pcl::fromROSMsg(*msg, *full_cloud);
        
        sad::CloudPtr cloud(new sad::PointCloudType);
        cloud->resize(full_cloud->size());
        for (size_t i = 0; i < full_cloud->size(); ++i) {
            cloud->points[i].x = full_cloud->points[i].x;
            cloud->points[i].y = full_cloud->points[i].y;
            cloud->points[i].z = full_cloud->points[i].z;
            cloud->points[i].intensity = full_cloud->points[i].intensity;
        }
        cloud->width = full_cloud->width;
        cloud->height = full_cloud->height;
        cloud->is_dense = full_cloud->is_dense;
        
        cloud_data_buffer.emplace_back(cloud);
        return true;
    }).Go();
    
    LOG(INFO) << "结束加载自定义rosbag的点云数据";
    std::cout << "总共读取了 " << cloud_data_buffer.size() << " 帧点云数据" << std::endl;
}
```

### 5. 新增函数 3: `test_Ndt_LO_CustomDataset()`

**功能**: 使用自定义数据集进行NDT+IMU融合的里程计测试

**主要步骤**:
1. 加载自定义rosbag的点云数据
2. 加载自定义rosbag的IMU数据
3. 进行IMU静止初始化
4. 遍历每一帧点云:
   - 体素化点云
   - NDT配准
   - IMU融合
   - 关键帧判定
   - 更新局部地图
5. 输出融合后的点云地图

**输出文件**: `./dataset/sad/ulhk/cs30_output_cloud.pcd`

### 6. 修改main()函数

**新增逻辑**: 根据 `use_custom_dataset` 标志选择使用自定义数据集还是原始ULHK数据集

```cpp
if (FLAGS_use_custom_dataset) {
    LOG(INFO) << "使用自定义数据集 (CS30)";
    test_Ndt_LO_CustomDataset(is_vis);
} else {
    LOG(INFO) << "使用原始ULHK数据集";
    test_Ndt_LO(is_vis);
}
```

## 编译方法

```bash
cd /home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/build
cmake ..
make test_call_back
```

## 使用方法

### 方法1: 使用自定义CS30数据集 (推荐)

```bash
./bin/test_call_back --use_custom_dataset=true
```

### 方法2: 使用原始ULHK数据集 (默认)

```bash
./bin/test_call_back
```

或显式指定:
```bash
./bin/test_call_back --use_custom_dataset=false
```

### 方法3: 完全自定义所有参数

```bash
./bin/test_call_back \
  --use_custom_dataset=true \
  --custom_bag_path="/path/to/your/data.bag" \
  --pointcloud_topic="/your/pointcloud/topic" \
  --imu_topic="/your/imu/topic"
```

## CS30数据集信息

| 属性 | 值 |
|------|-----|
| 文件路径 | `/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_ros1_converted.bag` |
| 时长 | 30.8秒 |
| 总消息数 | 4712条 |
| 点云话题 | `/camera1_SD0140820L0057/points2` |
| 点云消息数 | 456条 |
| 点云类型 | `sensor_msgs/PointCloud2` |
| IMU话题 | `/imu/data` |
| IMU消息数 | 533条 |
| IMU类型 | `sensor_msgs/Imu` |
| 压缩格式 | 无 |

## 关键参数说明

### NDT配准参数 (在 `test_Ndt_LO_CustomDataset()` 中)

```cpp
sad::Ndt3d::Options ndt_options;
ndt_options.voxel_size_ = 0.15;      // NDT栅格大小
ndt_options.max_iteration_ = 30;     // 最大迭代次数
ndt_options.min_effective_pts_ = 5;  // 最小有效点数
```

### 体素化参数

```cpp
double voxel_size = 0.05;  // 体素化大小 (5厘米)
```

### 局部地图参数

```cpp
int num_kfs_in_local_map = 30;  // 组成局部地图的关键帧数量
```

### 关键帧判定条件

```cpp
// 在 IsKeyframe() 函数中
return delta.translation().norm() > 0.1 || delta.so3().log().norm() > 3 * sad::math::kDEG2RAD;
```

- 位移阈值: 0.1m
- 角度阈值: 3度

## 数据流程

```
CS30 Rosbag 文件
    ↓
加载点云数据 (PointCloud2 → sad::CloudPtr)
    ↓
加载IMU数据 (sensor_msgs::Imu → sad::IMU)
    ↓
IMU静止初始化 (前3000条IMU数据)
    ↓
对每一帧点云:
    ├── 体素化
    ├── NDT配准
    ├── IMU预测 + EKF融合
    ├── 关键帧判定
    └── 更新局部地图
    ↓
输出融合后的点云地图 (.pcd格式)
```

## 故障排除

### 问题1: rosbag文件无法打开

**错误信息**: `Cannot open bag file`

**原因**: 文件路径不存在或权限不足

**解决方案**:
```bash
# 检查文件是否存在
ls -lh /home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_ros1_converted.bag

# 检查文件权限
chmod 644 cs30_ros1_converted.bag
```

### 问题2: 找不到指定话题

**错误信息**: 加载数据为0

**原因**: 话题名称与rosbag中的实际名称不匹配

**解决方案**:
```bash
# 查看rosbag中的所有话题
rosbag info cs30_ros1_converted.bag

# 根据实际话题修改参数
./bin/test_call_back \
  --use_custom_dataset=true \
  --pointcloud_topic="/actual/topic/name" \
  --imu_topic="/actual/imu/topic"
```

### 问题3: IMU初始化失败

**日志**: `IMU初始化失败，使用默认配置`

**原因**: 可能是静止初始化数据不足或质量差

**解决方案**:
- 确保IMU数据已成功加载
- 检查IMU数据时间戳是否有效
- 确认设备在初始化期间保持静止

## 关键改进点

1. **灵活的话题支持**: 通过命令行参数指定话题名称，不需要修改代码
2. **数据类型转换**: 正确处理ROS消息到内部数据结构的转换
3. **完整的IMU融合**: 包含IMU初始化、预测和EKF融合
4. **本地地图维护**: 使用关键帧管理局部地图，保持计算效率
5. **输出结果保存**: 自动保存融合后的点云地图

## 后续扩展建议

1. **支持更多数据集**: 通过添加新的数据集类型和话题配置
2. **参数配置文件**: 将参数保存到YAML文件而不是硬编码
3. **实时可视化**: 添加实时的点云和轨迹可视化
4. **性能优化**: 使用多线程加载和处理数据
5. **评估指标**: 添加轨迹精度和处理时间评估

## 参考资源

- [Sensor_msgs/PointCloud2文档](http://docs.ros.org/en/api_docs/cpp/classsensor__msgs_1_1PointCloud2.html)
- [Sensor_msgs/Imu文档](http://docs.ros.org/en/api_docs/cpp/classsensor__msgs_1_1Imu.html)
- [RosBag格式说明](http://wiki.ros.org/rosbag)
- [PCL PointCloud2转换教程](https://pcl.readthedocs.io/projects/tutorials/en/latest/converting_a_PointCloud2_message_to_PCL_PointCloud.html)

