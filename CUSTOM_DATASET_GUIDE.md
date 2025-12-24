# 自定义数据集 (CS30) 使用指南

## 概述
本文档说明如何在 `test_call_back.cc` 中使用自定义的CS30数据集进行NDT+IMU融合的里程计测试。

## 数据集信息
- **数据集名称**: CS30 (My_dataset)
- **Rosbag路径**: `/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_ros1_converted.bag`
- **点云话题**: `/camera1_SD0140820L0057/points2`
- **IMU话题**: `/imu/data`
- **时长**: 30.8秒
- **消息数**: 4712条

### Rosbag内容详情
```
点云数据:
  - 话题: /camera1_SD0140820L0057/points2
  - 消息数: 456条
  - 类型: sensor_msgs/PointCloud2

IMU数据:
  - 话题: /imu/data
  - 消息数: 533条
  - 类型: sensor_msgs/Imu
```

## 代码修改说明

### 1. 新增命令行参数

```cpp
// 自定义数据集配置 (My_dataset - CS30)
DEFINE_string(custom_bag_path, "./dataset/sad/ulhk/cs30_ros1_converted.bag", "path to custom rosbag");
DEFINE_string(custom_dataset_type, "CUSTOM", "Custom dataset type");
DEFINE_string(custom_config, "./config/velodyne_ulhk.yaml", "path of custom config yaml");
DEFINE_string(pointcloud_topic, "/camera1_SD0140820L0057/points2", "PointCloud2 topic name");
DEFINE_string(imu_topic, "/imu/data", "IMU topic name");
DEFINE_bool(use_custom_dataset, false, "use custom dataset?");
```

### 2. 新增函数

#### a) `loadCustomRosbagImuData()`
从自定义rosbag加载IMU数据，支持自定义话题名称。

```cpp
void loadCustomRosbagImuData(std::vector<IMUPtr>& imu_data_buffer)
{
    sad::RosbagIO rosbag_io(FLAGS_custom_bag_path, sad::DatasetType::CUSTOM);
    LOG(INFO) << "开始加载自定义rosbag的IMU数据，话题: " << FLAGS_imu_topic;
    rosbag_io
        .AddImuHandle([&](IMUPtr imu) {
            imu_data_buffer.emplace_back(imu);
            return true;
        }, FLAGS_imu_topic)
        .Go();
    LOG(INFO) << "总共读取了 " << imu_data_buffer.size() << " 条IMU数据";
}
```

#### b) `loadCustomRosbagPointCloudData()`
从自定义rosbag加载点云数据，支持自定义话题名称。

```cpp
void loadCustomRosbagPointCloudData(std::vector<sad::CloudPtr>& cloud_data_buffer)
{
    sad::RosbagIO rosbag_io(FLAGS_custom_bag_path, sad::DatasetType::CUSTOM);
    LOG(INFO) << "开始加载自定义rosbag的点云数据，话题: " << FLAGS_pointcloud_topic;
    rosbag_io
        .AddPointCloudHandle([&](sad::CloudPtr cloud) {
            cloud_data_buffer.emplace_back(cloud);
            return true;
        }, FLAGS_pointcloud_topic)
        .Go();
    LOG(INFO) << "总共读取了 " << cloud_data_buffer.size() << " 帧点云数据";
}
```

#### c) `test_Ndt_LO_CustomDataset()`
使用自定义数据集进行NDT+IMU融合的里程计测试。该函数：
- 加载自定义rosbag中的点云和IMU数据
- 进行IMU静止初始化
- 使用NDT算法进行点云配准
- 使用ESKF进行IMU融合
- 输出融合后的地图

## 使用方法

### 编译
```bash
cd /home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/build
cmake ..
make test_call_back
```

### 运行方式

#### 方式1: 使用自定义数据集 (CS30)
```bash
./bin/test_call_back --use_custom_dataset=true
```

#### 方式2: 使用原始ULHK数据集（默认）
```bash
./bin/test_call_back --use_custom_dataset=false
```

#### 方式3: 自定义所有参数
```bash
./bin/test_call_back \
  --use_custom_dataset=true \
  --custom_bag_path="/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_ros1_converted.bag" \
  --pointcloud_topic="/camera1_SD0140820L0057/points2" \
  --imu_topic="/imu/data"
```

## 输出说明

程序运行后会输出以下信息：

1. **日志输出**
   - IMU数据加载数量
   - 点云数据加载数量
   - 每一帧的处理进度
   - 关键帧检测结果
   - NDT配准位姿

2. **输出文件**
   - 位置: `./dataset/sad/ulhk/cs30_output_cloud.pcd`
   - 内容: 融合后的点云地图

## 关键参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `custom_bag_path` | `./dataset/sad/ulhk/cs30_ros1_converted.bag` | 自定义rosbag路径 |
| `pointcloud_topic` | `/camera1_SD0140820L0057/points2` | 点云话题名称 |
| `imu_topic` | `/imu/data` | IMU话题名称 |
| `use_custom_dataset` | `false` | 是否使用自定义数据集 |
| `voxel_size` | `0.05` | 体素化大小 (代码内设置为0.05m) |
| `num_kfs_in_local_map` | `30` | 局部地图中的关键帧数量 |

## 算法流程

```
1. 加载点云数据
   ↓
2. 加载IMU数据
   ↓
3. IMU静止初始化 (前3000条数据)
   ↓
4. 对每一帧点云:
   a. 体素化点云
   b. NDT配准
   c. IMU预测与融合
   d. 关键帧判定
   e. 更新局部地图
   ↓
5. 输出融合后的地图
```

## 故障排除

### 问题1: 无法加载rosbag
**原因**: rosbag文件路径不正确或文件不存在
**解决**:
```bash
# 检查文件是否存在
ls -lh /home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_ros1_converted.bag

# 查看rosbag信息
rosbag info cs30_ros1_converted.bag
```

### 问题2: 找不到指定话题
**原因**: 话题名称与rosbag中的实际名称不匹配
**解决**:
```bash
# 查看rosbag中的所有话题
rosbag info cs30_ros1_converted.bag | grep -E "topics:|^\s+/"

# 根据实际话题名称修改命令行参数
./bin/test_call_back --use_custom_dataset=true \
  --pointcloud_topic="/actual/pointcloud/topic" \
  --imu_topic="/actual/imu/topic"
```

### 问题3: IMU初始化失败
**原因**: 可能是静止初始化数据不足或IMU数据质量差
**解决**: 检查日志输出，确保:
- IMU数据已成功加载
- 设备在初始化期间保持静止

## 扩展与定制

### 添加新的自定义数据集
1. 在命令行参数中添加新的rosbag路径和话题名称
2. 修改 `test_Ndt_LO_CustomDataset()` 函数中的参数
3. 重新编译并运行

示例:
```cpp
DEFINE_string(custom_bag_path, "./dataset/new_dataset/data.bag", "path to rosbag");
DEFINE_string(pointcloud_topic, "/new_pointcloud_topic", "PointCloud2 topic name");
DEFINE_string(imu_topic, "/new_imu_topic", "IMU topic name");
```

## 参考资源

- [ROS Bag格式文档](http://wiki.ros.org/rosbag)
- [sensor_msgs/PointCloud2 消息格式](http://docs.ros.org/en/api_docs/cpp/classsensor__msgs_1_1PointCloud2.html)
- [sensor_msgs/Imu 消息格式](http://docs.ros.org/en/api_docs/cpp/classsensor__msgs_1_1Imu.html)
