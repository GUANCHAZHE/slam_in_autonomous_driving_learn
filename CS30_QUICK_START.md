# CS30数据集快速开始指南

## ⚡ 5分钟快速开始

### 编译
```bash
cd /home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/build
cmake ..
make test_call_back -j4
```

### 运行自定义数据集
```bash
./bin/test_call_back --use_custom_dataset=true
```

### 查看输出
输出文件位置:
```
./dataset/sad/ulhk/cs30_output_cloud.pcd
```

## 📋 关键参数配置

### 使用自定义rosbag路径
```bash
./bin/test_call_back --use_custom_dataset=true \
  --custom_bag_path="/path/to/your/data.bag"
```

### 使用自定义话题名称
```bash
./bin/test_call_back --use_custom_dataset=true \
  --pointcloud_topic="/your/lidar/topic" \
  --imu_topic="/your/imu/topic"
```

### 同时指定所有参数
```bash
./bin/test_call_back --use_custom_dataset=true \
  --custom_bag_path="/path/to/data.bag" \
  --pointcloud_topic="/camera1_SD0140820L0057/points2" \
  --imu_topic="/imu/data"
```

## 📊 CS30数据集信息速览

- **位置**: `/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_ros1_converted.bag`
- **大小**: 733.1 MB
- **时长**: 30.8秒
- **点云**:
  - 话题: `/camera1_SD0140820L0057/points2`
  - 消息数: 456
  - 类型: PointCloud2
- **IMU**:
  - 话题: `/imu/data`
  - 消息数: 533
  - 类型: Imu

## 🔍 验证数据集

```bash
# 查看rosbag信息
rosbag info cs30_ros1_converted.bag

# 列出所有话题
rosbag info cs30_ros1_converted.bag | grep "topics:"

# 播放rosbag (可选)
rosbag play cs30_ros1_converted.bag
```

## 📁 代码修改位置

修改文件: `src/ch7/test/test_call_back.cc`

| 功能 | 行号 |
|------|------|
| 命令行参数 | 59-64 |
| 加载IMU函数 | 430-455 |
| 加载点云函数 | 458-492 |
| 测试函数 | 1033-1130 |
| Main函数 | 1193-1210 |

## ✅ 验证修改成功

编译后应该没有错误:
```bash
make test_call_back 2>&1 | grep -i error
```

运行时查看日志:
```
开始加载自定义rosbag的IMU数据，话题: /imu/data
结束加载自定义rosbag的IMU数据
总共读取了 XXX 条IMU数据

开始加载自定义rosbag的点云数据，话题: /camera1_SD0140820L0057/points2
结束加载自定义rosbag的点云数据
总共读取了 XXX 帧点云数据
```

## 🐛 常见问题

**Q: 找不到rosbag文件**
```bash
A: 检查文件路径
ls -lh /home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/cs30_ros1_converted.bag
```

**Q: IMU/点云数据加载为0**
```bash
A: 检查话题名称是否正确
rosbag info cs30_ros1_converted.bag | grep "topics:" -A 50
```

**Q: IMU初始化失败**
```bash
A: 这是正常的，程序会使用默认配置继续运行
```

**Q: 输出点云文件位置**
```bash
./dataset/sad/ulhk/cs30_output_cloud.pcd
```

## 📝 默认配置

```
Rosbag路径: ./dataset/sad/ulhk/cs30_ros1_converted.bag
点云话题: /camera1_SD0140820L0057/points2
IMU话题: /imu/data
输出文件: ./dataset/sad/ulhk/cs30_output_cloud.pcd
体素化大小: 0.05m
NDT栅格大小: 0.15m
关键帧位移阈值: 0.1m
关键帧角度阈值: 3°
局部地图关键帧数: 30
```

## 🚀 优化建议

1. **加速编译**: 使用并行编译
   ```bash
   make test_call_back -j8
   ```

2. **查看详细日志**: 设置日志级别
   ```bash
   ./bin/test_call_back --use_custom_dataset=true --v=2
   ```

3. **处理大型点云**: 调整体素化大小
   ```bash
   修改代码中的 voxel_size = 0.1 (更大的值更快)
   ```

4. **调试IMU数据**: 启用IMU日志
   ```bash
   在代码中取消注释IMU打印语句
   ```

## 📚 相关文件

- 主代码: `src/ch7/test/test_call_back.cc`
- 完整文档: `CUSTOM_DATASET_INTEGRATION_GUIDE.md`
- 原始指南: `CUSTOM_DATASET_GUIDE.md`
- IO工具: `src/common/io_utils.h/cc`
- 数据类型: `src/common/dataset_type.h`
- IMU定义: `src/common/imu.h`

