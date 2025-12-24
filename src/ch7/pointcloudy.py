# -*- coding: utf-8 -*-
import time
from cv2 import merge
import open3d as o3d
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import os
import glob
from pathlib import Path
import copy  # 用于深拷贝，防止修改原始数据
import sophuspy as sp
from scipy.spatial.transform import Rotation as Rscipy  # 你前面已经导入过
import pcl

# 顶部添加：
from scipy.spatial.transform import Rotation as R_scipy

# For ROS2 support
try:
    from rosbags import rosbag2
    from rosbags.typesys import get_types_from_msg, register_types
    from rosbags.highlevel import AnyReader
    import rosbags.serde
    ROS2_SUPPORT_AVAILABLE = True
except ImportError:
    ROS2_SUPPORT_AVAILABLE = False

class PointCloudPlayer:
    """点云加载、播放和配准工具类"""

    def __init__(self, frame_path=None, bag_path=None, topic_name=None, frame_path_pcd=None):
        """
        初始化点云播放器
        
        Args:
            frame_path: 点云帧文件夹路径
            bag_path: rosbag 文件路径
            topic_name: rosbag 中的话题名称
        """
        self.frame_path = frame_path or "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames"
        self.frame_path_pcd = frame_path_pcd or "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames_pcd"
        self.frame_path_pcd_ros2 = frame_path_pcd or "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames_pcd_ros2"
        self.bag_path = bag_path or "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/test2.bag"
        # self.bag_path_ros2 = bag_path or "/home/keyirobot/Desktop/qixing_ws/rosbag/rosbag2_2000_01_01-08_05_21/"   # 直线
        # self.bag_path_ros2 = bag_path or "/home/keyirobot/Desktop/qixing_ws/rosbag/rosbag2_2000_01_01-08_24_07/"   # 旋转
        # self.bag_path_ros2 = bag_path or "/home/keyirobot/Desktop/qixing_ws/rosbag/rosbag2-cs30-0/"                  # 旋转
        self.bag_path_ros2 = bag_path or "/home/keyirobot/Desktop/qixing_ws/rosbag/cs30-depth-imu-13/"                  # 



        self.topic_name = topic_name or "/velodyne_points_0"
        # self.topic_name_ros2 = topic_name or "/camera/depth/points"
        # self.topic_name_ros2 = topic_name or "/camera/depth/points"           # astra
        self.topic_name_ros2 = topic_name or "/camera1_SD0140820L0057/points2"  # cs30

        

        self.all_frames = []
        self.scan_world = []

        self.map = o3d.geometry.PointCloud()
        self.source = o3d.geometry.PointCloud()
        self.target = o3d.geometry.PointCloud()

        self.last_kf_pose_ = np.eye(4)
        self.estimated_pose = []
        self.kf_distance_ = 0.5     # 关键帧距离阈值（米）
        self.kf_angle_deg_ = 10.0   # 关键帧角度阈值（度）

    @staticmethod
    def numpy_to_o3d(points_np):
        """
        将 Numpy 数组转换为 Open3D 点云对象
        
        功能说明：
        将形状为 (N, 3) 的 NumPy 数组转换为 Open3D 的 PointCloud 对象，
        用于后续的点云处理、可视化和配准操作。
        
        参数：
            points_np: numpy.ndarray
                输入的点云数据，形状为 (N, 3)，其中 N 是点的数量，
                每一行包含一个点的 [x, y, z] 坐标
                
        返回：
            pcd: o3d.geometry.PointCloud
                Open3D 点云对象，包含输入的所有点坐标
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)
        return pcd
    
    def save_pcd_open3d(self, points_np, save_path):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np.astype(np.float64))
        o3d.io.write_point_cloud(save_path, pcd, write_ascii=False)   # 二进制更小

    def matrix_to_norm_angle(R):
        trace = np.trace(R)
        angle = np.arccos((trace - 1) / 2)
        return angle

    def load_all_frame_npy(self, frame_path=None):
        """
        读取并预加载所有 npy 格式的点云文件
        
        Args:
            frame_path: 点云帧文件夹路径，如果为 None 则使用初始化时的路径
            
        Returns:
            list: 所有点云帧的列表
        """
        if frame_path is None:
            frame_path = self.frame_path
            
        file_paths = sorted(glob.glob(frame_path + "/*.npy"))

        if not file_paths:
            print("未找到文件，请检查路径")
            return []

        # 预加载数据到内存 (解决卡顿问题)
        print("正在预加载所有点云数据，请稍候...")
        self.all_frames = []
        for f in file_paths:
            self.all_frames.append(np.load(f))
        print("预加载完成，共 {} 帧".format(len(self.all_frames)))
        
        return self.all_frames

    def play_pointcloud_sequence_npy(self, all_frames=None, interval_seconds=0.05):
        """
        播放 npy 格式的点云序列
        
        Args:
            all_frames: 点云帧列表，如果为 None 则使用已加载的帧
            interval_seconds: 每帧播放间隔（秒），默认 50ms 一帧
        """
        if all_frames is None:
            all_frames = self.all_frames

        if not all_frames:
            print("没有可播放的点云数据")
            return

        vis = o3d.visualization.Visualizer()
        vis.create_window("Point Cloud Player", width=800, height=600)

        pcd = o3d.geometry.PointCloud()

        # 初始化第一帧
        pcd.points = o3d.utility.Vector3dVector(all_frames[0])
        vis.add_geometry(pcd)

        # 只在开始时重置一次视角
        vis.reset_view_point(True)

        for i in range(len(all_frames)):
            print("当前的播放帧 frame: {}".format(i))
            # 更新数据
            points = all_frames[i]
            pcd.points = o3d.utility.Vector3dVector(points)

            scan = pcd

            # 告知Open3D几何体已更新
            vis.update_geometry(scan)
            vis.poll_events()
            vis.update_renderer()

            # 控制播放速度
            start_t = time.time()
            while time.time() - start_t < interval_seconds:
                vis.poll_events()  # 在等待期间也要响应鼠标操作

        # 播放结束后保持窗口不关闭
        vis.run()
        vis.destroy_window()

    def play_pointcloud_sequence(self, frame_list, interval_ms=20):
        """
        播放点云序列 (frame_list 格式)
        
        Args:
            frame_list: 点云帧列表 [np.ndarray(N,3), np.ndarray(N,3), ...]
            interval_ms: 每帧刷新间隔（毫秒），默认 20ms
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window("Point Cloud Player")

        # 初始点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(frame_list[0])
        vis.add_geometry(pcd)

        for i, frame in enumerate(frame_list):
            # 更新点云内容
            pcd.points = o3d.utility.Vector3dVector(frame)
            vis.update_geometry(pcd)

            # 刷新渲染
            vis.poll_events()
            vis.update_renderer()

            # 播放间隔
            time.sleep(interval_ms / 1000.0)

        vis.destroy_window()

    def load_rosbag_save_local(self, output_dir=None):
        """
        从 rosbag 读取点云数据并保存为 npy 文件
        
        Args:
            output_dir: 输出目录，如果为 None 则使用 frame_path
        """
        if output_dir is None:
            output_dir = self.frame_path

        print(f"rosbag_path: {self.bag_path}")
        print(f"topic_name: {self.topic_name}")
        print(f"output_dir: {output_dir}")

        os.makedirs(output_dir, exist_ok=True)

        bag = rosbag.Bag(self.bag_path, "r")

        frame_id = 0
        for topics, msg, t in bag.read_messages(topics=self.topic_name):
            # 将 pointcloud2 转为 numpy 格式 (N, 3)
            points = np.array([
                [p[0], p[1], p[2]]
                for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            ], dtype=np.float32)
            print(f"points.shape: {points.shape}")

            np.save(f"{output_dir}/frame_{frame_id:06d}.npy", points)
            frame_id += 1

        print(f"总共保存 {frame_id} 帧")
        bag.close()

    def load_rosbag_save_local_pcd(self, output_dir=None):
        """
        从 rosbag 读取点云数据并保存为 pcd 文件

        Args:
            output_dir: 输出目录，如果为 None 则使用 frame_path
        """
        if output_dir is None:
            output_dir = self.frame_path_pcd

        print(f"rosbag_path: {self.bag_path}")
        print(f"topic_name: {self.topic_name}")
        print(f"output_dir: {output_dir}")

        os.makedirs(output_dir, exist_ok=True)

        bag = rosbag.Bag(self.bag_path, "r")

        frame_id = 0

        for topic, msg, t in bag.read_messages(topics=self.topic_name):
            # 提取 x,y,z
            pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points_np = np.array(list(pts), dtype=np.float32)   # (N,3)

            # 转成 Open3D 点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_np.astype(np.float64))

            # 保存为 PCD 文件
            save_path = os.path.join(output_dir, f"frame_{frame_id:06d}.pcd")
            o3d.io.write_point_cloud(save_path, pcd, write_ascii=False)
            print(f"Saved {save_path} {points_np.shape[0]} points")

            frame_id += 1

        print(f"总共保存 {frame_id} 帧 PCD")
        bag.close()

    def load_rosbag2_save_local_pcd(self, output_dir=None):
            """
            从 ROS2 rosbag2 读取点云数据并保存为 pcd 文件
            (修复了 AnyReader 返回值解包顺序错误)
            """
            if not ROS2_SUPPORT_AVAILABLE:
                print("错误: ROS2 支持不可用，请安装 rosbags 库: pip install rosbags")
                return

            target_bag_path = self.bag_path_ros2
            target_topic = self.topic_name_ros2

            if output_dir is None:
                output_dir = self.frame_path_pcd_ros2

            print(f"rosbag2_path: {target_bag_path}")
            print(f"target_topic: {target_topic}")
            print(f"output_dir: {output_dir}")

            os.makedirs(output_dir, exist_ok=True)
            frame_id = 0

            if not os.path.exists(target_bag_path):
                print(f"错误: ROS2 bag路径不存在: {target_bag_path}")
                return

            try:
                from rosbags.highlevel import AnyReader
                from pathlib import Path
                import traceback

                with AnyReader([Path(target_bag_path)]) as reader:
                    connections = [x for x in reader.connections if x.topic == target_topic]

                    if not connections:
                        print(f"警告: 未找到话题 {target_topic}")
                        return

                    print(f"开始处理话题: {target_topic}")

                    for connection in connections:
                        # --- 核心修复：修正了解包顺序 (conn, ts, data) ---
                        for conn, timestamp, rawdata in reader.messages(connections=[connection]):
                            try:
                                # 反序列化
                                msg = reader.deserialize(rawdata, connection.msgtype)

                                if 'PointCloud2' in connection.msgtype:
                                    # 确保 raw_data 是 uint8 数组
                                    if hasattr(msg.data, 'tobytes'):
                                        raw_np = np.frombuffer(msg.data.tobytes(), dtype=np.uint8)
                                    else:
                                        raw_np = np.frombuffer(msg.data, dtype=np.uint8)

                                    x_offset = y_offset = z_offset = -1
                                    if hasattr(msg, 'fields'):
                                        for field in msg.fields:
                                            if field.name == 'x': x_offset = field.offset
                                            elif field.name == 'y': y_offset = field.offset
                                            elif field.name == 'z': z_offset = field.offset
                                    
                                    if x_offset >= 0 and y_offset >= 0 and z_offset >= 0:
                                        point_step = msg.point_step
                                        n_points = msg.width * msg.height
                                        
                                        if len(raw_np) == n_points * point_step:
                                            raw_reshaped = raw_np.reshape(n_points, point_step)
                                            
                                            xs = raw_reshaped[:, x_offset:x_offset+4].copy().view(np.float32)
                                            ys = raw_reshaped[:, y_offset:y_offset+4].copy().view(np.float32)
                                            zs = raw_reshaped[:, z_offset:z_offset+4].copy().view(np.float32)
                                            
                                            points_np = np.hstack((xs, ys, zs))
                                            mask = ~np.isnan(points_np).any(axis=1)
                                            points_np = points_np[mask]
                                            
                                            if len(points_np) > 0:
                                                pcd = o3d.geometry.PointCloud()
                                                pcd.points = o3d.utility.Vector3dVector(points_np.astype(np.float64))
                                                save_path = os.path.join(output_dir, "frame_{:06d}.pcd".format(frame_id))
                                                o3d.io.write_point_cloud(save_path, pcd, write_ascii=False)
                                                
                                                if frame_id % 50 == 0:
                                                    print(f"Saved frame {frame_id}, points: {len(points_np)}")
                                                frame_id += 1
                                        else:
                                            pass 

                            except Exception:
                                traceback.print_exc()
                                continue

            except Exception as e:
                import traceback
                traceback.print_exc()
                print(f"读取错误: {str(e)}")

            print(f"操作完成，总共保存 {frame_id} 帧 PCD")    
    @staticmethod
    def icp_registration(source, target, voxel_size=1.0, init_guess=None):
        """
        执行 ICP 点云配准
        
        Args:
            source: 输入点云 (o3d.geometry.PointCloud)
            target: 地图/目标点云 (o3d.geometry.PointCloud)
            voxel_size: 体素大小（越大越稳，越小越精细）
            init_guess: 初始变换估计，如果为 None 则使用单位矩阵
            
        Returns:
            配准结果对象
        """
        # 下采样
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)

        # 设置初始估计 - 只有在没有传入参数时才使用默认值
        if init_guess is None:
            init_guess = np.eye(4)

        # 设置 ICP 参数
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=50
        )

        # 执行 ICP 配准
        result = o3d.pipelines.registration.registration_icp(
            source_down,
            target_down,
            voxel_size,
            init_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria,
        )

        return result

    def test_icp_registration(self, all_frames=None, voxel_size=0.1):
        """
        测试 ICP 配准功能
        
        Args:
            all_frames: 点云帧列表，如果为 None 则使用已加载的帧
            voxel_size: 体素大小，默认 0.5
            
        Returns:
            配准结果对象
        """
        if all_frames is None:
            all_frames = self.all_frames

        if len(all_frames) < 2:
            print("帧数不足，无法进行配准测试")
            return None

        print("Converting numpy to Open3D format...")

        # 取出第0帧和第1帧，并转化为 Open3D 对象
        frame1_pcd = self.numpy_to_o3d(all_frames[900])
        frame2_pcd = self.numpy_to_o3d(all_frames[910])
        frame3_pcd = self.numpy_to_o3d(all_frames[915])
        source = self.numpy_to_o3d(all_frames[925])
        target = self.numpy_to_o3d(all_frames[900])

        # 900 到 0 的变换
        result1 = self.icp_registration(frame2_pcd, frame1_pcd, voxel_size=voxel_size)
        pose1900_0 = result1.transformation
        print(f"pose1900_0:\n{pose1900_0}")
        frame2_world = copy.deepcopy(frame2_pcd)     # 这里需要深拷贝
        frame2_world = frame2_world.transform(pose1900_0)
        
        # 925 到 900 的变换
        result2 = self.icp_registration(frame3_pcd, frame2_pcd, voxel_size=voxel_size)
        pose3925_900 = result2.transformation
        print(f"pose3925_900:\n{pose3925_900}")
        
        # 得到 925 相对于 0 的变换
        T_925_0 = pose1900_0 @ pose3925_900
        frame3_world = copy.deepcopy(frame3_pcd)     # 这里需要深拷贝
        frame3_world = frame3_world.transform(T_925_0)

        # map_all = frame2_world + frame3_world + frame1_pcd

        print("Starting ICP registration...")
        # 调用配准函数
        result = self.icp_registration(source, target, voxel_size=voxel_size)
        # result = self.icp_registration(target, source, voxel_size=voxel_size)
        # print(f"ICP Result: {result}")

        pose = result.transformation
        print(f"pose:\n{pose}")
        
        # 提取旋转矩阵 R (前3行，前3列)
        R = pose[:3, :3]
        # 提取平移向量 t (前3行，第4列)
        t = pose[:3, 3]

        print(f"Rotation Matrix (R):\n{R}")
        print(f"Translation Vector (t):\n{t}")


        ######  测试代码
        T_test = np.eye(4)
        t_offset_test = np.array([0, 0, 3])
        T_test[:3, :3] = R
        print(f"T_test:\n{T_test}")
        source_trans = copy.deepcopy(source)
        source_trans.transform(T_test)

        # # 将 source 点云变换到 target 坐标系下
        # source_trans = copy.deepcopy(source)
        # source_trans.transform(pose)

        # 变换后的设为红色，目标设为绿色

        # 为了可视化效果明显，先给点云上色
        source.paint_uniform_color([1, 0, 0])  # 红色
        target.paint_uniform_color([0, 0, 1])  # 蓝色
        source_trans.paint_uniform_color([0, 1, 0])   # 绿色

        frame1_pcd.paint_uniform_color([1, 0, 0])  # 红色
        frame2_pcd.paint_uniform_color([0, 0, 1])  # 蓝色
        frame2_world.paint_uniform_color([0, 0, 1])  # 蓝色
        frame3_pcd.paint_uniform_color([0, 1, 0])   # 绿色
        frame3_world.paint_uniform_color([0, 1, 0])   # 绿色

        # 创建坐标轴看原点
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])

        # mergeed_pcd = target + source_trans

        o3d.visualization.draw_geometries(
            [frame1_pcd, frame2_pcd, axis],
            # [frame1_pcd , frame2_world, frame3_world, axis],
            # [map_all, axis],
            # [source, axis],
            # [target, source, axis],
            # [source_trans, source, axis],
            # [source_trans, target, axis],
            # [source_trans, target, source, axis],
            # [mergeed_pcd, axis],
            window_name="ICP Registration Result",
            width=800,
            height=600
        )

        return result




    # 输入的scan 为当前帧的点云
    # 输入的pose 为当前帧的位姿
    def icp_lo(self, scan):
        """
        执行 ICP 里程计，并将结果转换为 SE3 格式
        
        Args:
            scan: 当前帧的点云 (Open3D PointCloud 对象)
        """
        # 初始化地图
        if len(self.map.points) == 0:
            self.map += scan
            print(f"初始化 map 点云数量: {len(self.map.points)}")

            self.last_kf_pose_ = np.eye(4)
            self.target = scan
            return

        self.source = scan
        # 实时计算当前帧和上一帧的之间的 ICP 结果
        result = self.icp_registration(self.source, self.target)
        
        # 从结果中提取变换矩阵 T (4x4)
        pose_matrix = result.transformation
        
        # 把当前帧的点云变换到世界坐标系下
        scan_world = scan.transform(pose_matrix)
        
        # 添加到队列
        self.scan_world.append(scan_world)
        self.estimated_pose.append(pose_matrix)  # 存储 SE3 对象而不是矩阵

        if self.Is_keyframe(pose_matrix):
            
            self.last_kf_pose_ = pose_matrix
            self.target = scan

            # 把当前帧的点云添加到地图中
            self.map += scan_world
            print(f"当前地图点云数量: {len(self.map.points)}")



    # def Is_keyframe(self, current_pose):

    #     delta = np.linalg.inv(self.last_kf_pose_) @ current_pose
    #     print(f"delta:\n{delta}")

    #     R_deta = delta[:3, :3]
    #     t_deta = delta[:3, 3]
    #     # 计算旋转角度
    #     rotation_angle_rad = np.linalg.norm(R_deta)
    #     rotation_angle_deg = np.degrees(rotation_angle_rad)
    #     print(f"rotation_angle_deg: {rotation_angle_deg}")
    #     # 计算平移距离
    #     translation_distance = np.linalg.norm(t_deta)
    #     print(f"translation_distance: {translation_distance}")
    #     # 判断是否为关键帧
    #     return rotation_angle_deg > self.kf_angle_deg_ or translation_distance > self.kf_distance_

    def Is_keyframe(self, current_pose):
            
            delta = np.linalg.inv(self.last_kf_pose_) @ current_pose
            print(f"delta:\n{delta}")
            R_deta = delta[:3, :3]
            print(f"R_deta:\n{R_deta}")
            t_deta = delta[:3, 3]
            print(f"t_deta:\n{t_deta}")
            
            # 核心修复 2：使用 scipy 库将旋转矩阵 R_deta 转换为旋转向量，再求模长 (即弧度)
            try:
                # R_scipy.from_matrix(R_deta).as_rotvec() 得到旋转向量
                rotation_vector = R_scipy.from_matrix(R_deta).as_rotvec()
                rotation_angle_rad = np.linalg.norm(rotation_vector)
                rotation_angle_deg = np.degrees(rotation_angle_rad)
            except ValueError:
                # 捕获无效旋转矩阵的异常
                print("警告: 相对旋转矩阵无效，跳过旋转检查。")
                rotation_angle_deg = 0.0

            print(f"rotation_angle_deg: {rotation_angle_deg:.6f}")
            
            # 计算平移距离
            translation_distance = np.linalg.norm(t_deta)
            print(f"translation_distance: {translation_distance:.6f}")
            
            # 判断是否为关键帧
            return rotation_angle_deg > self.kf_angle_deg_ or translation_distance > self.kf_distance_

    def show_frame(self, frame):
        # 显示当前frame的点云
        frame_pcd = self.numpy_to_o3d(frame)
        print(f"frame_pcd 点云数量: {len(frame_pcd.points)}")
        o3d.visualization.draw_geometries([frame_pcd])



    def test_cloud_merge(self):
        fram1 = self.all_frames[0]
        fram2 = self.all_frames[1000]

        fram1_pcd = self.numpy_to_o3d(fram1)
        fram2_pcd = self.numpy_to_o3d(fram2)
        fram1_pcd.paint_uniform_color([1, 0, 0])  # 红色
        fram2_pcd.paint_uniform_color([0, 0, 1])  # 蓝色

        print(f"fram1_pcd 点云数量: {len(fram1_pcd.points)}")
        print(f"fram2_pcd 点云数量: {len(fram2_pcd.points)}")


        # 合并点云
        merged_pcd = fram1_pcd + fram2_pcd
        # 添加一个小的偏移
        offset_vector = np.array([0.1, 0.1, 0.1])
        merged_pcd.translate(offset_vector)
        merged_pcd.paint_uniform_color([0, 1, 0])  # 绿色
        print(f"merged_pcd 点云数量: {len(merged_pcd.points)}")

        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])

        o3d.visualization.draw_geometries(
            [fram1_pcd, fram2_pcd, merged_pcd, axis],
            window_name="Cloud Merge Result",
            width=800,
            height=600
        )

    # def test_key_frame(self, frame):
        # 检查关键帧的判断依据


def main():
    """主函数"""
    # 创建点云播放器实例
    player = PointCloudPlayer()

    # 将rosbag转换为pcd格式的点云 (ROS1)
    # player.load_rosbag_save_local_pcd()

    # 将rosbag2转换为pcd格式的点云 (ROS2) - 示例用法
    player.load_rosbag2_save_local_pcd()

    # 从文件中读取点云帧
    # player.load_all_frame_npy()

    # player.show_frame(player.all_frames[0])


    # 播放点云序列
    # player.play_pointcloud_sequence_npy()

    # 或执行 ICP 配准测试
    # player.test_icp_registration()

    # 测试点云合并
    # player.test_cloud_merge()

    # 继续开始相关的lo的书写
    # player.icp_lo(player.all_frames[0], np.eye(4))

    print("操作完成")


if __name__ == "__main__":
    main()