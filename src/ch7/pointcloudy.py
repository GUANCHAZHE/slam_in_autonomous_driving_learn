import time
import open3d as o3d
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import os
import glob
import copy  # 用于深拷贝，防止修改原始数据


class PointCloudPlayer:
    """点云加载、播放和配准工具类"""

    def __init__(self, frame_path=None, bag_path=None, topic_name=None):
        """
        初始化点云播放器
        
        Args:
            frame_path: 点云帧文件夹路径
            bag_path: rosbag 文件路径
            topic_name: rosbag 中的话题名称
        """
        self.frame_path = frame_path or "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/frames"
        self.bag_path = bag_path or "/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving/dataset/sad/ulhk/test2.bag"
        self.topic_name = topic_name or "/velodyne_points_0"
        self.all_frames = []

    @staticmethod
    def numpy_to_o3d(points_np):
        """将 Numpy (N,3) 转为 Open3D PointCloud"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)
        return pcd

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
            
        file_paths = sorted(glob.glob(f"{frame_path}/*.npy"))

        if not file_paths:
            print("未找到文件，请检查路径")
            return []

        # 预加载数据到内存 (解决卡顿问题)
        print("正在预加载所有点云数据，请稍候...")
        self.all_frames = []
        for f in file_paths:
            self.all_frames.append(np.load(f))
        print(f"预加载完成，共 {len(self.all_frames)} 帧")
        
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

    @staticmethod
    def icp_registration(source, target, voxel_size=1.0):
        """
        执行 ICP 点云配准
        
        Args:
            source: 输入点云 (o3d.geometry.PointCloud)
            target: 地图/目标点云 (o3d.geometry.PointCloud)
            voxel_size: 体素大小（越大越稳，越小越精细）
            
        Returns:
            配准结果对象
        """
        # 下采样
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)

        # 设置初始估计
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

    def test_icp_registration(self, all_frames=None, voxel_size=0.5):
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
        source = self.numpy_to_o3d(all_frames[0])
        target = self.numpy_to_o3d(all_frames[1])

        # 为了可视化效果明显，先给点云上色
        source.paint_uniform_color([1, 0.706, 0])  # 黄色
        target.paint_uniform_color([0, 0.651, 0.929])  # 蓝色

        print("Starting ICP registration...")
        # 调用配准函数
        result = self.icp_registration(source, target, voxel_size=voxel_size)
        T = result.transformation

        # 提取旋转矩阵 R (前3行，前3列)
        R = T[:3, :3]
        # 提取平移向量 t (前3行，第4列)
        t = T[:3, 3]

        print("\nTransformation Matrix (4x4):")
        print(result.transformation)
        print("\n--- SE3 Components ---")
        print("Rotation Matrix (R):\n", R)
        print("Translation Vector (t):\n", t)
        print(f"Fitness 重叠度: {result.fitness}, RMSE 均方根误差: {result.inlier_rmse}")

        # 可视化
        # 将 source 点云变换到 target 坐标系下
        source_trans = copy.deepcopy(source)
        source_trans.transform(result.transformation)

        # 变换后的设为红色，目标设为绿色
        source_trans.paint_uniform_color([1, 0, 0])
        target.paint_uniform_color([0, 1, 0])

        # 创建坐标轴看原点
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])

        o3d.visualization.draw_geometries(
            [source_trans, target, axis],
            window_name="ICP Registration Result",
            width=800,
            height=600
        )

        return result

    def icp_lo(self, ky_distance = 0.5, ky_angel = 30):
        # 整体的逻辑为
        # 实时计算当前帧和上一帧的之间的icp结果T
        # 将当前帧的点云变换到世界坐标系下，
        # 如果是关键帧将当前帧的世界坐标系和上一帧拼接起来，得到全局的地图
def main():
    """主函数"""
    # 创建点云播放器实例
    player = PointCloudPlayer()

    # 从文件中读取点云帧
    player.load_all_frame_npy()

    # 播放点云序列
    player.play_pointcloud_sequence_npy()

    # 或执行 ICP 配准测试
    # player.test_icp_registration()

    print("操作完成")


if __name__ == "__main__":
    main()
    