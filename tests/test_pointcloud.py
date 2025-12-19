import os
import numpy as np
from src.ch5_my.pointcloud import PointCloudProcessor
from src.ch7.pointcloudy import PointCloudPlayer


def test_bfnn_basic():
    p = PointCloudProcessor()
    point_np = np.array([[1, 1, 1], [2, 2, 2]])
    d = p.bfnn([0, 0, 0], point_np)
    expected = np.array([np.sqrt(3), np.sqrt(12)])
    assert np.allclose(d, expected)


def test_bfnn_cloud_mt():
    p = PointCloudProcessor()
    a = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    b = np.array([[0.1, 0.0, 0.0], [9.9, 0.0, 0.0]])
    matches = p.bfnn_cloud_mt(a, b)
    # matches is list of tuples (idx1, idx2, dist)
    assert len(matches) == len(a)
    idx_map = {m[0]: m[1] for m in matches}
    assert idx_map[0] == 0
    assert idx_map[1] == 1


def test_scan_to_range_image_shape():
    p = PointCloudProcessor()
    pts = np.array([[1.0, 0.0, 1.0], [0.5, 0.1, 1.2], [2.0, 1.0, 1.5]])
    img = p.scan_to_range_image(pts)
    assert img is not None
    assert img.dtype == np.uint8
    # image should have 3 channels
    assert img.ndim == 3 and img.shape[2] == 3


def test_load_rosbag2_save_local_pcd():
    """Test the ROS2 bag loading function."""
    player = PointCloudPlayer()

    # Check if ROS2 support is available
    if not player.ROS2_SUPPORT_AVAILABLE:
        print("Warning: ROS2 support not available, skipping ROS2 test")
        # Just check that the function exists
        assert hasattr(player, 'load_rosbag2_save_local_pcd')
        return

    # Test that the function exists and has correct signature
    assert hasattr(player, 'load_rosbag2_save_local_pcd')

    # This test would normally attempt to load an actual ROS2 bag file.
    # For now, just verify the function exists and has expected parameters.
    import inspect
    sig = inspect.signature(player.load_rosbag2_save_local_pcd)
    params = list(sig.parameters.keys())
    assert 'output_dir' in params
