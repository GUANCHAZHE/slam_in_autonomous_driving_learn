import os
import numpy as np
from src.ch5_my.pointcloud import PointCloudProcessor


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
