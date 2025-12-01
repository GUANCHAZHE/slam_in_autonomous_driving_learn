# -*- coding: utf-8 -*-
"""
性能测试：执行时间、内存消耗等
使用 pytest-benchmark 进行性能基准测试
"""

import pytest
import numpy as np
import time
import sys
from src.ch5_my.pointcloud import PointCloudProcessor


class TestPerformance:
    """性能测试类"""
    
    @pytest.fixture
    def processor(self):
        """创建处理器实例"""
        return PointCloudProcessor()
    
    @pytest.fixture
    def sample_points(self):
        """创建样本点云数据"""
        return np.random.rand(1000, 3) * 10
    
    @pytest.fixture
    def sample_clouds(self):
        """创建两个样本点云"""
        cloud1 = np.random.rand(100, 3) * 10
        cloud2 = np.random.rand(100, 3) * 10
        return cloud1, cloud2
    
    # ==================== 方法 1：pytest-benchmark ====================
    
    def test_bfnn_benchmark(self, benchmark, processor, sample_points):
        """基准测试：测试 bfnn 的执行时间"""
        target = [5, 5, 5]
        
        # benchmark 会自动多次运行并计算统计信息
        result = benchmark(processor.bfnn, target, sample_points)
        
        assert result is not None
        assert len(result) == len(sample_points)
        
        # 输出样例：
        # test_bfnn_benchmark
        #   0.250 ms ±  0.050 ms [1000 calls]
        #   Min:  0.200 ms, Max:  0.350 ms
    
    def test_bfnn_cloud_sequential_benchmark(self, benchmark, processor, sample_clouds):
        """基准测试：序列版本的点云匹配"""
        cloud1, cloud2 = sample_clouds
        
        result = benchmark(processor.bfnn_cloud, cloud1, cloud2)
        
        assert len(result) == len(cloud1)
    
    def test_bfnn_cloud_mt_benchmark(self, benchmark, processor, sample_clouds):
        """基准测试：并行版本的点云匹配"""
        cloud1, cloud2 = sample_clouds
        
        result = benchmark(processor.bfnn_cloud_mt, cloud1, cloud2)
        
        assert len(result) == len(cloud1)
    
    def test_scan_to_range_image_benchmark(self, benchmark, processor, sample_points):
        """基准测试：range image 生成"""
        result = benchmark(processor.scan_to_range_image, sample_points)
        
        assert result is not None
        assert result.dtype == np.uint8
    
    # ==================== 方法 2：手动计时 ====================
    
    def test_bfnn_with_timing(self, processor, sample_points):
        """手动计时方法"""
        target = [5, 5, 5]
        
        # 记录开始时间
        start_time = time.perf_counter()
        
        # 执行操作
        result = processor.bfnn(target, sample_points)
        
        # 记录结束时间
        end_time = time.perf_counter()
        elapsed = (end_time - start_time) * 1000  # 转换为毫秒
        
        # 输出时间信息
        print(f"\n⏱️  执行时间: {elapsed:.3f} ms")
        
        assert result is not None
    
    def test_bfnn_cloud_comparison(self, processor, sample_clouds):
        """比较序列和并行版本的执行时间"""
        cloud1, cloud2 = sample_clouds
        
        # 序列版本
        start = time.perf_counter()
        result_seq = processor.bfnn_cloud(cloud1, cloud2)
        time_seq = (time.perf_counter() - start) * 1000
        
        # 并行版本
        start = time.perf_counter()
        result_mt = processor.bfnn_cloud_mt(cloud1, cloud2)
        time_mt = (time.perf_counter() - start) * 1000
        
        # 计算加速比
        speedup = time_seq / time_mt
        
        print(f"\n📊 性能对比:")
        print(f"   序列版本: {time_seq:.3f} ms")
        print(f"   并行版本: {time_mt:.3f} ms")
        print(f"   加速比:   {speedup:.2f}x")
        
        assert len(result_seq) == len(result_mt)
    
    # ==================== 方法 3：测试参数化性能 ====================
    
    @pytest.mark.parametrize("cloud_size", [10, 50, 100, 500])
    def test_bfnn_cloud_scaling(self, processor, cloud_size):
        """测试不同数据量下的性能"""
        cloud1 = np.random.rand(cloud_size, 3)
        cloud2 = np.random.rand(cloud_size, 3)
        
        start = time.perf_counter()
        result = processor.bfnn_cloud(cloud1, cloud2)
        elapsed = (time.perf_counter() - start) * 1000
        
        print(f"\n📈 云大小: {cloud_size}, 耗时: {elapsed:.3f} ms")
        assert len(result) == cloud_size


# ==================== 内存分析 ====================

class TestMemoryUsage:
    """内存消耗测试"""
    
    def test_bfnn_memory(self):
        """测试内存消耗"""
        processor = PointCloudProcessor()
        
        # 创建大规模数据
        large_cloud = np.random.rand(10000, 3)
        target = [5, 5, 5]
        
        import sys
        
        # 获取内存占用
        initial_memory = sys.getsizeof(large_cloud)
        
        result = processor.bfnn(target, large_cloud)
        
        result_memory = sys.getsizeof(result)
        
        print(f"\n💾 内存消耗:")
        print(f"   输入数据: {initial_memory / 1024:.2f} KB")
        print(f"   输出数据: {result_memory / 1024:.2f} KB")
        
        assert result_memory > 0
    
    @pytest.fixture(autouse=True)
    def measure_memory(self):
        """测试前后测量内存"""
        import os
        
        # 获取初始内存
        initial_size = 0
        
        yield
        
        # 这里可以添加事后处理
        pass
