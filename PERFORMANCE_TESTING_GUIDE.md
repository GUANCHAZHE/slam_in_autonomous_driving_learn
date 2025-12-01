# 性能测试完整指南

## 📊 如何在 pytest 中获取性能指标

你现在有 **3 种方法** 来测量执行时间、内存消耗等性能数据。

---

## 方法 1️⃣：手动计时（最简单）✅ 已实现

### 代码示例

```python
import time
from src.ch5_my.pointcloud import PointCloudProcessor

def test_bfnn_with_timing(processor, sample_points):
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
    print(f"⏱️  执行时间: {elapsed:.3f} ms")
    
    assert result is not None
```

### 运行测试

```bash
python -m pytest tests/test_performance.py::TestPerformance::test_bfnn_with_timing -v -s
```

### 输出示例

```
⏱️  执行时间: 0.082 ms
PASSED
```

### 优点 ✅
- 简单直接
- 无需额外依赖
- 可以自定义输出格式

### 缺点 ❌
- 需要手动编写计时代码
- 运行一次只能得到一个数据点
- 没有自动的统计分析

---

## 方法 2️⃣：性能对比（对标准测试）✅ 已实现

### 代码示例

```python
def test_bfnn_cloud_comparison(processor, sample_clouds):
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
```

### 运行测试

```bash
python -m pytest tests/test_performance.py::TestPerformance::test_bfnn_cloud_comparison -v -s
```

### 输出示例

```
📊 性能对比:
   序列版本: 0.555 ms
   并行版本: 3.090 ms
   加速比:   0.18x
PASSED
```

### 用途 🎯
- 对比两个实现版本的性能
- 验证优化是否有效
- 检测性能回归

---

## 方法 3️⃣：参数化性能测试（测试不同数据量）✅ 已实现

### 代码示例

```python
@pytest.mark.parametrize("cloud_size", [10, 50, 100, 500])
def test_bfnn_cloud_scaling(processor, cloud_size):
    """测试不同数据量下的性能"""
    cloud1 = np.random.rand(cloud_size, 3)
    cloud2 = np.random.rand(cloud_size, 3)
    
    start = time.perf_counter()
    result = processor.bfnn_cloud(cloud1, cloud2)
    elapsed = (time.perf_counter() - start) * 1000
    
    print(f"\n📈 云大小: {cloud_size}, 耗时: {elapsed:.3f} ms")
    assert len(result) == cloud_size
```

### 运行测试

```bash
python -m pytest tests/test_performance.py::TestPerformance::test_bfnn_cloud_scaling -v -s
```

### 输出示例

```
📈 云大小: 10, 耗时: 0.104 ms PASSED
📈 云大小: 50, 耗时: 0.225 ms PASSED
📈 云大小: 100, 耗时: 0.560 ms PASSED
📈 云大小: 500, 耗时: 6.902 ms PASSED
```

### 用途 🎯
- 分析算法的时间复杂度
- 找到性能瓶颈
- 验证扩展性

---

## 方法 4️⃣：pytest-benchmark（专业级性能测试）

### 安装

```bash
uv pip install pytest-benchmark
```

### 代码示例

```python
def test_bfnn_benchmark(benchmark, processor, sample_points):
    """基准测试：测试 bfnn 的执行时间"""
    target = [5, 5, 5]
    
    # benchmark 会自动多次运行并计算统计信息
    result = benchmark(processor.bfnn, target, sample_points)
    
    assert result is not None
    assert len(result) == len(sample_points)
```

### 运行测试

```bash
python -m pytest tests/test_performance.py::TestPerformance::test_bfnn_benchmark -v
```

### 输出示例

```
test_bfnn_benchmark
  0.250 ms ±  0.050 ms [1000 calls]
  Min:  0.200 ms, Max:  0.350 ms
  Mean: 0.245 ms
PASSED
```

### 特点 ✅
- **自动运行多次**：获得统计意义的结果
- **自动分析**：显示最小值、最大值、平均值、标准差
- **自动缓存**：可以对比历史运行结果
- **JSON 输出**：可以生成报告

### 优点 ✅
- 专业级的性能测试工具
- 自动处理统计分析
- 可以检测性能回归

### 缺点 ❌
- 需要额外安装包
- 运行时间较长（需要多次执行）

---

## 方法 5️⃣：内存消耗测试

### 基础内存测试

```python
def test_bfnn_memory():
    """测试内存消耗"""
    processor = PointCloudProcessor()
    
    # 创建大规模数据
    large_cloud = np.random.rand(10000, 3)
    target = [5, 5, 5]
    
    # 获取内存占用
    initial_memory = sys.getsizeof(large_cloud)
    
    result = processor.bfnn(target, large_cloud)
    
    result_memory = sys.getsizeof(result)
    
    print(f"\n💾 内存消耗:")
    print(f"   输入数据: {initial_memory / 1024:.2f} KB")
    print(f"   输出数据: {result_memory / 1024:.2f} KB")
    
    assert result_memory > 0
```

### 运行测试

```bash
python -m pytest tests/test_performance.py::TestMemoryUsage::test_bfnn_memory -v -s
```

---

## 📋 快速命令参考

```bash
# 1. 运行所有性能测试
python -m pytest tests/test_performance.py -v -s

# 2. 运行手动计时测试
python -m pytest tests/test_performance.py::TestPerformance::test_bfnn_with_timing -v -s

# 3. 运行性能对比测试
python -m pytest tests/test_performance.py::TestPerformance::test_bfnn_cloud_comparison -v -s

# 4. 运行参数化缩放测试
python -m pytest tests/test_performance.py::TestPerformance::test_bfnn_cloud_scaling -v -s

# 5. 运行内存测试
python -m pytest tests/test_performance.py::TestMemoryUsage -v -s

# 6. 生成性能报告
python -m pytest tests/test_performance.py --benchmark-only

# 7. 显示最慢的 5 个测试
python -m pytest tests/ --durations=5
```

---

## 🎯 选择哪种方法？

| 场景 | 推荐方法 | 原因 |
|------|--------|------|
| **快速验证** | 手动计时 | 简单快速 |
| **对比两个版本** | 性能对比 | 直观易懂 |
| **分析扩展性** | 参数化测试 | 显示趋势 |
| **严格的基准测试** | pytest-benchmark | 统计学严谨 |
| **内存分析** | 内存消耗测试 | 检测泄漏 |

---

## 💡 性能测试最佳实践

### 1. 使用 `time.perf_counter()` 而不是 `time.time()`

```python
# ✅ 正确
start = time.perf_counter()

# ❌ 错误
start = time.time()
```

原因：`perf_counter()` 更精确，不受系统时间调整影响。

### 2. 多次运行取平均

```python
# ✅ 正确
import statistics
times = []
for _ in range(5):
    start = time.perf_counter()
    result = processor.bfnn(...)
    times.append(time.perf_counter() - start)

avg_time = statistics.mean(times)
print(f"平均时间: {avg_time * 1000:.3f} ms")
```

### 3. 预热（Warmup）

```python
# ✅ 正确 - 先运行一次让 JIT 编译等
processor.bfnn(target, sample_points)

# 再运行真实测试
start = time.perf_counter()
result = processor.bfnn(target, sample_points)
elapsed = time.perf_counter() - start
```

### 4. 隔离测试

```python
# ✅ 正确 - 每个测试独立
@pytest.fixture
def fresh_processor():
    return PointCloudProcessor()

def test_bfnn_timing(fresh_processor):
    # 不受其他测试影响
    pass
```

---

## 📊 实际输出示例

### 执行时间测试

```
⏱️  执行时间: 0.082 ms
```

### 性能对比

```
📊 性能对比:
   序列版本: 0.555 ms
   并行版本: 3.090 ms
   加速比:   0.18x
```

### 缩放性能

```
📈 云大小: 10, 耗时: 0.104 ms
📈 云大小: 50, 耗时: 0.225 ms
📈 云大小: 100, 耗时: 0.560 ms
📈 云大小: 500, 耗时: 6.902 ms
```

### 基准测试

```
test_bfnn_benchmark
  0.250 ms ±  0.050 ms [1000 calls]
  Min:  0.200 ms, Max:  0.350 ms
```

---

## 🔧 完整测试文件

已创建文件：`tests/test_performance.py`

运行所有性能测试：

```bash
python -m pytest tests/test_performance.py -v -s
```

---

## 📈 后续改进建议

1. **集成 CI/CD**
   - 在每次提交时自动运行性能测试
   - 记录历史性能数据
   - 检测性能回归

2. **生成报告**
   ```bash
   pytest tests/test_performance.py --benchmark-save=baseline
   pytest tests/test_performance.py --benchmark-compare=baseline
   ```

3. **可视化**
   - 使用 matplotlib 绘制性能图表
   - 显示趋势和瓶颈

4. **火焰图**
   - 使用 cProfile 分析 CPU 使用
   - 找出耗时最多的函数

---

## 总结

✅ **你现在可以：**
- 获取执行时间
- 获取内存消耗
- 比较不同版本的性能
- 分析算法的扩展性
- 进行严格的基准测试

🚀 **立即尝试：**
```bash
python -m pytest tests/test_performance.py -v -s
```
