# Pytest 使用指南

## 项目测试文件位置与说明

测试文件位于 `tests/test_pointcloud.py`，包含 3 个关键功能的单元测试。

---

## Pytest 基础概念

### 什么是 Pytest？
Pytest 是 Python 最流行的测试框架，提供简洁的语法、强大的断言和灵活的 fixture 系统。相比 unittest，pytest 更轻量级、易读性更高。

### 为什么使用 Pytest？
- ✅ **简洁语法**：只需写普通函数和 `assert` 语句，无需继承 TestCase 类
- ✅ **自动发现**：自动查找并运行以 `test_` 开头的函数
- ✅ **详细错误**：提供非常详尽的失败信息，便于快速定位问题
- ✅ **插件生态**：支持众多插件扩展（如 coverage、mock 等）
- ✅ **Fixture 系统**：强大的测试前后置处理机制

---

## 项目中的测试代码

### 文件结构
```python
# tests/test_pointcloud.py

def test_bfnn_basic():
    """测试暴力最近邻基本功能"""
    p = PointCloudProcessor()
    point_np = np.array([[1, 1, 1], [2, 2, 2]])
    d = p.bfnn([0, 0, 0], point_np)
    expected = np.array([np.sqrt(3), np.sqrt(12)])
    assert np.allclose(d, expected)

def test_bfnn_cloud_mt():
    """测试多线程最近邻匹配"""
    # ...

def test_scan_to_range_image_shape():
    """测试 range image 生成的形状"""
    # ...
```

### 测试说明

| 测试函数 | 功能 | 验证内容 |
|---------|------|--------|
| `test_bfnn_basic` | 暴力最近邻算法 | 计算点到目标的距离是否正确 |
| `test_bfnn_cloud_mt` | 多线程点云匹配 | 并行返回的 (idx1, idx2, dist) 元组是否正确匹配 |
| `test_scan_to_range_image_shape` | 激光 range image 生成 | 输出图像的形状和数据类型是否符合预期 |

---

## 如何运行测试

### 1. **运行所有测试**（最常用）
```bash
python -m pytest tests/test_pointcloud.py -v
```
- `-v` 或 `--verbose`：显示详细输出，列出每个测试的名称和结果

### 2. **运行单个测试**
```bash
python -m pytest tests/test_pointcloud.py::test_bfnn_basic -v
```
- 只运行 `test_bfnn_basic` 函数

### 3. **运行包含特定关键词的测试**
```bash
python -m pytest tests/test_pointcloud.py -k "mt" -v
```
- 只运行函数名包含 "mt" 的测试（即 `test_bfnn_cloud_mt`）

### 4. **显示打印输出**
```bash
python -m pytest tests/test_pointcloud.py -v -s
```
- `-s` 或 `--capture=no`：显示测试中的 print 输出

### 5. **生成覆盖率报告**（需要 pytest-cov）
```bash
python -m pytest tests/test_pointcloud.py --cov=src.ch5_my.pointcloud --cov-report=html
```
- 生成 HTML 格式的代码覆盖率报告到 `htmlcov/` 目录

### 6. **显示最慢的测试**
```bash
python -m pytest tests/test_pointcloud.py -v --durations=3
```
- 显示耗时最长的 3 个测试

### 7. **运行整个项目的所有测试**
```bash
python -m pytest tests/ -v
```
- 运行 `tests/` 目录下的所有测试文件

---

## Pytest 与 unittest 的对比

| 特性 | Pytest | unittest |
|------|--------|----------|
| 语法复杂度 | 简单（普通函数 + assert） | 复杂（需继承 TestCase） |
| 自动发现 | ✅ 自动 | ❌ 需要按规则命名 |
| 断言方式 | `assert` 表达式 | `self.assertEqual()` 等方法 |
| 前后置 | Fixture（灵活） | setUp/tearDown（刻板） |
| 测试失败输出 | 非常详细 | 相对简洁 |
| 学习曲线 | 平缓 | 陡峭 |

**本项目选用 Pytest 的原因**：代码简洁、易于维护、社区广泛支持。

---

## Pytest 配置文件

项目已有 `pyproject.toml` 配置，示例：

```toml
[tool.pytest.ini_options]
minversion = "8.0"
testpaths = ["tests"]
python_files = ["test_*.py", "*_test.py"]
python_classes = ["Test*"]
python_functions = ["test_*"]
addopts = "-v"
```

这些配置告诉 pytest：
- 测试位于 `tests/` 目录
- 自动查找以 `test_` 开头的函数
- 默认显示详细输出

---

## 本项目测试结果

最近一次运行结果（✅ 全部通过）：

```
============================= test session starts ==============================
platform linux -- Python 3.9.24, pytest-8.4.2, pluggy-1.6.0
collected 3 items

tests/test_pointcloud.py::test_bfnn_basic PASSED                        [ 33%]
tests/test_pointcloud.py::test_bfnn_cloud_mt PASSED                     [ 66%]
tests/test_pointcloud.py::test_scan_to_range_image_shape PASSED         [100%]

============================== 3 passed in 1.39s ===============================
```

---

## 常见使用场景

### 场景 1：开发新功能后立即测试
```bash
# 运行相关测试确保没有破坏现有功能
python -m pytest tests/test_pointcloud.py::test_bfnn_basic -v
```

### 场景 2：修复 Bug 前后对比
```bash
# 修复前运行测试，看失败的测试
python -m pytest tests/test_pointcloud.py -v
# 修复后再运行，确认问题解决
python -m pytest tests/test_pointcloud.py -v
```

### 场景 3：CI/CD 流程中自动测试
```bash
# 在 GitHub Actions / GitLab CI 中运行
python -m pytest tests/ -v --tb=short
```

### 场景 4：性能分析
```bash
# 找出哪些测试最耗时
python -m pytest tests/test_pointcloud.py --durations=0
```

---

## 如何添加新测试

### 示例：测试 `pcd_to_bird` 方法

```python
def test_pcd_to_bird_output():
    """测试鸟瞰图生成"""
    p = PointCloudProcessor()
    # 创建简单的点云数据
    points = np.array([
        [0.0, 0.0, 0.5],
        [1.0, 1.0, 1.0],
        [2.0, 2.0, 1.5]
    ])
    
    # 调用被测方法
    img = p.pcd_to_bird(points, resolution=0.1)
    
    # 断言检查结果
    assert img is not None
    assert img.dtype == np.uint8
    assert img.ndim == 3
    assert img.shape[2] == 3  # BGR 三通道
```

只需遵循 `test_*` 命名规则，pytest 会自动发现并运行。

---

## 关键断言（Assert）用法

```python
# 基本比较
assert x == y           # 相等
assert x != y           # 不相等
assert x > y            # 大于

# 容器检查
assert item in container
assert len(list) == 3

# NumPy 数组比较
import numpy as np
np.testing.assert_array_equal(a, b)      # 精确相等
np.testing.assert_allclose(a, b, rtol=1e-5)  # 浮点数近似相等

# 异常检查
with pytest.raises(ValueError):
    some_function()
```

---

## 总结

**快速命令速查表**：

| 命令 | 说明 |
|------|------|
| `python -m pytest` | 运行所有测试 |
| `python -m pytest -v` | 详细输出 |
| `python -m pytest -s` | 显示 print 输出 |
| `python -m pytest -k "pattern"` | 筛选测试 |
| `python -m pytest --collect-only` | 仅列出测试，不运行 |
| `python -m pytest -x` | 首次失败后停止 |
| `python -m pytest --maxfail=2` | 失败 2 次后停止 |

使用 pytest 进行测试驱动开发（TDD），可以显著提高代码质量和开发效率！ 🚀
