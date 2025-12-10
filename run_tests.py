#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pytest 快速启动脚本
运行此脚本可了解所有常见的 pytest 用法

用法: python run_tests.py
"""

import subprocess
import sys


def run_command(description, command):
    """运行命令并显示结果"""
    print(f"\n{'='*70}")
    print(f"📌 {description}")
    print(f"{'='*70}")
    print(f"🔧 命令: {command}")
    print(f"{'-'*70}")
    result = subprocess.run(command, shell=True)
    if result.returncode != 0:
        print(f"❌ 命令失败")
    else:
        print(f"✅ 命令成功")
    return result.returncode == 0


def main():
    """演示各种 pytest 用法"""
    
    print("\n" + "="*70)
    print("🧪 Pytest 使用演示")
    print("="*70)
    
    commands = [
        (
            "1️⃣ 运行所有测试（最常用）",
            "python -m pytest tests/test_pointcloud.py -v"
        ),
        (
            "2️⃣ 运行单个测试",
            "python -m pytest tests/test_pointcloud.py::test_bfnn_basic -v"
        ),
        (
            "3️⃣ 按关键词筛选测试（如 'mt' 表示多线程）",
            "python -m pytest tests/test_pointcloud.py -k 'mt' -v"
        ),
        (
            "4️⃣ 显示打印输出（-s）",
            "python -m pytest tests/test_pointcloud.py -v -s"
        ),
        (
            "5️⃣ 显示测试耗时（--durations）",
            "python -m pytest tests/test_pointcloud.py -v --durations=3"
        ),
        (
            "6️⃣ 列出所有测试但不运行（--collect-only）",
            "python -m pytest tests/test_pointcloud.py --collect-only"
        ),
    ]
    
    for i, (description, command) in enumerate(commands, 1):
        run_command(description, command)
        if i < len(commands):
            input("\n⏸️  按 Enter 继续下一条命令...")
    
    print("\n" + "="*70)
    print("✨ 演示完成！")
    print("="*70)
    print("\n📚 更多信息请查看:")
    print("   - PYTEST_GUIDE.md - 完整指南")
    print("   - PYTEST_QUICK_REFERENCE.md - 快速参考")
    print("   - REFACTOR_SUMMARY.md - 重构总结")


if __name__ == "__main__":
    main()
