#!/bin/bash

# CS30 自定义数据集验证脚本
# 用于验证修改是否成功

set -e

echo "=========================================="
echo "CS30 自定义数据集验证脚本"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 项目根目录
PROJECT_ROOT="/home/keyirobot/Desktop/qixing_ws/learn/slam_in_autonomous_driving"

# 检查项目目录
echo -e "${YELLOW}[1/6] 检查项目目录...${NC}"
if [ -d "$PROJECT_ROOT" ]; then
    echo -e "${GREEN}✓ 项目目录存在${NC}"
else
    echo -e "${RED}✗ 项目目录不存在: $PROJECT_ROOT${NC}"
    exit 1
fi
echo ""

# 检查源文件
echo -e "${YELLOW}[2/6] 检查源文件...${NC}"
SOURCE_FILE="$PROJECT_ROOT/src/ch7/test/test_call_back.cc"
if [ -f "$SOURCE_FILE" ]; then
    echo -e "${GREEN}✓ 源文件存在${NC}"
    
    # 检查是否包含新增函数
    if grep -q "loadCustomRosbagImuData" "$SOURCE_FILE"; then
        echo -e "${GREEN}✓ 发现 loadCustomRosbagImuData 函数${NC}"
    else
        echo -e "${RED}✗ 未找到 loadCustomRosbagImuData 函数${NC}"
        exit 1
    fi
    
    if grep -q "loadCustomRosbagPointCloudData" "$SOURCE_FILE"; then
        echo -e "${GREEN}✓ 发现 loadCustomRosbagPointCloudData 函数${NC}"
    else
        echo -e "${RED}✗ 未找到 loadCustomRosbagPointCloudData 函数${NC}"
        exit 1
    fi
    
    if grep -q "test_Ndt_LO_CustomDataset" "$SOURCE_FILE"; then
        echo -e "${GREEN}✓ 发现 test_Ndt_LO_CustomDataset 函数${NC}"
    else
        echo -e "${RED}✗ 未找到 test_Ndt_LO_CustomDataset 函数${NC}"
        exit 1
    fi
    
    if grep -q "FLAGS_use_custom_dataset" "$SOURCE_FILE"; then
        echo -e "${GREEN}✓ 发现 use_custom_dataset 参数${NC}"
    else
        echo -e "${RED}✗ 未找到 use_custom_dataset 参数${NC}"
        exit 1
    fi
else
    echo -e "${RED}✗ 源文件不存在: $SOURCE_FILE${NC}"
    exit 1
fi
echo ""

# 检查数据集文件
echo -e "${YELLOW}[3/6] 检查CS30数据集文件...${NC}"
BAG_FILE="$PROJECT_ROOT/dataset/sad/ulhk/cs30_ros1_converted.bag"
if [ -f "$BAG_FILE" ]; then
    echo -e "${GREEN}✓ Rosbag文件存在${NC}"
    SIZE=$(ls -lh "$BAG_FILE" | awk '{print $5}')
    echo "  文件大小: $SIZE"
else
    echo -e "${YELLOW}⚠ Rosbag文件不存在 (可选): $BAG_FILE${NC}"
    echo "  提示: 如果文件不存在，请从指定路径复制或下载"
fi
echo ""

# 检查构建目录
echo -e "${YELLOW}[4/6] 检查构建目录...${NC}"
BUILD_DIR="$PROJECT_ROOT/build"
if [ -d "$BUILD_DIR" ]; then
    echo -e "${GREEN}✓ 构建目录存在${NC}"
else
    echo -e "${YELLOW}⚠ 构建目录不存在，将在编译时创建${NC}"
fi
echo ""

# 检查编译
echo -e "${YELLOW}[5/6] 检查编译状态...${NC}"
BINARY="$PROJECT_ROOT/bin/test_call_back"
if [ -f "$BINARY" ]; then
    echo -e "${GREEN}✓ 二进制文件存在${NC}"
    echo "  位置: $BINARY"
else
    echo -e "${YELLOW}⚠ 二进制文件未编译，需要编译${NC}"
    echo "  编译命令: cd $BUILD_DIR && cmake .. && make test_call_back -j4"
fi
echo ""

# 检查文档
echo -e "${YELLOW}[6/6] 检查文档文件...${NC}"
DOCS=(
    "CUSTOM_DATASET_INTEGRATION_GUIDE.md"
    "CUSTOM_DATASET_GUIDE.md"
    "CS30_QUICK_START.md"
    "CODE_CHANGES_SUMMARY.md"
)

for doc in "${DOCS[@]}"; do
    DOC_FILE="$PROJECT_ROOT/$doc"
    if [ -f "$DOC_FILE" ]; then
        echo -e "${GREEN}✓ $doc${NC}"
    else
        echo -e "${YELLOW}⚠ $doc 不存在${NC}"
    fi
done
echo ""

# 总结
echo "=========================================="
echo "验证完成！"
echo "=========================================="
echo ""
echo "下一步操作:"
echo "1. 编译程序:"
echo "   cd $BUILD_DIR"
echo "   cmake .."
echo "   make test_call_back -j4"
echo ""
echo "2. 运行程序:"
echo "   cd $PROJECT_ROOT"
echo "   ./bin/test_call_back --use_custom_dataset=true"
echo ""
echo "3. 查看输出:"
echo "   output: ./dataset/sad/ulhk/cs30_output_cloud.pcd"
echo ""
echo "更多信息请查看: $PROJECT_ROOT/CS30_QUICK_START.md"
echo ""
