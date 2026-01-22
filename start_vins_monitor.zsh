#!/bin/zsh

# VINS监控快速启动脚本

echo "=== VINS持久监控启动器 ==="

# 设置颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MONITOR_SCRIPT="$SCRIPT_DIR/vins_persistent_monitor.zsh"
CONFIG_FILE="$SCRIPT_DIR/config/euroc/real_euroc_mono_imu_config_fixed.yaml"

echo -e "${BLUE}脚本目录:${NC} $SCRIPT_DIR"
echo -e "${BLUE}监控脚本:${NC} $MONITOR_SCRIPT"
echo -e "${BLUE}配置文件:${NC} $CONFIG_FILE"
echo ""

# 检查文件
if [[ ! -f "$MONITOR_SCRIPT" ]]; then
    echo -e "${RED}✗ 监控脚本不存在: $MONITOR_SCRIPT${NC}"
    exit 1
fi

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo -e "${RED}✗ 配置文件不存在: $CONFIG_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 文件检查通过${NC}"
echo ""

# 显示选项
echo -e "${YELLOW}选择启动模式:${NC}"
echo "1. 持久监控 (推荐) - 持续运行直到手动停止"
echo "2. 测试模式 - 运行30秒后自动停止"
echo "3. 仅启动VINS节点 - 不进行监控"
echo "4. 查看帮助信息"
echo ""

read -p "请选择 (1-4): " choice

case $choice in
    1)
        echo -e "${GREEN}启动持久监控...${NC}"
        echo -e "${YELLOW}按 Ctrl+C 停止监控${NC}"
        echo ""
        exec "$MONITOR_SCRIPT" "$CONFIG_FILE" --interval 5 --max-restart 10
        ;;
    2)
        echo -e "${GREEN}启动测试模式 (30秒)...${NC}"
        timeout 30s "$MONITOR_SCRIPT" "$CONFIG_FILE" --interval 3
        echo -e "${YELLOW}测试完成${NC}"
        ;;
    3)
        echo -e "${GREEN}仅启动VINS节点...${NC}"
        cd /home/bluebarry/desktop/wc/vins_fu_ws
        source install/setup.zsh
        ros2 run vins vins_node "$CONFIG_FILE"
        ;;
    4)
        echo -e "${GREEN}显示帮助信息...${NC}"
        "$MONITOR_SCRIPT" --help
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac
