#!/bin/zsh

# VINS-Fusion-ROS2 持久监控脚本
# 即使没有数据源也会持续运行，直到手动终止

# 导入轨迹检测模块
source "$(dirname "$0")/trajectory_detector.zsh"

# 配置参数
SCRIPT_NAME=$(basename "$0")
VINS_NODE_NAME="vins_node"
LOG_FILE="/tmp/vins_persistent_monitor.log"
MONITOR_INTERVAL=5  # 监控间隔（秒）
CONFIG_FILE=""
MAX_RESTART_ATTEMPTS=10  # 最大重启尝试次数
RESTART_COOLDOWN=30  # 重启冷却时间（秒）

# 统计变量
RECOVERY_COUNT=0
FAILURE_COUNT=0
LAST_RESTART_TIME=0
VINS_PID=""

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log_debug() {
    echo -e "${BLUE}[DEBUG]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# 检查ROS2环境
check_ros2_env() {
    if [[ -z "$ROS_DISTRO" ]]; then
        log_error "ROS2环境未设置，请先source ROS2 setup文件"
        exit 1
    fi
    log_info "ROS2环境检查通过: $ROS_DISTRO"
}

# 检查VINS节点是否运行
check_vins_node() {
    if [[ -n "$VINS_PID" ]] && kill -0 "$VINS_PID" 2>/dev/null; then
        return 0
    fi
    
    # 检查ROS2节点列表
    local node_running=$(ros2 node list 2>/dev/null | grep -c "vins_node" || true)
    if [[ $node_running -gt 0 ]]; then
        return 0
    fi
    
    return 1
}

# 启动VINS节点
start_vins_node() {
    if [[ -z "$CONFIG_FILE" ]]; then
        log_error "配置文件未设置"
        return 1
    fi
    
    if [[ ! -f "$CONFIG_FILE" ]]; then
        log_error "配置文件不存在: $CONFIG_FILE"
        return 1
    fi
    
    # 检查重启频率
    local current_time=$(date +%s)
    if [[ $((current_time - LAST_RESTART_TIME)) -lt $RESTART_COOLDOWN ]]; then
        log_warn "重启冷却中，等待 $RESTART_COOLDOWN 秒..."
        return 1
    fi
    
    log_info "启动VINS节点，配置文件: $CONFIG_FILE"
    
    # 切换到VINS工作空间
    cd /home/bluebarry/desktop/wc/vins_fu_ws
    
    # Source工作空间
    if [[ -f "install/setup.zsh" ]]; then
        source install/setup.zsh
    elif [[ -f "install/setup.bash" ]]; then
        source install/setup.bash
    fi
    
    # 停止现有节点
    pkill -f "vins_node" 2>/dev/null || true
    sleep 2
    
    # 启动VINS节点（后台运行）
    nohup ros2 run vins vins_node "$CONFIG_FILE" > /tmp/vins_node.log 2>&1 &
    VINS_PID=$!
    
    log_info "VINS节点已启动，PID: $VINS_PID"
    echo $VINS_PID > /tmp/vins_node.pid
    
    # 更新重启时间
    LAST_RESTART_TIME=$current_time
    ((RECOVERY_COUNT++))
    
    # 等待节点启动
    sleep 5
    if check_vins_node; then
        log_info "VINS节点启动成功"
        return 0
    else
        log_error "VINS节点启动失败"
        ((FAILURE_COUNT++))
        return 1
    fi
}

# 获取VINS节点状态
get_vins_status() {
    local node_status="未知"
    
    if check_vins_node; then
        node_status="运行中"
        
        # 检查日志中的状态
        if [[ -f "/tmp/vins_node.log" ]]; then
            local last_lines=$(tail -10 /tmp/vins_node.log)
            
            if echo "$last_lines" | grep -q "waiting for image and imu"; then
                node_status="$node_status (等待数据)"
            elif echo "$last_lines" | grep -q "Feature tracking failed"; then
                node_status="$node_status (特征跟踪失败)"
            elif echo "$last_lines" | grep -q "Large position jump"; then
                node_status="$node_status (位置跳跃)"
            fi
        fi
    else
        node_status="未运行"
    fi
    
    echo "$node_status"
}

# 持久监控循环
persistent_monitor() {
    log_info "启动持久VINS节点监控..."
    log_info "监控间隔: ${MONITOR_INTERVAL}秒"
    log_info "最大重启尝试: $MAX_RESTART_ATTEMPTS"
    log_info "重启冷却时间: ${RESTART_COOLDOWN}秒"
    log_info "按 Ctrl+C 停止监控"
    
    while true; do
        local current_time=$(date +%s)
        local node_status=$(get_vins_status)
        
        log_debug "节点状态: $node_status"
        
        # 检查节点是否还在运行
        if ! check_vins_node; then
            log_warn "VINS节点未运行，尝试重启..."
            
            if [[ $RECOVERY_COUNT -ge $MAX_RESTART_ATTEMPTS ]]; then
                log_error "已达到最大重启尝试次数 ($MAX_RESTART_ATTEMPTS)，停止自动重启"
                log_info "节点将保持未运行状态，直到手动干预"
            else
                start_vins_node
            fi
        fi
        
        # 定期报告统计信息
        if (( current_time % 60 == 0 )); then
            log_info "监控统计 - 重启次数:$RECOVERY_COUNT 失败次数:$FAILURE_COUNT 当前状态:$node_status"
        fi
        
        # 显示状态信息
        echo -e "\r${GREEN}[$(date '+%H:%M:%S')]${NC} 状态: $node_status | 重启: $RECOVERY_COUNT | 失败: $FAILURE_COUNT" | tr -d '\n'
        
        sleep $MONITOR_INTERVAL
    done
}

# 信号处理
cleanup() {
    echo ""
    log_info "收到退出信号，正在清理..."
    
    if [[ -n "$VINS_PID" ]]; then
        log_info "停止VINS节点 (PID: $VINS_PID)"
        kill "$VINS_PID" 2>/dev/null || true
    fi
    
    pkill -f "vins_node" 2>/dev/null || true
    
    log_info "监控统计 - 总重启次数:$RECOVERY_COUNT 总失败次数:$FAILURE_COUNT"
    log_info "持久监控已停止"
    exit 0
}

# 显示帮助信息
show_help() {
    echo "VINS-Fusion-ROS2 持久监控脚本"
    echo ""
    echo "用法: $SCRIPT_NAME <config_file> [options]"
    echo ""
    echo "选项:"
    echo "  --interval N       设置监控间隔秒数 (默认: $MONITOR_INTERVAL)"
    echo "  --max-restart N   设置最大重启尝试次数 (默认: $MAX_RESTART_ATTEMPTS)"
    echo "  --cooldown N      设置重启冷却时间秒数 (默认: $RESTART_COOLDOWN)"
    echo "  --help|-h         显示此帮助信息"
    echo ""
    echo "特性:"
    echo "  - 持续运行，直到手动终止 (Ctrl+C)"
    echo "  - 智能重启机制，防止无限重启"
    echo "  - 实时状态显示"
    echo "  - 详细日志记录"
    echo ""
    echo "示例:"
    echo "  $SCRIPT_NAME config/euroc/real_euroc_mono_imu_config_fixed.yaml"
    echo "  $SCRIPT_NAME config.yaml --interval 10 --max-restart 5"
    echo ""
}

# 主函数
main() {
    # 检查参数
    if [[ $# -lt 1 ]]; then
        show_help
        exit 1
    fi
    
    # 先检查是否是帮助选项
    if [[ "$1" == "--help" || "$1" == "-h" ]]; then
        show_help
        exit 0
    fi
    
    # 处理配置文件路径
    if [[ "$1" == /* ]]; then
        CONFIG_FILE="$1"
    else
        CONFIG_FILE="$(cd "$(dirname "$0")" && pwd)/$1"
    fi
    shift
    
    # 解析选项
    while [[ $# -gt 0 ]]; do
        case $1 in
            --interval)
                MONITOR_INTERVAL="$2"
                shift 2
                ;;
            --max-restart)
                MAX_RESTART_ATTEMPTS="$2"
                shift 2
                ;;
            --cooldown)
                RESTART_COOLDOWN="$2"
                shift 2
                ;;
            *)
                log_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 设置信号处理
    trap cleanup SIGINT SIGTERM
    
    # 检查环境
    check_ros2_env
    
    # 创建日志目录
    mkdir -p "$(dirname "$LOG_FILE")"
    
    log_info "VINS持久监控脚本启动"
    log_info "配置文件: $CONFIG_FILE"
    log_info "日志文件: $LOG_FILE"
    
    # 首次启动VINS节点
    if ! start_vins_node; then
        log_warn "首次启动失败，但监控将继续"
    fi
    
    # 开始持久监控
    persistent_monitor
}

# 运行主函数
main "$@"
