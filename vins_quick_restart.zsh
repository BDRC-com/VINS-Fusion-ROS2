#!/bin/zsh

# VINS快速重启脚本
# 当特征匹配失败时快速重启VINS

VINS_NODE="vins_estimator"
RESTART_TOPIC="/vins_restart"

echo "正在重启VINS节点..."

# 方法1: 发送重启信号
echo "发送重启信号..."
ros2 topic pub --once "$RESTART_TOPIC" std_msgs/msg/Bool "{data: true}" 2>/dev/null

# 等待2秒
sleep 2

# 方法2: 如果信号无效，强制重启
if ! ros2 node list | grep -q "$VINS_NODE"; then
    echo "节点已停止，重新启动..."
    
    # 杀死所有vins相关进程
    pkill -f "vins_node" 2>/dev/null || true
    
    # 重新启动（需要提供配置文件）
    if [[ $# -eq 1 ]]; then
        cd /home/bluebarry/desktop/wc/vins_fu_ws
        source install/setup.zsh 2>/dev/null || source install/setup.bash 2>/dev/null
        ros2 run vins vins_node "$1" &
        echo "VINS节点已重新启动，配置文件: $1"
    else
        echo "请提供配置文件路径: $0 <config_file>"
    fi
else
    echo "VINS节点重启信号已发送"
fi
