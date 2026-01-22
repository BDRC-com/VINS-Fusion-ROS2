#!/bin/bash
# jieya.zsh
echo "================================"
echo "VINS-Fusion 非压缩图片版本启动脚本"
echo "================================"

CONFIG_FILE="/home/bluebarry/desktop/wc/vins_fu_ws/VINS-Fusion-ROS2/config/euroc/euroc_stereo_config.yaml"

# # 检查是否有参数
# if [ $# -ne 1 ]; then
#     echo "错误: 需要指定配置文件路径"
#     echo "用法: $0 <配置文件路径>"
#     echo "例如: $0 '/home/bluebarry/desktop/wc/vins_fu_ws/VINS-Fusion-ROS2/config/euroc/euroc_stereo_config.yaml'"
#     exit 1
# fi

CONFIG_FILE=$1
echo "配置文件: $CONFIG_FILE"

# 检查配置文件是否存在
if [ ! -f "$CONFIG_FILE" ]; then
    echo "错误: 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo "1. 检查话题..."
echo "当前活跃的话题:"
ros2 topic list | grep -E "(image_rect|compressed)"

# 更好的检测方法：检查话题是否存在（不要求有数据）
echo "检查压缩图片话题是否存在..."
if ros2 topic info /stereo/left/image_rect/compressed > /dev/null 2>&1; then
    echo "✓ 找到压缩图片话题: /stereo/left/image_rect/compressed"
else
    echo "⚠ 未找到压缩图片话题，但可能正在启动中..."
fi

if ros2 topic info /stereo/right/image_rect/compressed > /dev/null 2>&1; then
    echo "✓ 找到压缩图片话题: /stereo/right/image_rect/compressed"
else
    echo "⚠ 未找到压缩图片话题，但可能正在启动中..."
fi

echo "2. 启动图片转换节点..."
echo "注意: 即使暂时没有数据，转换节点也会启动并等待数据"

# 检查是否已有转换节点
EXISTING_CONVERTERS=$(ros2 node list | grep -c image_republisher)
if [ $EXISTING_CONVERTERS -ge 2 ]; then
    echo "图片转换节点已在运行"
else
    # 启动左目图片转换
    echo "启动左目图片转换..."
    ros2 run image_transport republish compressed raw \
        --ros-args \
        --remap in:=/stereo/left/image_rect/compressed \
        --remap out:=/stereo/left/image_rect &
    LEFT_PID=$!
    
    sleep 1
    
    # 启动右目图片转换
    echo "启动右目图片转换..."
    ros2 run image_transport republish compressed raw \
        --ros-args \
        --remap in:=/stereo/right/image_rect/compressed \
        --remap out:=/stereo/right/image_rect &
    RIGHT_PID=$!
    
    echo "图片转换节点已启动 (PID: 左=$LEFT_PID, 右=$RIGHT_PID)"
fi

echo "等待3秒让转换节点启动..."
sleep 3

echo "3. 检查转换后的话题..."
echo "非压缩图片话题:"
if ros2 topic info /stereo/left/image_rect > /dev/null 2>&1; then
    echo "✓ 左目话题: /stereo/left/image_rect (已创建)"
    echo "  发布者:" $(ros2 topic info /stereo/left/image_rect --verbose 2>/dev/null | grep "Publisher count" | cut -d: -f2)
else
    echo "✗ 左目话题未创建"
fi

if ros2 topic info /stereo/right/image_rect > /dev/null 2>&1; then
    echo "✓ 右目话题: /stereo/right/image_rect (已创建)"
    echo "  发布者:" $(ros2 topic info /stereo/right/image_rect --verbose 2>/dev/null | grep "Publisher count" | cut -d: -f2)
else
    echo "✗ 右目话题未创建"
fi

echo "4. 检查数据流..."
echo "数据流检查（等待5秒看是否有数据）:"

echo "左目压缩图片:"
timeout 5 ros2 topic hz /stereo/left/image_rect/compressed 2>/dev/null || echo "  检查中..."

echo "左目非压缩图片:"
timeout 5 ros2 topic hz /stereo/left/image_rect 2>/dev/null || echo "  检查中..."

echo "右目压缩图片:"
timeout 5 ros2 topic hz /stereo/right/image_rect/compressed 2>/dev/null || echo "  检查中..."

echo "右目非压缩图片:"
timeout 5 ros2 topic hz /stereo/right/image_rect 2>/dev/null || echo "  检查中..."

echo "5. 启动VINS节点..."
echo "使用配置文件: $CONFIG_FILE"
echo "启动VINS-Fusion (非压缩图片版本)..."

# 设置日志格式
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message}"
export RCUTILS_LOGGING_SEVERITY=INFO

# 启动VINS节点
ros2 run vins vins_node "$CONFIG_FILE" --ros-args --log-level vins_estimator:=info
VINS_PID=$!

# 清理函数
cleanup() {
    echo "正在停止所有节点..."
    if [ -n "$LEFT_PID" ] && ps -p $LEFT_PID > /dev/null 2>&1; then
        kill $LEFT_PID 2>/dev/null
    fi
    if [ -n "$RIGHT_PID" ] && ps -p $RIGHT_PID > /dev/null 2>&1; then
        kill $RIGHT_PID 2>/dev/null
    fi
    echo "所有节点已停止"
    exit
}

# 捕获退出信号
trap cleanup SIGINT SIGTERM

wait