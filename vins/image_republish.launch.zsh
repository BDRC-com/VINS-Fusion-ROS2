#!/bin/bash
# 保存为 start_vins.sh
echo "启动图片转换节点..."
source 
# 启动左目图片转换
ros2 run image_transport republish compressed in:=/stereo/left/image_rect/compressed raw out:=/stereo/left/image_rect &
LEFT_PID=$!

# 启动右目图片转换
ros2 run image_transport republish compressed in:=/stereo/right/image_rect/compressed raw out:=/stereo/right/image_rect &
RIGHT_PID=$!

echo "图片转换节点已启动，PID: 左=$LEFT_PID, 右=$RIGHT_PID"
echo "等待2秒让转换节点启动..."
sleep 2

echo "检查话题..."
ros2 topic list | grep -E "(image_rect|cam[0-9])"

echo "启动VINS节点..."
ros2 run vins vins_node '/home/bluebarry/desktop/wc/vins_fu_ws/VINS-Fusion-ROS2/config/euroc/euroc_stereo_config.yaml'
VINS_PID=$!

# 捕获退出信号
trap "echo '正在停止所有节点...'; kill $LEFT_PID $RIGHT_PID $VINS_PID; echo '所有节点已停止'; exit" SIGINT SIGTERM

wait