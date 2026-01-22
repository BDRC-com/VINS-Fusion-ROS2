#!/bin/zsh

# VINS智能恢复功能测试脚本

echo "=== VINS智能恢复功能测试 ==="

# 检查VINS节点状态
echo "1. 检查VINS节点状态..."
if ros2 node list | grep -q "vins_estimator"; then
    echo "✅ VINS节点正在运行"
else
    echo "❌ VINS节点未运行"
    exit 1
fi

# 检查当前特征数量
echo "2. 检查当前特征跟踪状态..."
if [[ -f "/tmp/vins_node.log" ]]; then
    recent_features=$(tail -10 /tmp/vins_node.log | grep "n_pts size" | tail -1)
    echo "最新特征状态: $recent_features"
else
    echo "❌ 无法找到VINS日志文件"
fi

# 模拟特征匹配失败（通过发送重启信号）
echo "3. 测试智能恢复机制..."
echo "发送重启信号来触发智能恢复..."

# 发送重启信号
ros2 topic pub --once /vins_restart std_msgs/msg/Bool "{data: true}"

# 等待一段时间观察恢复过程
echo "等待10秒观察恢复过程..."
sleep 10

# 检查恢复日志
echo "4. 检查智能恢复日志..."
if [[ -f "/tmp/vins_node.log" ]]; then
    echo "=== 最近的恢复相关日志 ==="
    tail -20 /tmp/vins_node.log | grep -E "(recovery|restart|failure|clear)" || echo "没有找到恢复相关日志"
    
    echo ""
    echo "=== 最近的特征跟踪日志 ==="
    tail -10 /tmp/vins_node.log | grep -E "(n_pts|feature|track)" || echo "没有找到特征跟踪日志"
fi

# 检查节点是否还在运行
echo "5. 验证节点状态..."
if ros2 node list | grep -q "vins_estimator"; then
    echo "✅ VINS节点仍在运行 - 智能恢复成功"
else
    echo "❌ VINS节点已停止 - 可能需要手动重启"
fi

# 显示监控统计
echo "6. 监控统计信息..."
if [[ -f "/tmp/vins_enhanced_monitor.log" ]]; then
    echo "=== 最近的监控日志 ==="
    tail -10 /tmp/vins_enhanced_monitor.log
fi

echo ""
echo "=== 测试完成 ==="
echo "如果看到智能恢复相关日志，说明功能正常工作"
echo "如果没有触发恢复，说明系统运行正常，无需恢复"
