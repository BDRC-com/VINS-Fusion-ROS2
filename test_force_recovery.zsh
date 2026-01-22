#!/bin/zsh

# 强制触发VINS异常来测试智能恢复功能

echo "=== 强制测试VINS智能恢复功能 ==="

echo "1. 当前VINS状态检查..."
if ros2 node list | grep -q "vins_estimator"; then
    echo "✅ VINS节点运行中"
else
    echo "❌ VINS节点未运行"
    exit 1
fi

echo "2. 强制停止VINS节点来模拟异常..."
pkill -f "vins_node"
sleep 2

echo "3. 检查节点是否已停止..."
if ! ros2 node list | grep -q "vins_estimator"; then
    echo "✅ VINS节点已停止 - 模拟异常成功"
else
    echo "❌ VINS节点仍在运行"
    exit 1
fi

echo "4. 等待监控脚本检测到异常并自动重启..."
echo "监控10秒..."

for i in {1..10}; do
    echo -n "."
    sleep 1
done
echo ""

echo "5. 检查VINS节点是否自动重启..."
if ros2 node list | grep -q "vins_estimator"; then
    echo "✅ VINS节点已自动重启 - 智能监控正常工作"
    
    # 获取新的PID
    new_pid=$(ps aux | grep vins_node | grep -v grep | awk '{print $2}')
    echo "新节点PID: $new_pid"
    
else
    echo "❌ VINS节点未自动重启"
fi

echo "6. 检查监控日志..."
if [[ -f "/tmp/vins_enhanced_monitor.log" ]]; then
    echo "=== 最近的监控日志 ==="
    tail -15 /tmp/vins_enhanced_monitor.log | grep -E "(restart|recovery|异常|退出)" || echo "没有找到相关日志"
fi

echo "7. 检查VINS节点日志..."
if [[ -f "/tmp/vins_node.log" ]]; then
    echo "=== 最近的VINS日志 ==="
    tail -10 /tmp/vins_node.log
fi

echo ""
echo "=== 测试完成 ==="
echo "如果VINS节点自动重启，说明智能监控系统正常工作"
