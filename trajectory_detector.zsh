#!/bin/zsh

# VINS轨迹异常检测模块
# 检测轨迹跳跃、漂移和异常

# 轨迹异常检测参数
TRAJECTORY_JUMP_THRESHOLD=2.0      # 位置跳跃阈值(米)
TRAJECTORY_ANGLE_THRESHOLD=30.0    # 角度跳跃阈值(度)
TRAJECTORY_STUCK_TIME=5.0          # 轨迹停止时间阈值(秒)
TRAJECTORY_DRIFT_THRESHOLD=0.5     # 漂移速度阈值(米/秒)

# 轨迹状态变量
last_pose_time=0
last_position_x=0
last_position_y=0
last_position_z=0
last_orientation_w=1
trajectory_stuck_start_time=0
trajectory_jump_count=0
trajectory_total_distance=0

# 检测轨迹异常
detect_trajectory_anomaly() {
    local current_time=$(date +%s.%N)
    
    # 获取最新的位姿数据
    local odom_data=$(timeout 2 ros2 topic echo /odometry --once 2>/dev/null)
    
    if [[ -z "$odom_data" ]]; then
        echo "WARN:NoOdometryData"
        return 1
    fi
    
    # 解析位置数据 - 使用更精确的解析方法
    local pos_x=$(echo "$odom_data" | grep -A 20 "position:" | grep "x:" | head -1 | awk '{print $2}')
    local pos_y=$(echo "$odom_data" | grep -A 20 "position:" | grep "y:" | head -1 | awk '{print $2}')
    local pos_z=$(echo "$odom_data" | grep -A 20 "position:" | grep "z:" | head -1 | awk '{print $2}')
    
    # 解析姿态数据
    local ori_w=$(echo "$odom_data" | grep -A 20 "orientation:" | grep "w:" | head -1 | awk '{print $2}')
    
    # 检查数据有效性
    if [[ -z "$pos_x" || -z "$pos_y" || -z "$pos_z" || -z "$ori_w" ]]; then
        echo "WARN:InvalidPoseData"
        return 1
    fi
    
    # 清理数据中的特殊字符
    pos_x=$(echo "$pos_x" | sed 's/,$//' | sed 's/^ *//' | sed 's/ *$//')
    pos_y=$(echo "$pos_y" | sed 's/,$//' | sed 's/^ *//' | sed 's/ *$//')
    pos_z=$(echo "$pos_z" | sed 's/,$//' | sed 's/^ *//' | sed 's/ *$//')
    ori_w=$(echo "$ori_w" | sed 's/,$//' | sed 's/^ *//' | sed 's/ *$//')
    
    # 检查数值是否有效（允许负数和小数）
    if ! echo "$pos_x" | grep -qE '^-?[0-9]*\.?[0-9]+$' || \
       ! echo "$pos_y" | grep -qE '^-?[0-9]*\.?[0-9]+$' || \
       ! echo "$pos_z" | grep -qE '^-?[0-9]*\.?[0-9]+$' || \
       ! echo "$ori_w" | grep -qE '^-?[0-9]*\.?[0-9]+$'; then
        echo "WARN:InvalidNumericValues: x=$pos_x y=$pos_y z=$pos_z w=$ori_w"
        return 1
    fi
    
    # 如果是第一次接收数据，初始化并返回
    if [[ $last_pose_time -eq 0 ]]; then
        last_pose_time=$current_time
        last_position_x=$pos_x
        last_position_y=$pos_y
        last_position_z=$pos_z
        last_orientation_w=$ori_w
        echo "INFO:TrajectoryInit"
        return 0
    fi
    
    # 计算时间差
    local time_diff=$(echo "$current_time - $last_pose_time" | bc -l)
    
    # 检查轨迹是否停止
    if [[ $(echo "$time_diff > $TRAJECTORY_STUCK_TIME" | bc -l) -eq 1 ]]; then
        echo "ERROR:TrajectoryStuck:${time_diff}s"
        trajectory_stuck_start_time=$current_time
        return 2
    fi
    
    # 计算位置变化
    local dx=$(echo "$pos_x - $last_position_x" | bc -l)
    local dy=$(echo "$pos_y - $last_position_y" | bc -l)
    local dz=$(echo "$pos_z - $last_position_z" | bc -l)
    local distance_jump=$(echo "sqrt($dx*$dx + $dy*$dy + $dz*$dz)" | bc -l)
    
    # 计算角度变化
    local angle_diff=$(echo "scale=3; a($ori_w - $last_orientation_w) * 180 / 3.14159" | bc -l 2>/dev/null || echo "0")
    
    # 检测位置跳跃
    if [[ $(echo "$distance_jump > $TRAJECTORY_JUMP_THRESHOLD" | bc -l) -eq 1 ]]; then
        echo "ERROR:TrajectoryJump:${distance_jump}m"
        ((trajectory_jump_count++))
        return 3
    fi
    
    # 检测角度跳跃
    if [[ $(echo "scale=3; if ($angle_diff < 0) $angle_diff * -1 else $angle_diff" | bc -l) -gt 0 ]]; then
        angle_diff=$(echo "scale=3; if ($angle_diff < 0) $angle_diff * -1 else $angle_diff" | bc -l)
    fi
    
    if [[ $(echo "$angle_diff > $TRAJECTORY_ANGLE_THRESHOLD" | bc -l) -eq 1 ]]; then
        echo "ERROR:TrajectoryRotation:${angle_diff}deg"
        return 4
    fi
    
    # 检测异常漂移速度
    if [[ $(echo "$time_diff > 0.1" | bc -l) -eq 1 ]]; then
        local drift_speed=$(echo "$distance_jump / $time_diff" | bc -l)
        if [[ $(echo "$drift_speed > $TRAJECTORY_DRIFT_THRESHOLD" | bc -l) -eq 1 ]]; then
            echo "ERROR:TrajectoryDrift:${drift_speed}m/s"
            return 5
        fi
    fi
    
    # 更新总移动距离
    trajectory_total_distance=$(echo "$trajectory_total_distance + $distance_jump" | bc -l)
    
    # 更新上次位姿
    last_pose_time=$current_time
    last_position_x=$pos_x
    last_position_y=$pos_y
    last_position_z=$pos_z
    last_orientation_w=$ori_w
    
    echo "INFO:TrajectoryNormal:${distance_jump}m"
    return 0
}

# 获取轨迹统计信息
get_trajectory_statistics() {
    echo "轨迹统计 - 跳跃次数:$trajectory_jump_count 总距离:$trajectory_total_distance"
}

# 重置轨迹状态
reset_trajectory_state() {
    last_pose_time=0
    last_position_x=0
    last_position_y=0
    last_position_z=0
    last_orientation_w=1
    trajectory_stuck_start_time=0
    trajectory_jump_count=0
    trajectory_total_distance=0
    echo "轨迹状态已重置"
}
