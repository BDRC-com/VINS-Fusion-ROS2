# VINS-Fusion-ROS2 监控脚本使用说明

## 📋 脚本概览

本目录包含三个VINS监控脚本：

1. **vins_enhanced_monitor.zsh** - 增强版智能恢复监控脚本
2. **vins_persistent_monitor.zsh** - 持久监控脚本（推荐）
3. **start_vins_monitor.zsh** - 快速启动器

## 🚀 快速开始

### 方法1：使用快速启动器（推荐新手）

```bash
cd /home/bluebarry/desktop/wc/vins_fu_ws/VINS-Fusion-ROS2
./start_vins_monitor.zsh
```

然后选择：
- 1: 持久监控（推荐）
- 2: 测试模式（30秒）
- 3: 仅启动VINS节点
- 4: 查看帮助

### 方法2：直接使用持久监控脚本

```bash
cd /home/bluebarry/desktop/wc/vins_fu_ws/VINS-Fusion-ROS2
./vins_persistent_monitor.zsh config/euroc/real_euroc_mono_imu_config_fixed.yaml
```

## ⚙️ 持久监控脚本特性

### 核心功能
- ✅ **持续运行** - 直到手动按Ctrl+C停止
- ✅ **智能重启** - 自动重启失败的VINS节点
- ✅ **重启限制** - 防止无限重启循环
- ✅ **冷却时间** - 重启间隔保护（默认30秒）
- ✅ **实时状态** - 显示节点运行状态
- ✅ **详细日志** - 记录所有操作和状态变化

### 命令选项

```bash
./vins_persistent_monitor.zsh <config_file> [options]
```

**选项：**
- `--interval N` - 监控间隔秒数（默认：5秒）
- `--max-restart N` - 最大重启尝试次数（默认：10次）
- `--cooldown N` - 重启冷却时间秒数（默认：30秒）
- `--help|-h` - 显示帮助信息

**示例：**
```bash
# 基本使用
./vins_persistent_monitor.zsh config/euroc/real_euroc_mono_imu_config_fixed.yaml

# 自定义参数
./vins_persistent_monitor.zsh config.yaml --interval 10 --max-restart 5 --cooldown 60
```

## 📊 状态显示说明

监控过程中会显示实时状态：

```
[17:26:53] 状态: 运行中 | 重启: 1 | 失败: 0
```

**状态含义：**
- `运行中` - VINS节点正常工作
- `运行中 (等待数据)` - 节点启动但等待IMU/图像数据
- `运行中 (特征跟踪失败)` - 特征点跟踪出现问题
- `运行中 (位置跳跃)` - 检测到位置异常跳跃
- `未运行` - 节点已停止

## 🔧 配置文件

推荐使用修复后的配置文件：
- `config/euroc/real_euroc_mono_imu_config_fixed.yaml` - 修复了IMU参数的单目配置

## 📝 日志文件

- **监控日志**: `/tmp/vins_persistent_monitor.log`
- **VINS节点日志**: `/tmp/vins_node.log`

查看日志：
```bash
# 查看监控日志
tail -f /tmp/vins_persistent_monitor.log

# 查看VINS节点日志
tail -f /tmp/vins_node.log
```

## 🛠️ 故障排除

### 问题1：脚本无法启动
```bash
# 检查权限
ls -la vins_persistent_monitor.zsh

# 添加执行权限
chmod +x vins_persistent_monitor.zsh
```

### 问题2：VINS节点启动失败
```bash
# 检查配置文件
./vins_persistent_monitor.zsh --help

# 查看节点日志
cat /tmp/vins_node.log
```

### 问题3：无限重启
- 脚本会自动限制重启次数（默认10次）
- 可以调整 `--max-restart` 参数
- 检查数据源是否正常

### 问题4：没有数据源
这是正常的，VINS会显示"等待数据"状态但不会退出监控。

## 🎯 最佳实践

1. **使用持久监控脚本**进行长期运行
2. **调整监控间隔**根据需要（5-10秒比较合适）
3. **设置合理的重启限制**避免无限循环
4. **定期检查日志**了解系统状态
5. **使用Ctrl+C优雅停止**监控

## 📞 获取帮助

```bash
# 查看脚本帮助
./vins_persistent_monitor.zsh --help

# 查看快速启动器
./start_vins_monitor.zsh
```

---
**注意**: 监控脚本会持续运行直到手动停止。即使没有IMU/图像数据源，监控也会继续工作。
