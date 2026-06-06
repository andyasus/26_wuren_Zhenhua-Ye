#!/bin/bash
# 锥桶地图可视化启动脚本
# 请在 workspace 根目录（ros2_homework_advanced）下运行

set -e

# 获取脚本所在目录（workspace 根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "  锥桶地图可视化 - 启动脚本"
echo "=========================================="

# 1. 设置环境变量
echo "[1/4] 设置 ROS2 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. 启动 RViz（后台）
echo "[2/4] 启动 RViz2..."
rviz2 -d src/cone_map_visualizer/config/cone_map.rviz &
RVIZ_PID=$!
sleep 2

# 3. 启动锥桶可视化节点（后台）
echo "[3/4] 启动锥桶可视化节点..."
ros2 run cone_map_visualizer cone_map_visualizer_node &
NODE_PID=$!
sleep 1

# 4. 播放 rosbag 数据
echo "[4/4] 播放 rosbag 数据..."
ros2 bag play src/map_to_visualize/ -r 1.0 --clock

# 清理后台进程
echo ""
echo "Bag 播放完毕，正在清理..."
kill $NODE_PID 2>/dev/null || true
kill $RVIZ_PID 2>/dev/null || true
wait 2>/dev/null || true
echo "完成！"
