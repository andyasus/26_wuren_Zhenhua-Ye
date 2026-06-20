#!/bin/bash
# ============================================================
# Gazebo + ROS2 仿真一键启动脚本
# 用法:
#   ./run_sim.sh                          # 基础启动
#   ./run_sim.sh fusion                   # 启动 + 传感器融合
#   ./run_sim.sh mapping                  # 启动 + 锥桶检测
#   ./run_sim.sh planning                 # 启动 + 规划控制
#   ./run_sim.sh all                      # 启动全部
# ============================================================
set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$WS_DIR"

echo "============================================"
echo "  FSAE 无人系统仿真启动"
echo "============================================"

# 1. Source ROS2 + 本工作空间
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ 找不到 ROS2 Humble，请先安装"
    exit 1
fi

if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "⚠ 未找到 install/setup.bash，正在编译..."
    colcon build --packages-select car_sim sensor_fusion cone_mapping
    source install/setup.bash
fi

# 2. 解析参数
WITH_FUSION="false"
WITH_MAPPING="false"
WITH_PLANNING="false"

case "${1:-}" in
    fusion)
        WITH_FUSION="true"
        echo "  ✅ 传感器融合: 启用"
        ;;
    mapping)
        WITH_MAPPING="true"
        echo "  ✅ 锥桶检测: 启用"
        ;;
    planning)
        WITH_PLANNING="true"
        echo "  ✅ 规划控制: 启用"
        ;;
    all)
        WITH_FUSION="true"
        WITH_MAPPING="true"
        WITH_PLANNING="true"
        echo "  ✅ 全部模块: 启用"
        ;;
    *)
        echo "  ℹ 仅基础仿真 (传感器桥接 + Gazebo)"
        ;;
esac

# 3. 启动
echo "============================================"
echo "  正在启动 Gazebo + 传感器桥接..."
echo "  (渲染引擎: ogre, 兼容混合显卡)"
echo "============================================"
echo ""
echo "  启动后可检查传感器数据："
echo "    ros2 topic echo /gps/fix"
echo "    ros2 topic echo /vehicle_pose"
echo "    ros2 topic echo /scan"
echo ""

ros2 launch car_sim sim_bringup_harmonic.launch.py \
    with_fusion:="$WITH_FUSION" \
    with_mapping:="$WITH_MAPPING" \
    with_planning:="$WITH_PLANNING"
