# FSAE 无人系统大作业 —— 第 3 组

> **直角弯赛道 · 建图-规划-控制-仿真 完整闭环**
>
> ROS2 Humble + Gazebo Harmonic · 2026 年 6 月

---

## 📋 目录

- [小组分工](#小组分工)
- [系统架构](#系统架构)
- [各模块说明](#各模块说明)
- [话题接口总览](#话题接口总览)
- [编译与运行](#编译与运行)
- [联调记录与问题修复](#联调记录与问题修复)
- [已知限制与优化方向](#已知限制与优化方向)
- [AI 使用说明](#ai-使用说明)

---

## 👥 小组分工

| 组员 | 负责模块 | 包名 | 完成状态 |
|------|---------|------|:--:|
| 龚鑫哲 | 仿真环境 + 传感器桥接 | `car_sim` | ✅ |
| 叶镇华 | 传感器融合 (EKF) | `sensor_fusion` | ✅ |
| 何嘉喜 | 锥桶检测 + 地标匹配建图 | `cone_mapping` | ✅ |
| 陈政烨 | 路径规划 + Pure Pursuit 控制 | `planning_control` | ✅ |

> 另有 `sim_perception`（感知仿真，Pyarmor 加密）和 `tracks`（锥桶 3D 模型）两个辅助包。

---

## 🏗 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                         Gazebo Harmonic                          │
│  ┌─────────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌────────┐ ┌───────┐   │
│  │ Camera  │ │LiDAR │ │ GPS  │ │ IMU  │ │  Mag   │ │ Odom  │   │
│  └────┬────┘ └──┬───┘ └──┬───┘ └──┬───┘ └───┬────┘ └──┬────┘   │
└───────┼─────────┼─────────┼─────────┼─────────┼─────────┼───────┘
        │         │         │         │         │         │
   ┌────▼─────────▼─────────▼─────────▼─────────▼─────────▼────┐
   │                   gazebo_bridge (v3)                       │
   │         gz topic -e --json-output → ROS2 topics            │
   └────┬─────────┬─────────┬─────────┬─────────┬─────────┬────┘
        │         │         │         │         │         │
   /camera   /scan    /gps/fix  /imu/data /mag/data /odometry
   /image_raw
        │         │         │         │         │         │
   ┌────▼────┐ ┌──▼───┐ ┌───▼────────▼─────────▼─────────▼────┐
   │  RViz   │ │ cone │ │         sensor_fusion               │
   │ 可视化  │ │detect│ │   EKF (px,py,yaw,v,yaw_rate)        │
   └─────────┘ └──┬───┘ └────────────────┬────────────────────┘
                  │                      │
             /detected_cone_poses   /vehicle_pose
                  │                      │
             ┌────▼──────────┐    ┌──────▼──────────┐
             │ landmark_loc. │    │  path_planner   │
             │ 地标匹配定位   │    │  离线中线+样条   │
             └────┬──────────┘    └──────┬──────────┘
                  │                      │
          /corrected_pose               /path
                                         │
                                  ┌──────▼──────────┐
                                  │  controller     │
                                  │  Pure Pursuit   │
                                  └──────┬──────────┘
                                         │
                                    /cmd_vel
                                         │
                                  ┌──────▼──────────┐
                                  │ AckermannSteer  │
                                  │ (Gazebo 插件)    │
                                  └─────────────────┘
```

---

## 📦 各模块说明

### 1. `car_sim` — 仿真环境 + 传感器桥接（龚鑫哲）

**职责**: 提供 Gazebo 仿真世界、车辆 SDF 模型（含 6 个传感器）、自定义 Python 桥接节点。

| 传感器 | SDF 类型 | 更新频率 | ROS2 话题 |
|--------|---------|:--:|------|
| GPS | `navsat` | 10Hz | `/gps/fix` |
| IMU | `imu` | 10Hz | `/imu/data` |
| 磁力计 | `magnetometer` | 10Hz | `/mag/data` |
| 里程计 | AckermannSteering 插件 | 30Hz | `/odometry` |
| LiDAR | `lidar` | 10Hz | `/scan` |
| 相机 | `camera` | 15Hz | `/camera/image_raw` |

**关键修复**:
- GPS 传感器类型从 Gazebo Classic 的 `gps` 改为 Harmonic 的 `navsat`
- 使用 subprocess + JSON 方案桥接（gz-transport13 Python subscribe 有底层 bug）
- `GZ_IP=127.0.0.1` 禁用组播，解决 "Network is unreachable"
- `--render-engine ogre` 兼容 Intel+NVIDIA 混合显卡

### 2. `sensor_fusion` — 传感器融合 (叶镇华)

**职责**: 订阅 GPS/IMU/Mag/Odom，用 5 维 EKF 融合输出 `/vehicle_pose`。

| 传感器 | 观测维度 | 噪声设置 |
|--------|---------|:--:|
| GPS | 位置 (px, py) | 0.5 m |
| 磁力计 | 航向角 (yaw) | 0.05 rad |
| 里程计 | 速度 (v) | 0.1 m/s |
| IMU | 角速度 (yaw_rate) | 0.05 rad/s |

**核心算法**:
- 运动模型: 匀速+恒角速度模型，雅可比矩阵 F 手动推导
- GPS→米制: 局部切平面法，第一帧定为原点 (0, -15)
- 磁力计→航向: `yaw = π/2 - atan2(my, mx)`（ENU + FLU 坐标系）

### 3. `cone_mapping` — 锥桶检测 + 地标定位（何嘉喜）

**职责**: 从 LiDAR 点云检测锥桶，与预标定地图匹配，修正车辆位姿。

- `cone_detector`: 欧式距离聚类 → 锥桶中心坐标
- `landmark_localizer`: SVD 刚性变换求解，发布 `/corrected_pose`

### 4. `planning_control` — 路径规划 + 控制（陈政烨）

**职责**: 生成赛道中线参考路径，用 Pure Pursuit 算法输出控制指令。

- `path_planner_node`: 离线中线 + 三次样条插值 → `/path`（200 个点）
- `controller_node`: Pure Pursuit 控制器 → `/cmd_vel`（linear.x=速度, angular.z=转向角）

| 参数 | 值 | 说明 |
|------|----|------|
| 轴距 | 0.3 m | 与 SDF 的 `wheel_base` 一致 |
| 前视距离 | 1.5 m | 基础值，随速度动态调整 |
| 最大速度 | 2.0 m/s | 直道 |
| 最小速度 | 0.5 m/s | 弯道 |

---

## 📡 话题接口总览

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/gps/fix` | NavSatFix | gazebo_bridge | sensor_fusion |
| `/imu/data` | Imu | gazebo_bridge | sensor_fusion |
| `/mag/data` | MagneticField | gazebo_bridge | sensor_fusion |
| `/odometry` | Odometry | gazebo_bridge | sensor_fusion |
| `/scan` | LaserScan | gazebo_bridge | cone_detector |
| `/camera/image_raw` | Image | gazebo_bridge | (RViz) |
| `/vehicle_pose` | PoseStamped | sensor_fusion | controller, landmark_localizer |
| `/path` | Path | path_planner | controller |
| `/cmd_vel` | Twist | controller | gazebo_bridge→Gazebo |
| `/detected_cone_poses` | PoseArray | cone_detector | landmark_localizer |
| `/corrected_pose` | PoseStamped | landmark_localizer | (RViz) |

---

## 🔧 编译与运行

### 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Harmonic (gz-sim8)
- Python 3.10 + numpy + scipy + PyYAML

### 编译

```bash
cd ~/FSAE-Unmanned-System-Homework/Homework6/ros2_ws_Gazebo
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 运行

```bash
# 一键启动全部（仿真 + 融合 + 建图 + 规划控制）
./run_sim.sh all

# 仅基础仿真 + 传感器桥接
./run_sim.sh

# 仿真 + 传感器融合
./run_sim.sh fusion

# 仿真 + 锥桶检测建图
./run_sim.sh mapping

# 仿真 + 规划控制
./run_sim.sh planning
```

### 让车跑起来

```bash
# 直接通过 Gazebo 话题控制车辆
gz topic -t "/model/shixi_car/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 2.0}"

# 或者启动 planning_control 后，通过 ROS2 话题控制
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}"
```

### 查看数据

```bash
# 查看融合位姿
ros2 topic echo /vehicle_pose

# 查看 GPS
ros2 topic echo /gps/fix

# 查看参考路径
ros2 topic echo /path
```

---

## 🔧 联调记录与问题修复

### 2026.6.20 联调修复汇总

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | Gazebo GUI 空白/冻结 | ogre2 不兼容混合显卡 | 启动参数 `--render-engine ogre` |
| 2 | GPS 话题存在但无数据 | SDF 用了 Classic 的 `type='gps'`，Harmonic 需 `type='navsat'` | 改 SDF + bridge 话题后缀 |
| 3 | 传感器 JSON 数据全零 | protobuf JSON 序列化用 camelCase | `_safe_get()` 兼容两种命名 |
| 4 | 传感器数据出不来 | 传感器插件放在 world SDF 无法加载 | 移到模型 SDF 中 |
| 5 | spawn 失败 | 组播 "Network is unreachable" | `GZ_IP=127.0.0.1` 禁用组播 |
| 6 | 轴距不匹配 | controller 用 0.25，SDF 是 0.3 | 统一为 0.3 |
| 7 | planning_control 未集成 | 主 launch 未包含 | 新增 `with_planning` 参数 |
| 8 | 依赖缺失 | scipy / yaml 未声明 | package.xml 补充 |

---

## ⚠️ 已知限制与优化方向

1. **磁力计航向偏置**: 融合输出 yaw≈-3.05 rad（≈-175°），SDF 中车辆朝向 1.57 rad（90°）。Gazebo 模拟磁场与 spherical_coordinates 定义的北向不完全一致，真实场景需磁力计标定
2. **里程计需车辆移动**: 车静止时里程计数据为零，发布 `cmd_vel` 后正常
3. **GPS 高度忽略**: 假设赛道平坦，仅用经纬度
4. **无时间同步**: 各传感器回调独立触发，未做时间戳对齐
5. **离线路径规划**: 目前用预设中线点，在线建图路径规划待完善
6. **`sim_perception` 包**: 代码被 Pyarmor 加密，无法审计和修改

---

## 🤖 AI 使用说明

本项目中 AI 工具主要用于以下方面：
- **算法理解**: EKF 雅可比矩阵推导、GPS 局部切平面公式理解
- **调试辅助**: launch 文件语法错误排查、Gazebo SDF 格式校验
- **环境配置**: 依赖包安装、protobuf 版本兼容性分析
- **文档整理**: README 格式化、接口表格生成

**所有核心代码均为组员手写并添加详细中文注释**，AI 生成的部分均经过理解和修改后使用。答辩时可逐行解释代码逻辑。

---

## 📁 项目文件结构

```
Homework6/ros2_ws_Gazebo/
├── run_sim.sh                    # 一键启动脚本
├── README.md                     # 本文件
├── PROJECT_SUMMARY.md            # 项目修复详细记录
├── src/
│   ├── car_sim/                  # 仿真环境 + 传感器桥接
│   │   ├── car_sim/
│   │   │   ├── gazebo_bridge.py  # 传感器桥接节点 (v3)
│   │   │   └── spawn_car.py      # 模型 spawn
│   │   ├── urdf/car_sim.sdf      # 车辆模型（含6个传感器）
│   │   ├── worlds/shixi_track.sdf # 赛道世界
│   │   └── launch/
│   ├── sensor_fusion/            # 传感器融合
│   │   ├── sensor_fusion/
│   │   │   ├── sensor_fusion_node.py
│   │   │   ├── ekf_localization.py
│   │   │   └── gps_converter.py
│   │   └── 叶镇华 传感器融合节点.md
│   ├── cone_mapping/             # 锥桶检测 + 地标定位
│   │   ├── cone_mapping/
│   │   │   ├── cone_detector.py
│   │   │   └── landmark_localizer.py
│   │   └── config/cone_map.yaml
│   ├── planning_control/         # 路径规划 + 控制
│   │   ├── planning_control/
│   │   │   ├── path_planner_node.py
│   │   │   └── controller_node.py
│   │   └── test_path_control.py
│   ├── sim_perception/           # 感知仿真 (加密)
│   └── tracks/                   # 锥桶 3D 模型
├── docs/
│   └── team_B_sensor_fusion_handover.md
├── build/  install/  log/
└── config/
```
