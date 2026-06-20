# planning_control 包

这个包是我负责的路径规划和车辆控制部分。

## 我做了什么

大作业里我们组的分工是：
- 队友A做仿真环境和传感器
- 队友B做传感器融合（EKF），发布 /vehicle_pose
- 队友C做建图（还没做完）
- 我负责规划和控制

我这个包会订阅队友B的 /vehicle_pose，再自己生成一条参考路径 /path，
然后用 Pure Pursuit 算法算出速度和转向角，发布 /cmd_vel 给仿真里的车。

## 文件说明

- `planning_control/path_planner_node.py`：路径规划节点
- `planning_control/controller_node.py`：Pure Pursuit 控制节点
- `launch/planning_control_launch.launch.py`：一键启动上面两个节点
- `test_path_control.py`：离线测试脚本，不用启动 ROS2 也能跑
- `笔记.md`：我写这个包时的学习记录和踩坑
- `作业要求对照.md`：对照大作业 PPT 一条条看有没有覆盖

## 怎么跑

### 1. 编译

先进入工作空间：

```powershell
cd gazebo+大作业/ros2_ws
colcon build --packages-select planning_control
. install/setup.ps1
```

如果是在 Linux 或者 WSL 里跑，就用：

```bash
cd gazebo+大作业/ros2_ws
colcon build --packages-select planning_control
source install/setup.bash
```

如果提示 scipy 没装，要装一下：

```bash
pip install scipy
```

### 2. 启动

```bash
ros2 launch planning_control planning_control_launch.launch.py
```

注意：这个包本身只负责规划和控制，前面要先启动 Gazebo、传感器节点、
传感器融合节点，这样 /vehicle_pose 才有数据。

## 路径是怎么规划的

因为队友C的建图节点还没做完，所以我先用了一个离线方法：

直接从 model.sdf 里把蓝色和黄色锥桶的坐标抄出来，
按赛道截面手动列出中线点，然后用 scipy 的三次样条插值平滑一下。

等 /track_map 有了之后，可以改成在线规划，代码里已经留了回调函数，
只是具体解析还要等建图节点的消息格式确定。

## 控制是怎么做的

用的是 Pure Pursuit 纯追踪算法。思路是：

1. 在车前面找一个目标点，距离大概 1.5m 到 2m 左右
2. 看这个目标点在车的左侧还是右侧
3. 根据横向偏差算转向角
4. 弯越急，速度越慢

具体参数在 controller_node.py 最上面，比如：
- wheelbase = 0.25（轴距，和仿真插件里的一致）
- max_speed = 2.0（最大速度，先设小一点，调好了再加）
- lookahead_base = 1.5（基础前视距离）

## 我踩过的坑

1. 一开始我不知道 /vehicle_pose 的频率只有 10Hz，控制循环也设的 10Hz，
   后来发现这样刚好匹配，不用太快。

2. 坐标系一定要搞清楚。world 是东北天（x东 y北），base_link 是前左上。
   算 Pure Pursuit 时一定要把目标点转到车的局部坐标系，
   不然转向角方向会反。

3. 赛道转弯部分左右锥桶数量不一样，一开始用循环取平均，
    最后一个点没配对上，路径终点偏到了 (24.45, 11.93)。
    后来改成手动按截面列出所有中线点，样条插值时强制对齐起点和终点，
    现在路径起终点都对了。

4. definition.md 里写的是 /cmd_ackermann（AckermannDrive），
    但实际仿真用的是 Gazebo Harmonic 的 AckermannSteering 插件，
    它只认 /cmd_vel（Twist）。我一开始按 definition.md 写的，
    后来看了队友A的 bridge.yaml 才发现不对，改成了 /cmd_vel。
    另外 definition.md 里轴距写 0.3m，但仿真插件里是 0.25m，
    也得按仿真的来，不然转向角算出来会偏。

## 接口

订阅：
- /vehicle_pose（geometry_msgs/PoseStamped）
- /track_map（visualization_msgs/MarkerArray，可选）

发布：
- /path（nav_msgs/Path）
- /cmd_vel（geometry_msgs/Twist，linear.x=速度，angular.z=转向角）

## 还没做的事

- 在线规划：等 /track_map 确定格式后再写
- 速度规划：现在只是简单按曲率线性减速，可以做得更平滑
- 参数调优：前视距离、最大速度这些都要在仿真里试
