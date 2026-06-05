# ROS2进阶作业：锥桶地图可视化

## 作业完成情况

利用提供的 `map_to_visualize` bag 文件和 `fsd_common_msgs` 消息包，可视化话题 `/estimation/slam/map` 中的锥桶地图（红色、蓝色、未知颜色锥桶）。

-  创建 `cone_map_visualizer` 包，订阅 `/estimation/slam/map`，将锥桶转为 `MarkerArray` 发布到 `/cone_markers`
-  红/蓝/黄锥桶对应红/蓝/黄色 Marker，未知颜色用灰色
-  提供 RViz 配置文件（Fixed Frame 设为 `world`）
-  用实际 bag 数据验证，成功显示 28 个锥桶

## 坐标系问题

- bag 中 frame_id 是 `world`，而 RViz 默认用 `map`。通过读取 `Map.msg` 的 `header.frame_id` 获知。
- 解决办法：在 RViz 中将 Fixed Frame 改为 `world`（我用的这个），或发布 `world→map` 的静态 tf 变换。

## 运行方式

**编译**
```bash
source /opt/ros/humble/setup.bash
colcon build
```

**一键脚本（推荐）（运行完会自动关闭）**
```bash
source install/setup.bash
./run_visualization.sh
```

**或者手动开三个终端**

终端1 - RViz：
```bash
source install/setup.bash
rviz2 -d src/cone_map_visualizer/config/cone_map.rviz
```

终端2 - 可视化节点：
```bash
source install/setup.bash
ros2 run cone_map_visualizer cone_map_visualizer_node
```

终端3 - 播放数据：
```bash
source install/setup.bash
ros2 bag play src/map_to_visualize/
```

## 学习记录

1. 学习时间不够啊，学得一知半解😭
   - 我让DeepSeek教我建立项目文件夹，读.msg文件：`Map.msg` 里按颜色分数组存放锥桶（`cone_red/blue/yellow/unknown`），每个 `Cone` 有 `position` + 颜色/姿态置信度。
   - 用 `visualization_msgs/MarkerArray` 一次性发布多个 Marker，比单个 Marker 效率更高
   - 坐标系不匹配时，调整 RViz 的 Fixed Frame 是最简单的方法
2. 然后一点一点把节点代码和launch代码堆出来。
3. 一键运行的脚本是DeepSeek告诉我的，不然每次都得开三个终端调试，学到了。
4. 编译 `fsd_common_msgs` 时遇到 `can_msgs` 缺失的问题（依赖了系统中没有的包），去掉该依赖后编译通过 
    