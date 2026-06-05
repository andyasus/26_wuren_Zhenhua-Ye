# 乌龟画8字 🐢
## 用的两圆法
两圆法——让乌龟先左转一整圈，再右转一整圈，如此循环。
- 左转一圈 → 上半圆
- 右转一圈 → 下半圆
- 拼起来就是一个8字
- 8字尺寸：半径R=2.0

## 节点启动命令

```bash
# 构建
cd ~/FSAE-Unmanned-System-Homework/Homework4/turtle_figure8
colcon build --packages-select turtle_figure8
source install/setup.bash

# 默认参数启动（默认参数：速度 1.5，半径 2.0）
ros2 launch turtle_figure8 figure8_launch.py

# 自定义参数启动
ros2 launch turtle_figure8 figure8_launch.py linear_speed:=2.0 radius:=2.5
```

## 学习记录
- 本人代码能力比较一般，我们专业只学了python，所以这次作业的节点代码还是决定使用python写（C++的基础语法是参加车队笔试前一周速通的）；
- ROS2的基本使用是在本次完成作业的过程中加紧学习的（看鱼香ROS的图文教程学的）；
- figure8_node.py和figure8_launch.py两个文件是我先自己尝试写，写完之后发现乌龟容易撞墙，然后询问DeepSeek来修改，节点代码里加了把重置乌龟到屏幕中心的部分；
- 根据DeepSeek的建议优化了代码，让代码看起来更规范一点。
