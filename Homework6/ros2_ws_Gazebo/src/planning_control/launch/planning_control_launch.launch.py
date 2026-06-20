"""
一键启动规划和控制节点
用法：
ros2 launch planning_control planning_control_launch.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 路径规划节点
        Node(
            package='planning_control',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        # 车辆控制节点
        Node(
            package='planning_control',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
    ])
