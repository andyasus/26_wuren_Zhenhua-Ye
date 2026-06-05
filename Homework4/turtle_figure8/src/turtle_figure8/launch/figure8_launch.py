#!/usr/bin/env python3
"""
启动乌龟8字运动：turtlesim + 控制节点
用法:
  ros2 launch turtle_figure8 figure8_launch.py
  ros2 launch turtle_figure8 figure8_launch.py linear_speed:=2.0 radius:=2.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtle_figure8')
    config = os.path.join(pkg_dir, 'config', 'params.yaml')

    # 命令行可覆盖的参数
    speed_arg = DeclareLaunchArgument('linear_speed', default_value='1.5',
                                      description='线速度 (m/s)')
    radius_arg = DeclareLaunchArgument('radius', default_value='2.0',
                                       description='圆半径 (0.5~2.75)')

    turtlesim = Node(package='turtlesim', executable='turtlesim_node',
                     name='turtlesim', output='screen')

    control = Node(package='turtle_figure8', executable='figure8_node',
                   name='turtle_figure8', output='screen',
                   parameters=[config,
                               {'linear_speed': LaunchConfiguration('linear_speed'),
                                'radius': LaunchConfiguration('radius')}])

    return LaunchDescription([speed_arg, radius_arg, turtlesim, control])