"""
锥桶建图+定位 启动文件
启动: 锥桶检测 + 地标匹配定位 + 融合定位 (可选) + RViz

运行:
  ros2 launch cone_mapping cone_mapping.launch.py

  # 同时启动 EKF 融合定位:
  ros2 launch cone_mapping cone_mapping.launch.py with_fusion:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    try:
        share_dir = get_package_share_directory('cone_mapping')
    except Exception:
        share_dir = os.path.dirname(os.path.dirname(__file__))

    cone_map_path = os.path.join(share_dir, 'config', 'cone_map.yaml')

    # 参数
    with_fusion = LaunchConfiguration('with_fusion', default='false')
    with_rviz = LaunchConfiguration('with_rviz', default='true')

    declare_with_fusion = DeclareLaunchArgument('with_fusion', default_value='false')
    declare_with_rviz = DeclareLaunchArgument('with_rviz', default_value='true')

    # 锥桶检测节点
    cone_detector = Node(
        package='cone_mapping',
        executable='cone_detector',
        name='cone_detector',
        output='screen',
    )

    # 地标匹配定位节点
    landmark_localizer = Node(
        package='cone_mapping',
        executable='landmark_localizer',
        name='landmark_localizer',
        output='screen',
        parameters=[{'cone_map': cone_map_path}],
    )

    # 融合定位节点 (可选)
    fusion_node = Node(
        package='sensor_fusion',
        executable='fusion_node',
        name='sensor_fusion_node',
        output='screen',
        condition=IfCondition(with_fusion),
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(with_rviz),
    )

    return LaunchDescription([
        declare_with_fusion,
        declare_with_rviz,
        cone_detector,
        landmark_localizer,
        fusion_node,
        rviz_node,
    ])
