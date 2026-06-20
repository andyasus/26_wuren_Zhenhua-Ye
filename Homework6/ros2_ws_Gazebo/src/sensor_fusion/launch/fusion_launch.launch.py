"""一键启动传感器融合节点"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='sensor_fusion_node',
            output='screen'
        ),
    ])