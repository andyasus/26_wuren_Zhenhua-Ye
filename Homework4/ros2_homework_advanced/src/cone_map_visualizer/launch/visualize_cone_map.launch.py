"""启动文件：播放 bag 数据、启动锥桶可视化节点、并打开 RViz。"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('cone_map_visualizer')
    bag_path = os.path.join(
        os.path.dirname(package_dir),  # go up from share/ to install/
        '..', '..', 'src', 'map_to_visualize'
    )
    # Resolve to absolute path
    bag_path = os.path.abspath(bag_path)

    return LaunchDescription([
        # 1. 播放 rosbag 数据
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '-r', '1.0', '--clock'],
            output='screen',
        ),

        # 2. 启动锥桶可视化节点
        Node(
            package='cone_map_visualizer',
            executable='cone_map_visualizer_node',
            name='cone_map_visualizer',
            output='screen',
        ),

        # 3. 启动 RViz2（使用预设的 cone_map.rviz 配置文件，自动显示锥桶 Marker）
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(package_dir, 'config', 'cone_map.rviz')],
            output='screen',
        ),
    ])
