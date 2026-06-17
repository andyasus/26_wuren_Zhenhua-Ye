from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. robot_state_publisher 解析URDF，发布静态TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=['vehicle_tf.urdf'],  # 确保这个文件在同一目录或指定路径
            output='screen'
        ),
        
        # 2. 启动RViz（方便他们可视化）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])