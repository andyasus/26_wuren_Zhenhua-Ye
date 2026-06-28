#!/usr/bin/env python3
"""
sim_bringup.launch.py
---------------------
完整仿真启动文件:
  1. 设置 Gazebo 模型路径 (加载锥桶等赛道模型)
  2. 处理 test.xacro -> URDF -> robot_description
  3. 启动 Gazebo 并加载 map.world
  4. 启动 robot_state_publisher (发布TF)
  5. 将机器人模型 spawn 到 Gazebo 中
  6. (可选) 启动 sim_perception 感知节点

运行方式:
  ros2 launch car_sim sim_bringup.launch.py

  # 同时启动感知节点:
  ros2 launch car_sim sim_bringup.launch.py with_perception:=true

  # 指定机器人初始位置 (赛道起点附近):
  ros2 launch car_sim sim_bringup.launch.py spawn_x:=0.0 spawn_y:=-14.0 spawn_z:=1.0
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # 路径计算
    # ============================================================
    try:
        car_sim_share = get_package_share_directory('car_sim')
    except Exception:
        # 回退: 从当前文件路径向上查
        car_sim_share = os.path.dirname(os.path.dirname(__file__))
        print(f'[WARN] 无法通过ament找到car_sim, 回退到: {car_sim_share}')

    # tracks 目录: 在 car_sim 源目录的兄弟目录
    # 结构: .../src/car_sim 和 .../src/tracks
    car_sim_src = os.path.join(car_sim_share, '..', '..', '..', 'src', 'car_sim')
    car_sim_src = os.path.normpath(car_sim_src)
    if not os.path.isdir(car_sim_src):
        # car_sim_share 可能本身就是源目录
        car_sim_src = car_sim_share
    tracks_dir = os.path.normpath(os.path.join(car_sim_src, '..', 'tracks'))
    car_sim_urdf = os.path.join(car_sim_src, 'urdf')

    print(f'[INFO] car_sim 源码目录: {car_sim_src}')
    print(f'[INFO] tracks 目录: {tracks_dir}')
    print(f'[INFO] urdf 目录: {car_sim_urdf}')

    # ============================================================
    # Launch 参数
    # ============================================================
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='1.0')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')
    with_perception = LaunchConfiguration('with_perception', default='false')

    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description='机器人spawn X坐标 (m)')
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y', default_value='0.0',
        description='机器人spawn Y坐标 (m)')
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z', default_value='1.0',
        description='机器人spawn Z坐标 (m), 建议>0让车自由落体到地面')
    declare_spawn_yaw = DeclareLaunchArgument(
        'spawn_yaw', default_value='0.0',
        description='机器人朝向 (rad)')
    declare_with_perception = DeclareLaunchArgument(
        'with_perception', default_value='false',
        description='是否同时启动 sim_perception 感知节点')

    # ============================================================
    # Gazebo 模型路径 (让Gazebo找到 blue_cone/yellow_cone/shixi)
    # ============================================================
    gazebo_model_paths = [
        os.path.join(tracks_dir, 'models'),
        os.path.join(os.path.expanduser('~'), '.gazebo', 'models'),
        '/usr/share/gazebo/models',
    ]
    gazebo_model_path_str = ':'.join(
        p for p in gazebo_model_paths if os.path.isdir(p)
    )
    print(f'[INFO] GAZEBO_MODEL_PATH = {gazebo_model_path_str}')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path_str
    )

    # ============================================================
    # Xacro -> URDF
    # ============================================================
    import xacro

    test_xacro_path = os.path.join(car_sim_urdf, 'test.xacro')
    orig_xacro_path = os.path.join(car_sim_urdf, 'car_sim.xacro')

    if os.path.isfile(test_xacro_path):
        xacro_file = test_xacro_path
        print(f'[INFO] 使用 xacro 文件: {xacro_file}')
    else:
        xacro_file = orig_xacro_path
        print(f'[WARN] test.xacro 不存在, 回退到: {xacro_file}')

    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()
    print(f'[INFO] URDF 生成成功, 长度: {len(robot_description)} 字符')

    # ============================================================
    # 查找 world 文件
    # ============================================================
    world_path = os.path.join(tracks_dir, 'map.world')
    if not os.path.isfile(world_path):
        print(f'[WARN] 未找到 map.world 于 {world_path}, Gazebo将启动空白世界')
        world_path = ''

    # ============================================================
    # Gazebo 服务端 + 客户端
    # ============================================================
    if world_path:
        gazebo_cmd = ['gzserver', world_path,
                      '-s', 'libgazebo_ros_init.so',
                      '-s', 'libgazebo_ros_factory.so']
    else:
        gazebo_cmd = ['gzserver',
                      '-s', 'libgazebo_ros_init.so',
                      '-s', 'libgazebo_ros_factory.so']

    gazebo_server = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen'
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # ============================================================
    # robot_state_publisher (发布 /tf 和 /tf_static)
    # ============================================================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )

    # ============================================================
    # spawn_entity (将机器人模型放入Gazebo)
    # ============================================================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'shixi_car',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            '-timeout', '30.0',
        ]
    )

    # ============================================================
    # sim_perception 感知节点 (可选)
    # ============================================================
    perception_node = Node(
        package='sim_perception',
        executable='sim_node',
        name='sim_node',
        output='screen',
        condition=IfCondition(with_perception)
    )

    # ============================================================
    # 构建 LaunchDescription
    # ============================================================
    return LaunchDescription([
        # 声明参数
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_with_perception,
        # 设置环境变量
        set_gazebo_model_path,
        # 启动组件
        gazebo_server,
        gazebo_client,
        robot_state_pub,
        spawn_entity,
        perception_node,
    ])
