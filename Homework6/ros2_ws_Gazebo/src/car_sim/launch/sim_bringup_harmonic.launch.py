#!/usr/bin/env python3
"""
sim_bringup_harmonic.launch.py  (Gazebo Harmonic 版本)
----------------------------------
完整仿真启动文件:
  1. 设置 GZ_SIM_RESOURCE_PATH (加载锥桶等赛道模型)
  2. 处理 car_sim.xacro -> URDF -> robot_description
  3. 启动 Gazebo Harmonic 并加载 map.world
  4. 启动自定义 gazebo_bridge (传感器数据桥接，替换不兼容的 ros_gz_bridge)
  5. 启动 robot_state_publisher (发布TF)
  6. 将机器人模型 spawn 到 Gazebo 中
  7. (可选) 启动 sensor_fusion 融合定位节点
  8. (可选) 启动 cone_mapping 锥桶定位
  9. (可选) 启动 planning_control 路径规划+控制

运行方式:
  ros2 launch car_sim sim_bringup_harmonic.launch.py
  ros2 launch car_sim sim_bringup_harmonic.launch.py with_fusion:=true with_mapping:=true with_planning:=true
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_workspace_src(share_dir):
    """从 install/share 目录推回 workspace/src"""
    # share_dir 形如: .../install/xxx/share/xxx
    # 向上 4 级 = workspace 根目录
    ws_root = os.path.normpath(os.path.join(share_dir, '..', '..', '..', '..'))
    src_dir = os.path.join(ws_root, 'src')
    if os.path.isdir(src_dir):
        return src_dir
    return None


def generate_launch_description():

    # ============================================================
    # 路径: 优先用 install/share (编译后), 回退到源码路径
    # ============================================================
    try:
        car_sim_share = get_package_share_directory('car_sim')
    except Exception:
        car_sim_share = os.path.dirname(os.path.dirname(__file__))
        print(f'[WARN] car_sim share 回退: {car_sim_share}')

    # 已安装到 share 的文件 (通过 setup.py data_files)
    xacro_file = os.path.join(car_sim_share, 'urdf', 'car_sim.xacro')
    sdf_file = os.path.join(car_sim_share, 'urdf', 'car_sim.sdf')       # Harmonic 用 SDF spawn
    world_path = os.path.join(car_sim_share, 'worlds', 'shixi_track.sdf')

    # 锥桶地图 (来自 cone_mapping 包的 share)
    try:
        cone_map_share = get_package_share_directory('cone_mapping')
        cone_map_path = os.path.join(cone_map_share, 'config', 'cone_map.yaml')
    except Exception:
        cone_map_path = os.path.join(car_sim_share, '..', '..', '..', 'src',
                                     'cone_mapping', 'config', 'cone_map.yaml')

    # workspace/src 目录 (用于加载 Gazebo 模型)
    ws_src = _find_workspace_src(car_sim_share)
    if ws_src:
        tracks_models = os.path.join(ws_src, 'tracks', 'models')
    else:
        tracks_models = ''

    print(f'[INFO] car_sim share: {car_sim_share}')
    print(f'[INFO] workspace src: {ws_src}')
    print(f'[INFO] xacro: {xacro_file}')
    print(f'[INFO] sdf: {sdf_file}')
    print(f'[INFO] world: {world_path}')
    print(f'[INFO] tracks models: {tracks_models}')
    print(f'[INFO] cone map: {cone_map_path}')

    # ============================================================
    # Launch 参数
    # ============================================================
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='-15.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.06')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.5708')
    with_fusion = LaunchConfiguration('with_fusion', default='false')
    with_mapping = LaunchConfiguration('with_mapping', default='false')
    with_planning = LaunchConfiguration('with_planning', default='false')

    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='0.0')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-15.0')
    declare_spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.06')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='1.5708')
    declare_with_fusion = DeclareLaunchArgument('with_fusion', default_value='false')
    declare_with_mapping = DeclareLaunchArgument('with_mapping', default_value='false')
    declare_with_planning = DeclareLaunchArgument('with_planning', default_value='false')

    # ============================================================
    # GZ_SIM_RESOURCE_PATH (让 Gazebo 找到锥桶模型)
    # ============================================================
    gz_model_paths = []
    if tracks_models and os.path.isdir(tracks_models):
        gz_model_paths.append(tracks_models)
    gz_model_paths += [
        os.path.expanduser('~/.gz/sim/models'),
        '/usr/share/gz/gz-sim/models',
    ]
    gz_model_path_str = ':'.join(p for p in gz_model_paths if os.path.isdir(p))
    print(f'[INFO] GZ_SIM_RESOURCE_PATH = {gz_model_path_str}')

    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_model_path_str
    )

    # GZ_IP=127.0.0.1 禁用组播（解决 "Network is unreachable" 导致 spawn 失败）
    set_gz_ip = SetEnvironmentVariable(
        name='GZ_IP',
        value='127.0.0.1'
    )

    # ============================================================
    # Xacro -> URDF
    # ============================================================
    import xacro

    if not os.path.isfile(xacro_file):
        print(f'[ERROR] xacro 文件不存在: {xacro_file}')
        sys.exit(1)

    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()
    print(f'[INFO] URDF 生成成功, {len(robot_description)} 字符')

    # 临时 URDF 仅用于 robot_state_publisher (TF)，spawn 使用 SDF 文件

    # ============================================================
    # Gazebo Harmonic
    # ============================================================
    if os.path.isfile(world_path):
        # --render-engine ogre：使用 ogre(ogre1) 渲染，兼容 Intel+NVIDIA 混合显卡
        # 如显卡完美支持 ogre2，可删除此参数或改为 ogre2
        gz_sim_cmd = ['gz', 'sim', '-r', '--render-engine', 'ogre', world_path]
    else:
        print(f'[WARN] 未找到 map.world, 使用空世界')
        gz_sim_cmd = ['gz', 'sim', '-r', '--render-engine', 'ogre']

    gazebo_sim = ExecuteProcess(cmd=gz_sim_cmd, output='screen')

    # ============================================================
    # gazebo_bridge (自定义 Python 桥接，替换不兼容的 ros_gz_bridge)
    # ⚠ ros-humble-ros-gz-bridge 链接的是 libignition-transport11 (Fortress),
    #   而 Gazebo Harmonic 8 用的是 libgz-transport13，两者不兼容。
    #   本自定义节点使用 gz-transport13 Python API 直接读取传感器数据。
    # ============================================================
    gz_bridge = Node(
        package='car_sim',
        executable='gazebo_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[
            {'world_name': 'shixi_track'},
            {'model_name': 'shixi_car'},
        ],
    )

    # ============================================================
    # robot_state_publisher
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
    # spawn 机器人 (直接用 gz-transport, 绕过 ros_gz_sim create 的 protobuf bug)
    # ============================================================
    spawn_entity = Node(
        package='car_sim',
        executable='spawn_car',
        name='spawn_shixi_car',
        output='screen',
        arguments=[sdf_file, 'shixi_track', 'shixi_car'],
    )

    # ============================================================
    # sensor_fusion (可选)
    # ============================================================
    fusion_node = Node(
        package='sensor_fusion',
        executable='fusion_node',
        name='sensor_fusion_node',
        output='screen',
        condition=IfCondition(with_fusion)
    )

    # ============================================================
    # cone_mapping (可选)
    # ============================================================
    cone_detector = Node(
        package='cone_mapping',
        executable='cone_detector',
        name='cone_detector',
        output='screen',
        condition=IfCondition(with_mapping),
    )

    landmark_localizer = Node(
        package='cone_mapping',
        executable='landmark_localizer',
        name='landmark_localizer',
        output='screen',
        condition=IfCondition(with_mapping),
        parameters=[{'cone_map': cone_map_path}],
    )

    # ============================================================
    # planning_control (可选)
    # ============================================================
    path_planner = Node(
        package='planning_control',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        condition=IfCondition(with_planning),
    )

    controller = Node(
        package='planning_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        condition=IfCondition(with_planning),
    )

    # ============================================================
    # 组装
    # ============================================================
    actions = [
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_with_fusion,
        declare_with_mapping,
        declare_with_planning,
        set_gz_resource,
        set_gz_ip,
        gazebo_sim,
        gz_bridge,
        robot_state_pub,
        spawn_entity,
        fusion_node,
        cone_detector,
        landmark_localizer,
        path_planner,
        controller,
    ]

    return LaunchDescription(actions)
