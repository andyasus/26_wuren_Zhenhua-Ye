#!/usr/bin/env python3
"""直接用 gz-transport 将车辆 SDF 生成到 Gazebo，绕过 ros_gz_sim create 的 protobuf bug"""
import os
import sys
import time
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node


def main():
    sdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    world_name = sys.argv[2] if len(sys.argv) > 2 else 'shixi_track'
    model_name = sys.argv[3] if len(sys.argv) > 3 else 'shixi_car'

    if not sdf_path:
        print('[spawn_car] ERROR: 缺少 SDF 路径参数')
        sys.exit(1)

    with open(sdf_path, 'r') as f:
        sdf_content = f.read()

    msg = EntityFactory()
    msg.sdf = sdf_content
    msg.name = model_name

    node = Node()

    service = f'/world/{world_name}/create'
    print(f'[spawn_car] 等待 Gazebo 就绪...')
    print(f'[spawn_car] 服务: {service}')
    print(f'[spawn_car] SDF:  {sdf_path} ({len(sdf_content)} 字节)')

    # 等 Gazebo 就绪——因为 Gazebo 启动需要时间
    # 特别是首次启动时加载模型和锥桶场景
    max_retries = 30
    first = True
    for i in range(max_retries):
        success, response = node.request(
            service, msg, EntityFactory, Boolean, timeout=5000
        )
        if first:
            first = False
            if not success:
                print(f'[spawn_car] Gazebo 尚未就绪，等待中...')

        if success and response.data:
            print(f'[spawn_car] ✓ 模型 [{model_name}] 在第 {i+1} 次尝试后创建成功')
            return 0

        print(f'[spawn_car] 重试 {i+1}/{max_retries} (success={success}, data={response.data})')
        time.sleep(2)

    print(f'[spawn_car] ✗ 模型 [{model_name}] 创建失败：{max_retries} 次重试后仍未成功', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
