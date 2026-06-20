"""
离线测试脚本

因为 ROS2 环境有时候不好启动，先写个纯 Python 的测试，
验证路径规划和控制逻辑对不对。
"""

import numpy as np
from scipy.interpolate import splprep, splev
import math


# ================== 复制 path_planner_node 里的逻辑 ==================

def build_offline_centerline():
    center = [
        [0.0, -15.0],
        [0.0, -10.0],
        [0.0, -5.0],
        [0.0, 0.0],
        [0.725, 4.075],
        [2.805, 7.65],
        [5.97, 10.28],
        [9.84, 11.655],
        [12.0, 12.0],
        [17.0, 12.0],
        [22.0, 12.0],
        [27.0, 12.0],
    ]
    return np.array(center)


def smooth_centerline(points, num=200):
    x = points[:, 0]
    y = points[:, 1]
    tck, u = splprep([x, y], s=0.1, k=3)
    u_new = np.linspace(0, 1, num)
    x_new, y_new = splev(u_new, tck)
    smoothed = np.column_stack([x_new, y_new])
    smoothed[0] = points[0]
    smoothed[-1] = points[-1]
    return smoothed


# ================== 复制 controller_node 里的逻辑 ==================

def find_target_point(x, y, yaw, ref_path, lookahead_base, lookahead_gain, speed):
    """
    复制 controller_node 里的 find_target_point 逻辑
    注意：这里 lookahead 的计算方式和 controller_node 完全一致
    """
    lookahead = lookahead_base + lookahead_gain * speed

    distances = np.linalg.norm(ref_path - np.array([x, y]), axis=1)
    nearest_idx = np.argmin(distances)

    target_idx = nearest_idx
    for i in range(nearest_idx, len(ref_path)):
        dist = np.linalg.norm(ref_path[i] - np.array([x, y]))
        if dist >= lookahead:
            target_idx = i
            break

    target_idx = min(target_idx, len(ref_path) - 1)
    tx, ty = ref_path[target_idx]

    dx = tx - x
    dy = ty - y

    x_local = dx * math.cos(yaw) + dy * math.sin(yaw)
    y_local = -dx * math.sin(yaw) + dy * math.cos(yaw)

    return x_local, y_local


def pure_pursuit(x_local, y_local, wheelbase):
    L = math.sqrt(x_local ** 2 + y_local ** 2)
    if L < 0.1:
        L = 0.1
    kappa = 2.0 * y_local / (L * L)
    steering = math.atan(wheelbase * kappa)
    max_steering = 0.6
    steering = max(-max_steering, min(max_steering, steering))
    return steering


# ================== 测试 ==================

if __name__ == '__main__':
    print('开始离线测试...')

    # 测试路径规划
    center = build_offline_centerline()
    path = smooth_centerline(center)
    print(f'原始中线点数：{len(center)}')
    print(f'平滑后路径点数：{len(path)}')
    print(f'路径起点：({path[0][0]:.2f}, {path[0][1]:.2f})')
    print(f'路径终点：({path[-1][0]:.2f}, {path[-1][1]:.2f})')

    # 和 controller_node 一样的参数
    wheelbase = 0.3  # 和 car_sim.sdf 里的 wheel_base 一致
    max_speed = 2.0
    lookahead_base = 1.5
    lookahead_gain = 0.3

    # 测试1：车在北向直道上，速度约 2m/s
    # 此时 lookahead = 1.5 + 0.3 * 2.0 = 2.1m
    print('\n=== 测试1：直道向北（速度2m/s，前视2.1m）===')
    x, y, yaw = 0.0, -10.0, math.pi / 2
    x_local, y_local = find_target_point(x, y, yaw, path, lookahead_base, lookahead_gain, max_speed)
    steering = pure_pursuit(x_local, y_local, wheelbase)
    print(f'车辆位置：({x:.2f}, {y:.2f}), 朝向：{yaw:.2f}')
    print(f'前视距离：{lookahead_base + lookahead_gain * max_speed:.2f}m')
    print(f'目标点局部坐标：({x_local:.2f}, {y_local:.2f})')
    print(f'转向角：{steering:.4f} rad')

    # 测试2：车在弯道入口偏右，速度约 1m/s（弯道减速）
    # 此时 lookahead = 1.5 + 0.3 * 1.0 = 1.8m
    print('\n=== 测试2：弯道入口偏右（速度1m/s，前视1.8m）===')
    x, y, yaw = 1.0, 2.0, math.pi / 2
    slow_speed = 1.0
    x_local, y_local = find_target_point(x, y, yaw, path, lookahead_base, lookahead_gain, slow_speed)
    steering = pure_pursuit(x_local, y_local, wheelbase)
    print(f'车辆位置：({x:.2f}, {y:.2f}), 朝向：{yaw:.2f}')
    print(f'前视距离：{lookahead_base + lookahead_gain * slow_speed:.2f}m')
    print(f'目标点局部坐标：({x_local:.2f}, {y_local:.2f})')
    print(f'转向角：{steering:.4f} rad')

    # 测试3：车在起点，验证起终点是否正确
    print('\n=== 测试3：起点终点验证 ===')
    print(f'路径起点：({path[0][0]:.2f}, {path[0][1]:.2f})  (期望 0.00, -15.00)')
    print(f'路径终点：({path[-1][0]:.2f}, {path[-1][1]:.2f})  (期望 27.00, 12.00)')
    start_ok = abs(path[0][0]) < 0.01 and abs(path[0][1] + 15.0) < 0.01
    end_ok = abs(path[-1][0] - 27.0) < 0.01 and abs(path[-1][1] - 12.0) < 0.01
    print(f'起点正确：{start_ok}，终点正确：{end_ok}')

    print('\n所有测试完成。')
