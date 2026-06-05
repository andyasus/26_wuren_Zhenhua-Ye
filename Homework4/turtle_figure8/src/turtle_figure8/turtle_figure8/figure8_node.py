#!/usr/bin/env python3
"""
两圆法8字运动节点：左转一圈 + 右转一圈 = 8字
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute


class Figure8Node(Node):
    """发布速度指令，控制乌龟画8字（两圆法）。"""

    def __init__(self):
        super().__init__('turtle_figure8')

        # 1. 读取并校验参数
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('turtle_name', 'turtle1')
        self.declare_parameter('reset_turtle', True)

        self.v = self.get_parameter('linear_speed').value
        self.R = self.get_parameter('radius').value
        self.rate = self.get_parameter('control_rate').value
        self.turtle = self.get_parameter('turtle_name').value
        self.do_reset = self.get_parameter('reset_turtle').value

        # 参数保护
        if self.v <= 0:
            self.v = 0.5
            self.get_logger().warn('线速度 <= 0, 已设为 0.5')
        if self.R <= 0:
            self.R = 0.5
            self.get_logger().warn('半径 <= 0, 已设为 0.5')
        if self.R > 2.75:
            self.get_logger().warn(f'半径 {self.R} > 2.75 可能会撞墙！(屏幕 11x11)')
        if self.rate <= 0:
            self.rate = 50.0

        # 2. 计算运动参数
        self.omega = self.v / self.R           # 角速度大小
        self.T_circle = 2 * math.pi * self.R / self.v  # 画一个圆的时间
        self.T_cycle = 2 * self.T_circle       # 一个完整8字的时间

        # 3. 创建发布器和服务客户端
        self.pub = self.create_publisher(
            Twist, f'/{self.turtle}/cmd_vel', 10)
        self.teleport = self.create_client(
            TeleportAbsolute, f'/{self.turtle}/teleport_absolute')

        # 4. 重置乌龟位置到屏幕中心
        if self.do_reset:
            if self.teleport.wait_for_service(timeout_sec=2.0):
                req = TeleportAbsolute.Request()
                req.x, req.y, req.theta = 5.5, 5.5, 0.0
                self.teleport.call_async(req)
                self.get_logger().info('乌龟已重置到 (5.5, 5.5, 0.0)')
            else:
                self.get_logger().warn('teleport 服务不可用，跳过重置')

        # 5. 启动定时器
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self.timer_cb)

        # 6. 打印启动信息
        self.get_logger().info('=' * 45)
        self.get_logger().info('8字运动已启动 (两圆法)')
        self.get_logger().info(f'  速度={self.v} m/s, 半径={self.R} m')
        self.get_logger().info(f'  角速度={self.omega:.2f} rad/s')
        self.get_logger().info(f'  每圆={self.T_circle:.1f}s, 每8字={self.T_cycle:.1f}s')
        self.get_logger().info(f'  8字大小 ≈ {4*self.R:.1f} x {2*self.R:.1f}')
        self.get_logger().info('=' * 45)

    def timer_cb(self):
        """定时器回调：根据相位切换左转/右转。"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # 半个周期左转，半个周期右转
        w = self.omega if (elapsed % self.T_cycle) < self.T_circle else -self.omega

        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

        self.get_logger().info(
            f't={elapsed:5.1f}s  v={self.v:.1f}  ω={w:+.2f}  '
            f'{"上半圆" if w > 0 else "下半圆"}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('正在停止乌龟...')
        node.pub.publish(Twist())  # 发零速度指令
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()