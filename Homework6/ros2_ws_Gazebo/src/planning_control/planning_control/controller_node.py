"""
车辆控制节点

用 Pure Pursuit（纯追踪）算法让车沿着 /path 跑。
订阅 /vehicle_pose 获取当前位置，订阅 /path 获取参考路径，
输出 /cmd_vel 控制速度和转向角。

公式：
- 在车辆前方找一个距离为 L 的目标点
- 计算横向偏差 y_lat（目标点在车辆左侧还是右侧）
- 曲率 kappa = 2 * y_lat / (L * L)
- 转向角 delta = atan(轴距 * kappa)

注意：仿真用的是 Gazebo Harmonic 的 AckermannSteering 插件，
它订阅 /cmd_vel（geometry_msgs/Twist），不是 /cmd_ackermann。
linear.x = 线速度，angular.z = 前轮转向角。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion
import math
import numpy as np


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # 订阅当前位姿和参考路径
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vehicle_pose',
            self.pose_callback,
            10
        )
        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )

        # 发布控制指令
        # 仿真用的是 /cmd_vel (Twist)，不是 /cmd_ackermann
        # linear.x = 线速度，angular.z = 前轮转向角
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 定时器，10Hz 发布控制量
        self.timer = self.create_timer(0.1, self.control_loop)

        # 当前位姿
        self.current_pose = None

        # 参考路径，转成 numpy 数组方便计算
        self.ref_path = None

        # 车辆参数
        self.wheelbase = 0.3   # 轴距，和 car_sim.sdf 里的 wheel_base 必须一致
        self.max_speed = 2.0  # 最大速度，先设慢一点，调好了再加快
        self.min_speed = 0.5  # 最低速度，弯道用

        # Pure Pursuit 前视距离
        self.lookahead_base = 1.5  # 基础前视距离
        self.lookahead_gain = 0.3  # 速度越快看得越远

        self.get_logger().info('控制节点启动啦')

    def pose_callback(self, msg):
        """保存当前位姿"""
        self.current_pose = msg.pose

    def path_callback(self, msg):
        """保存参考路径"""
        points = []
        for pose in msg.poses:
            points.append([pose.pose.position.x, pose.pose.position.y])
        self.ref_path = np.array(points)

    def get_yaw_from_pose(self, pose):
        """从四元数里提取 yaw 角"""
        q = pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return yaw

    def find_target_point(self, x, y, yaw):
        """
        在参考路径上找前视距离对应的目标点
        返回目标点在车辆局部坐标系下的 (x_local, y_local)
        """
        if self.ref_path is None or len(self.ref_path) == 0:
            return None

        # 计算当前速度，这里简单用固定值，后面可以改成根据实际速度调整
        # 因为 /vehicle_pose 里没有速度，先用最大速度算
        speed = self.max_speed
        lookahead = self.lookahead_base + self.lookahead_gain * speed

        # 找路径上距离车辆当前位置最近的点
        distances = np.linalg.norm(self.ref_path - np.array([x, y]), axis=1)
        nearest_idx = np.argmin(distances)

        # 从最近点开始往后找，找到第一个距离超过 lookahead 的点
        target_idx = nearest_idx
        for i in range(nearest_idx, len(self.ref_path)):
            dist = np.linalg.norm(self.ref_path[i] - np.array([x, y]))
            if dist >= lookahead:
                target_idx = i
                break

        # 如果到末尾都没找到，就用最后一个点
        target_idx = min(target_idx, len(self.ref_path) - 1)
        tx, ty = self.ref_path[target_idx]

        # 把目标点从 world 坐标系转到车辆局部坐标系
        # 世界坐标系下目标点相对于车的位置
        dx = tx - x
        dy = ty - y

        # 旋转到车辆局部坐标系（前左上）
        # 车辆朝向是 yaw，局部 x 轴向前，y 轴向左
        x_local = dx * math.cos(yaw) + dy * math.sin(yaw)
        y_local = -dx * math.sin(yaw) + dy * math.cos(yaw)

        return x_local, y_local

    def compute_speed(self, curvature):
        """
        根据曲率决定速度
        曲率越大（弯越急），速度越慢
        """
        # 用曲率的绝对值
        k = abs(curvature)

        # 简单线性减速
        # 曲率 0 时速度最大，曲率 1 时速度最小
        speed = self.max_speed - k * (self.max_speed - self.min_speed)
        speed = max(self.min_speed, min(self.max_speed, speed))

        return speed

    def control_loop(self):
        """主控制循环"""
        # 如果还没收到位姿或路径，就不发控制
        if self.current_pose is None:
            self.get_logger().info('还没收到 /vehicle_pose，等一等...', throttle_sec=2.0)
            return

        if self.ref_path is None:
            self.get_logger().info('还没收到 /path，等一等...', throttle_sec=2.0)
            return

        # 当前位姿
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.get_yaw_from_pose(self.current_pose)

        # 找目标点
        target = self.find_target_point(x, y, yaw)
        if target is None:
            return

        x_local, y_local = target

        # 前视距离 L
        L = math.sqrt(x_local ** 2 + y_local ** 2)

        # 防止除零
        if L < 0.1:
            L = 0.1

        # Pure Pursuit 公式
        # y_local 就是横向偏差（在车辆左侧为正，右侧为负）
        kappa = 2.0 * y_local / (L * L)
        steering_angle = math.atan(self.wheelbase * kappa)

        # 限制转向角，防止打太大
        max_steering = 0.6  # 约34度
        steering_angle = max(-max_steering, min(max_steering, steering_angle))

        # 根据曲率算速度
        speed = self.compute_speed(kappa)

        # 发布控制指令
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steering_angle)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'速度={speed:.2f}, 转向={steering_angle:.2f}, '
            f'目标局部=({x_local:.2f}, {y_local:.2f})',
            throttle_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
