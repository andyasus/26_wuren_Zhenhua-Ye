"""
传感器融合ROS2节点
订阅：GPS、IMU、磁力计、里程计
发布：融合后的车辆位姿 /vehicle_pose

坐标系：
所有传感器数据都是相对于base_link的
融合结果发布到world坐标系下

接口定义（来自gazebo+大作业/test/test/definition.md）：
  订阅：
    /gps/fix       → sensor_msgs/NavSatFix   经纬度
    /imu/data      → sensor_msgs/Imu         角速度+线加速度
    /mag/data      → sensor_msgs/MagneticField 磁场→航向角
    /odometry      → nav_msgs/Odometry       线速度
  发布：
    /vehicle_pose  → geometry_msgs/PoseStamped 融合后位姿
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import numpy as np

from .ekf_localization import EKF
from .gps_converter import GPSConverter

class SensorFusionNode(Node):
    """传感器融合节点"""

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # 1. 创建EKF和GPS转换器
        self.ekf = EKF()
        self.gps_converter = GPSConverter()

        # 2. 订阅传感器话题
        # 订阅GPS
        self.sub_gps = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        # 订阅IMU
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # 订阅磁力计
        self.sub_mag = self.create_subscription(
            MagneticField, '/mag/data', self.mag_callback, 10)
        
        # 订阅里程计
        self.sub_odo = self.create_subscription(
            Odometry, '/odometry', self.odo_callback, 10)
        
        # 3. 发布融合后的位姿
        self.pub_pose = self.create_publisher(
            PoseStamped, '/vehicle_pose', 10)
        
        # 4. 定时器：定期执行预测和发布（10Hz）
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 记录上一帧时间，用于计算dt
        self.last_time = self.get_clock().now()

        self.get_logger().info('传感器融合节点已启动了喵～')

    def gps_callback(self, msg):
        """GPS数据回调"""
        # 将经纬度转换为world坐标
        x, y = self.gps_converter.convert(msg.latitude, msg.longitude)
        # 更新EKF的位置观测
        self.ekf.update_gps(x, y)
        self.get_logger().debug(f'GPS更新了喵：（{x:.2f},{y:.2f})')

    def imu_callback(self, msg):
        """IMU数据回调，提取角速度"""
        # 角速度的z分量就是yaw_rate
        yaw_rate = msg.angular_velocity.z
        self.ekf.update_imu_yaw_rate(yaw_rate)

    def mag_callback(self, msg):
        """磁力计数据回调，提取航向角"""
        # 从磁场强度计算航向角
        # msg.magnetic_field.x, y, z是三维磁场强度
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y

        # 世界系 ENU (x=东,y=北), 车体系 FLU (x=前,y=左)
        # 磁力计 rpy=(0,0,0) 与 base_link 同朝向
        # 世界磁场指向 +y(北), 车体读数: mx=B·sinθ, my=B·cosθ
        # atan2(my,mx) = π/2 - yaw  ⇒  yaw = π/2 - atan2(my,mx)
        yaw = np.pi / 2 - np.arctan2(my, mx)
        # 归一化到 [-pi, pi]
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        self.ekf.update_magnetometer(yaw)

    def odo_callback(self, msg):
        """里程计数据回调，提取速度"""
        # 前向速度在twist.linear.x中（局部坐标系下）
        v = msg.twist.twist.linear.x
        self.ekf.update_odometry(v)
        
    def timer_callback(self):
        """定时器回调，预测+发布"""
        # 计算时间步长
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 限制dt大小，防止启动时跳变
        if dt > 0.5:
            dt = 0.1
        
        # 执行EKF预测
        self.ekf.predict(dt)

        # 获取当前状态
        px, py, yaw, v, yaw_rate = self.ekf.get_state()

        # 发布位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(px)
        pose_msg.pose.position.y = float(py)
        pose_msg.pose.position.z = 0.0

        # 将yaw转为四元数
        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pub_pose.publish(pose_msg)

        # 打印日志
        self.get_logger().info(f'位姿: x={px:.2f}, y={py:.2f}, yaw={yaw:.2f}, v={v:.2f}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()