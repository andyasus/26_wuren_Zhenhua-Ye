"""
锥桶检测节点
订阅 /scan, 聚类提取锥桶坐标, 发布在 lidar_link 坐标系下
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')

        # 参数
        self.cluster_tolerance = 0.4      # 同一锥桶点间距阈值 (m)
        self.min_cluster_size = 2          # 最少点数才算锥桶
        self.max_cluster_size = 8          # 最多点数 (过滤墙面等)
        self.max_range = 10.0              # 最大检测距离

        # 订阅
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # 发布检测到的锥桶 (MarkerArray 用于 RViz)
        self.pub_cones = self.create_publisher(
            MarkerArray, '/detected_cones', 10)

        # 发布锥桶在 base_link 下的坐标 (自定义方式: 通过 Marker 传递)
        # 也发布一份 PoseArray 格式供定位节点使用
        from geometry_msgs.msg import PoseArray, Pose
        self.pub_cone_poses = self.create_publisher(
            PoseArray, '/detected_cone_poses', 10)

        self.get_logger().info('锥桶检测节点已启动')

    def scan_callback(self, scan: LaserScan):
        """处理雷达扫描数据"""
        # 1. 将 LaserScan 转为笛卡尔坐标点
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < self.max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append(np.array([x, y]))
            angle += scan.angle_increment

        if len(points) < 2:
            return

        points = np.array(points)

        # 2. 简单欧式距离聚类
        clusters = self._cluster(points)

        # 3. 提取每个聚类的中心
        cone_centers = []
        for cluster in clusters:
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                center = np.mean(cluster, axis=0)
                cone_centers.append(center)

        # 4. 发布 MarkerArray (在 lidar_link 坐标系)
        self._publish_markers(cone_centers, scan.header.stamp, scan.header.frame_id)
        self._publish_pose_array(cone_centers, scan.header.stamp, scan.header.frame_id)

    def _cluster(self, points):
        """简单距离聚类"""
        clusters = []
        used = set()

        for i in range(len(points)):
            if i in used:
                continue
            cluster = [points[i]]
            used.add(i)
            # 找所有距离小于 tolerance 的点
            for j in range(i + 1, len(points)):
                if j in used:
                    continue
                dist = np.linalg.norm(points[i] - points[j])
                if dist < self.cluster_tolerance:
                    cluster.append(points[j])
                    used.add(j)
            clusters.append(np.array(cluster))

        return clusters

    def _publish_markers(self, centers, stamp, frame_id):
        """发布锥桶 MarkerArray 供 RViz 显示"""
        markers = MarkerArray()
        for i, center in enumerate(centers):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame_id
            marker.ns = 'cones'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8
            markers.markers.append(marker)

        self.pub_cones.publish(markers)

    def _publish_pose_array(self, centers, stamp, frame_id):
        """发布锥桶位置 PoseArray 供其他节点使用"""
        from geometry_msgs.msg import PoseArray, Pose
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = frame_id
        for center in centers:
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self.pub_cone_poses.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
