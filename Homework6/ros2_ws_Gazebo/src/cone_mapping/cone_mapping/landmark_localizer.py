"""
地标匹配定位节点
- 加载锥桶地图 (cone_map.yaml)
- 接收检测锥桶 (在 lidar_link 坐标系) 和当前位姿估计
- 将检测锥桶转到 world 系，与已知地图匹配
- 发布修正后的位姿 /corrected_pose
"""

import math
import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class LandmarkLocalizer(Node):
    def __init__(self):
        super().__init__('landmark_localizer')

        # 1. 加载锥桶地图
        map_path = self.declare_parameter(
            'cone_map', '').get_parameter_value().string_value
        if not map_path:
            # 默认路径
            map_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'cone_map.yaml')
            map_path = os.path.normpath(map_path)

        self.known_cones = self._load_map(map_path)
        self.get_logger().info(f'加载了 {len(self.known_cones)} 个锥桶地标')

        # 2. TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. 订阅检测锥桶 + 当前位姿
        self.sub_cones = self.create_subscription(
            PoseArray, '/detected_cone_poses', self.cones_callback, 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, '/vehicle_pose', self.pose_callback, 10)

        # 4. 发布修正位姿
        self.pub_corrected = self.create_publisher(
            PoseStamped, '/corrected_pose', 10)

        # 5. 发布地图锥桶 MarkerArray (RViz)
        self.pub_map_markers = self.create_publisher(
            MarkerArray, '/cone_map_markers', 10)

        # 6. 发布匹配成功的锥桶对
        self.pub_match_markers = self.create_publisher(
            MarkerArray, '/matched_cones', 10)

        # 当前位姿估计
        self.current_pose = None

        # 发布地图锥桶 (一次性)
        self._publish_map_markers()

        self.get_logger().info('地标定位节点已启动')

    def _load_map(self, path):
        """加载锥桶地图"""
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().error(f'锥桶地图不存在: {path}')
            return []

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        cones = []
        for c in data.get('cones', []):
            cones.append(np.array([c['x'], c['y']]))
        return cones

    def pose_callback(self, msg: PoseStamped):
        """保存当前位姿估计"""
        self.current_pose = msg

    def cones_callback(self, msg: PoseArray):
        """接收检测到的锥桶，执行地标匹配"""
        if self.current_pose is None or len(self.known_cones) == 0:
            return

        if len(msg.poses) == 0:
            return

        # 1. 将检测锥桶从检测帧变换到 world 系
        detected_world = []
        src_frame = msg.header.frame_id  # lidar_link
        stamp = msg.header.stamp

        try:
            tf = self.tf_buffer.lookup_transform(
                'world', src_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().debug(f'TF 变换失败: {e}')
            return

        for pose in msg.poses:
            # 用 TF 变换
            from geometry_msgs.msg import PoseStamped as PS
            ps_in = PS()
            ps_in.header = msg.header
            ps_in.pose = pose
            ps_out = do_transform_pose(ps_in, tf)
            detected_world.append(np.array([
                ps_out.pose.position.x,
                ps_out.pose.position.y]))

        if len(detected_world) < 2:
            return

        detected_world = np.array(detected_world)

        # 2. 最近邻匹配 (检测锥桶 ↔ 已知锥桶)
        known = np.array(self.known_cones)
        matches = []
        for dw in detected_world:
            dists = np.linalg.norm(known - dw, axis=1)
            min_idx = np.argmin(dists)
            if dists[min_idx] < 3.0:  # 3m 匹配阈值
                matches.append((dw, known[min_idx], min_idx))

        if len(matches) < 2:
            self.get_logger().debug(f'匹配对数不足: {len(matches)}')

        # 3. 发布匹配对
        self._publish_matches(matches)

        # 4. 用匹配对计算修正位姿 (SVD 刚性变换)
        corrected = self._compute_correction(matches)
        if corrected is not None:
            corrected.header.stamp = stamp
            corrected.header.frame_id = 'world'
            self.pub_corrected.publish(corrected)

    def _compute_correction(self, matches):
        """用 SVD 求解检测锥桶→已知锥桶的刚性变换，修正位姿"""
        if len(matches) < 2:
            return None

        # 质心
        src_pts = np.array([m[0] for m in matches])  # detected
        dst_pts = np.array([m[1] for m in matches])  # known

        src_centroid = np.mean(src_pts, axis=0)
        dst_centroid = np.mean(dst_pts, axis=0)

        # 去质心
        src_centered = src_pts - src_centroid
        dst_centered = dst_pts - dst_centroid

        # SVD 求旋转
        H = src_centered.T @ dst_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # 确保 det(R) = 1
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # 旋转角
        theta = math.atan2(R[1, 0], R[0, 0])

        # 平移 = dst_centroid - R @ src_centroid
        t = dst_centroid - R @ src_centroid

        # 修正后的位姿 = 当前位姿 + 修正量
        if self.current_pose is not None:
            cur_x = self.current_pose.pose.position.x
            cur_y = self.current_pose.pose.position.y
            # 从当前位姿提取 yaw
            from tf_transformations import euler_from_quaternion
            q = self.current_pose.pose.orientation
            _, _, cur_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # 修正
            corrected_x = cur_x + t[0]
            corrected_y = cur_y + t[1]
            corrected_yaw = cur_yaw + theta

            msg = PoseStamped()
            msg.pose.position.x = float(corrected_x)
            msg.pose.position.y = float(corrected_y)
            msg.pose.position.z = 0.0

            from tf_transformations import quaternion_from_euler
            qc = quaternion_from_euler(0.0, 0.0, corrected_yaw)
            msg.pose.orientation.x = qc[0]
            msg.pose.orientation.y = qc[1]
            msg.pose.orientation.z = qc[2]
            msg.pose.orientation.w = qc[3]

            return msg

        return None

    def _publish_map_markers(self):
        """发布已知锥桶地图 MarkerArray"""
        markers = MarkerArray()
        for i, cone in enumerate(self.known_cones):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'cone_map'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(cone[0])
            m.pose.position.y = float(cone[1])
            m.pose.position.z = 0.15
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.3
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 0.5
            markers.markers.append(m)
        self.pub_map_markers.publish(markers)

    def _publish_matches(self, matches):
        """发布匹配对 (绿色连线 Marker)"""
        markers = MarkerArray()
        marker_id = 0

        for dw, kw, _ in matches:
            # 检测锥桶 (绿色球)
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'matches'
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(dw[0])
            m.pose.position.y = float(dw[1])
            m.pose.position.z = 0.15
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8
            markers.markers.append(m)

            # 已知锥桶 (青色球)
            m2 = Marker()
            m2.header.frame_id = 'world'
            m2.header.stamp = self.get_clock().now().to_msg()
            m2.ns = 'matches'
            m2.id = marker_id
            marker_id += 1
            m2.type = Marker.SPHERE
            m2.action = Marker.ADD
            m2.pose.position.x = float(kw[0])
            m2.pose.position.y = float(kw[1])
            m2.pose.position.z = 0.15
            m2.scale.x = 0.25
            m2.scale.y = 0.25
            m2.scale.z = 0.25
            m2.color.r = 0.0
            m2.color.g = 1.0
            m2.color.b = 1.0
            m2.color.a = 0.6
            markers.markers.append(m2)

        self.pub_match_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
