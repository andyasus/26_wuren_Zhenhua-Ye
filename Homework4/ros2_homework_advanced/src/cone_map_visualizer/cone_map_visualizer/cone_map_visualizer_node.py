import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from fsd_common_msgs.msg import Map


class ConeMapVisualizer(Node):
    """订阅 /estimation/slam/map 话题，获取锥桶地图并转换为 RViz Marker 进行可视化。"""

    def __init__(self):
        super().__init__('cone_map_visualizer')

        # 发布锥桶 Marker 数组，供 RViz 订阅
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_markers', 10)

        # 订阅地图话题
        self.sub = self.create_subscription(
            Map,
            '/estimation/slam/map',
            self.map_callback,
            10
        )

        self.get_logger().info('锥桶地图可视化节点已启动！')

    def map_callback(self, msg):
        """收到 Map 消息时，将锥桶转换为 Marker 并发布。"""
        marker_array = MarkerArray()

        # 为每种颜色的锥桶创建 marker
        self._create_cone_markers(
            marker_array, msg.cone_red, msg.header,
            'cone_red', 1.0, 0.0, 0.0       # 红色
        )
        self._create_cone_markers(
            marker_array, msg.cone_blue, msg.header,
            'cone_blue', 0.0, 0.0, 1.0      # 蓝色
        )
        self._create_cone_markers(
            marker_array, msg.cone_yellow, msg.header,
            'cone_yellow', 1.0, 1.0, 0.0    # 黄色
        )
        self._create_cone_markers(
            marker_array, msg.cone_unknown, msg.header,
            'cone_unknown', 0.6, 0.6, 0.6   # 未知 -> 灰色
        )

        self.marker_pub.publish(marker_array)
        self.get_logger().debug(
            f'已发布 {len(marker_array.markers)} 个锥桶 Marker '
            f'(红色={len(msg.cone_red)}, 蓝色={len(msg.cone_blue)}, '
            f'黄色={len(msg.cone_yellow)}, 未知={len(msg.cone_unknown)})'
        )

    def _create_cone_markers(self, marker_array, cones, header,
                             namespace, r, g, b):
        """将锥桶列表转换为 Marker，添加到 marker_array 中。"""
        for i, cone in enumerate(cones):
            marker = Marker()

            # 使用地图消息中的坐标系（与 bag 中的 frame_id 一致）
            marker.header.frame_id = header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = namespace
            marker.id = i
            # 使用 CYLINDER（柱体）来直观表示锥桶
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # 位置
            marker.pose.position.x = cone.position.x
            marker.pose.position.y = cone.position.y
            marker.pose.position.z = cone.position.z
            marker.pose.orientation.w = 1.0

            # 尺寸（锥桶大小）
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.4

            # 颜色
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            # 一直显示，直到收到新的 marker
            marker.lifetime = rclpy.time.Duration(seconds=0).to_msg()

            marker_array.markers.append(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
