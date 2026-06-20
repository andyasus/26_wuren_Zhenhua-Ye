"""
路径规划节点

这个节点负责生成一条让车跟着跑的参考路径。
目前有两种方式：
1. 离线方式：直接从赛道锥桶坐标（model.sdf里抄出来的）生成中线
2. 在线方式：等队友的建图节点做好后，订阅 /track_map 实时更新路径

输出话题：/path （nav_msgs/Path）
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import numpy as np
from scipy.interpolate import splprep, splev


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # 发布规划好的路径
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # 订阅建图节点发布的锥桶地图（如果有的话）
        self.track_map_sub = self.create_subscription(
            MarkerArray,
            '/track_map',
            self.track_map_callback,
            10
        )

        # 用一个定时器，每秒发布一次路径
        # 这样即使没收到 /track_map，离线路径也会一直发
        self.timer = self.create_timer(1.0, self.publish_path)

        # 标记是否收到过 /track_map
        self.has_online_map = False

        # 离线赛道中线点
        # 从 model.sdf 里按赛道截面手动列出的中线点
        # 一共12个截面，覆盖起点到终点
        self.centerline = self.build_offline_centerline()

        # 平滑后的路径
        self.smoothed_path = self.smooth_centerline(self.centerline)

        self.get_logger().info('路径规划节点启动啦，先用离线中线跑')

    def build_offline_centerline(self):
        """
        从赛道锥桶坐标生成中线点
        直接按赛道截面列出每个中线点，起点到终点一共12个点。

        因为转弯部分左右锥桶数量不一样，不能直接取平均，
        所以我按赛道截面手动配对，列出每个截面的中线点。
        """
        # 直接列出每个截面的中线点
        # 从 model.sdf 里按赛道截面手动配对得到的
        center = [
            [0.0, -15.0],       # 起点截面
            [0.0, -10.0],
            [0.0, -5.0],
            [0.0, 0.0],
            [0.725, 4.075],     # 转弯开始
            [2.805, 7.65],
            [5.97, 10.28],
            [9.84, 11.655],
            [12.0, 12.0],       # 进入东向直道
            [17.0, 12.0],
            [22.0, 12.0],
            [27.0, 12.0],       # 赛道终点
        ]

        return np.array(center)

    def smooth_centerline(self, points, num=200):
        """
        用三次样条把中线点插值成平滑曲线
        points: Nx2 的numpy数组
        num: 输出多少个点
        """
        if len(points) < 4:
            self.get_logger().warning('中线点太少，没法样条插值，直接用原来的点')
            return points

        x = points[:, 0]
        y = points[:, 1]

        # s=0 表示曲线严格经过所有点，s>0 会稍微平滑一点
        # 我把 s 从 0.5 改成 0.1，这样终点不会偏太多，同时又不会太抖
        try:
            tck, u = splprep([x, y], s=0.1, k=3)
            u_new = np.linspace(0, 1, num)
            x_new, y_new = splev(u_new, tck)
            smoothed = np.column_stack([x_new, y_new])

            # 强制让第一个点精确等于原始起点，最后一个点精确等于原始终点
            # 因为样条有时候会差一点，直接补上比较保险
            smoothed[0] = points[0]
            smoothed[-1] = points[-1]
            return smoothed
        except Exception as e:
            self.get_logger().error(f'样条插值出错了: {e}')
            return points

    def track_map_callback(self, msg):
        """
        收到 /track_map 后，从中提取锥桶位置生成路径
        这个功能等队友建图节点做好后才能用
        """
        if not self.has_online_map:
            self.get_logger().info('收到 /track_map 啦，切换到在线规划！')
            self.has_online_map = True

        # 这里先简单处理：把 marker 位置分成蓝黄两组，再算中线
        # 具体实现要等建图节点的消息格式确定
        # 现在先用离线路径兜底
        pass

    def publish_path(self):
        """发布 /path 话题"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'

        for x, y in self.smoothed_path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
