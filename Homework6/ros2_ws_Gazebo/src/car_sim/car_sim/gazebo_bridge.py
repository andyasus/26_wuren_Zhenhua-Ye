#!/usr/bin/env python3
"""
gazebo_bridge.py (v3)
=====================
ROS2 ↔ Gazebo Harmonic 传感器桥接节点（subprocess+JSON 方案）

背景：
  gz-transport13 Python 的 subscribe() 有底层 bug（订阅返回 True 但回调从不触发），
  因此改用 `gz topic -e --json-output` 子进程方式读取数据。

v3 修复：
  - 修正 JSON 字段名：protobuf 默认 JSON 序列化使用 camelCase
    （latitude_deg → latitudeDeg, angular_velocity → angularVelocity 等）
  - 新增 LiDAR → /scan (LaserScan) 桥接
  - 新增 Camera → /camera/image_raw (Image) 桥接
  - 新增 stderr 读取线程，捕获子进程错误
  - 新增首帧数据日志，便于快速诊断
  - `_safe_get` 同时尝试 camelCase 和 snake_case，提高兼容性

桥接的传感器：
  Gazebo 话题                                                → ROS2 话题
  ──────────────────────────────────────────────────────      ─────────────
  /world/{world}/model/{model}/link/gps_link/sensor/gps/navsat /gps/fix
  /world/{world}/model/{model}/link/imu_link/sensor/imu/imu    /imu/data
  /world/{world}/model/{model}/link/magnetometer_link/...      /mag/data
  /world/{world}/model/{model}/link/lidar_link/sensor/lidar/scan /scan
  /world/{world}/model/{model}/link/camera_link/sensor/camera/image /camera/image_raw
  /model/{model}/odometry                                      /odometry
"""

import json
import os
import subprocess
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import (
    NavSatFix, NavSatStatus, Imu, MagneticField,
    LaserScan, Image,
)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose, Twist
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as TimeMsg


# ──────────────────────────────────────────────────────────────────
#  字段名映射：snake_case → camelCase（protobuf JSON 默认格式）
# ──────────────────────────────────────────────────────────────────
_SNAKE_TO_CAMEL = {
    "latitude_deg": "latitudeDeg",
    "longitude_deg": "longitudeDeg",
    "angular_velocity": "angularVelocity",
    "linear_acceleration": "linearAcceleration",
    "field_tesla": "fieldTesla",
    "pose": "pose",           # 无下划线，不变
    "position": "position",   # 无下划线，不变
    "orientation": "orientation",  # 无下划线，不变
    "twist": "twist",         # 无下划线，不变
    "linear": "linear",       # 无下划线，不变
    "angular": "angular",     # 无下划线，不变
}


def _to_camel(snake_key):
    """snake_case → camelCase（仅用于无映射表的情况）"""
    if snake_key in _SNAKE_TO_CAMEL:
        return _SNAKE_TO_CAMEL[snake_key]
    parts = snake_key.split("_")
    return parts[0] + "".join(p.capitalize() for p in parts[1:])


class GazeboBridge(Node):
    """ROS2 ↔ Gazebo Harmonic 传感器桥接节点（subprocess+JSON v3）"""

    def __init__(self):
        super().__init__("gazebo_bridge")

        # ---------- 参数 ----------
        self.declare_parameter("world_name", "shixi_track")
        self.declare_parameter("model_name", "shixi_car")
        self.world = (
            self.get_parameter("world_name").get_parameter_value().string_value
        )
        self.model = (
            self.get_parameter("model_name").get_parameter_value().string_value
        )

        # ---------- 话题名模板 ----------
        self._sensor_tmpl = (
            "/world/{world}/model/{model}/link/{link}/sensor/{sensor}/{output}"
        )

        # ---------- ROS2 发布器 ----------
        self.pub_gps = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)
        self.pub_mag = self.create_publisher(MagneticField, "/mag/data", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odometry", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_camera = self.create_publisher(Image, "/camera/image_raw", 10)

        # ---------- ROS2 订阅器：cmd_vel (ROS2 → Gazebo) ----------
        self.sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self._cmd_vel_topic = f"/model/{self.model}/cmd_vel"

        # ---------- 首帧标志（用于调试日志） ----------
        self._first_msg = {
            "gps": True, "imu": True, "mag": True,
            "odom": True, "scan": True, "camera": True,
        }

        # ---------- 子进程列表 ----------
        self._processes = []

        # ---------- 传感器配置 ----------
        sensors = [
            ("gps_link", "gps", "navsat", self._handle_gps, "gps"),
            ("imu_link", "imu", "imu", self._handle_imu, "imu"),
            ("magnetometer_link", "magnetometer", "magnetometer", self._handle_mag, "mag"),
            ("lidar_link", "lidar", "scan", self._handle_scan, "scan"),
            ("camera_link", "camera", "image", self._handle_camera, "camera"),
        ]

        for link, sensor, output, handler, tag in sensors:
            topic = self._sensor_tmpl.format(
                world=self.world, model=self.model,
                link=link, sensor=sensor, output=output,
            )
            self._start_reader(topic, handler, tag)

        # 里程计（独立话题）
        odom_topic = f"/model/{self.model}/odometry"
        self._start_reader(odom_topic, self._handle_odom, "odom")

        self.get_logger().info(
            f"✅ 桥接已启动（v3 subprocess 模式）：world={self.world}, model={self.model}"
        )
        self.get_logger().info(
            "   JSON 字段名使用 camelCase（protobuf 默认），同时兼容 snake_case"
        )

    # ──────────────────────────────────────────────────────────────
    #  子进程管理
    # ──────────────────────────────────────────────────────────────

    def _start_reader(self, topic, handler, tag):
        """启动子进程读取 gz topic -e 的输出"""
        cmd = ["gz", "topic", "-e", "-t", topic, "--json-output"]
        env = dict(os.environ, GZ_IP="127.0.0.1")  # 禁用组播
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env,
            )
            self._processes.append(proc)
            self.get_logger().info(f"  ✔ 已启动读取 [{tag}]: {topic}")

            # stdout 读取线程
            t = threading.Thread(
                target=self._reader_loop,
                args=(proc, topic, handler, tag),
                daemon=True,
            )
            t.start()

            # stderr 读取线程（捕获 gz 命令的错误输出）
            te = threading.Thread(
                target=self._stderr_reader,
                args=(proc, topic, tag),
                daemon=True,
            )
            te.start()
        except Exception as e:
            self.get_logger().error(f"   ✘ 启动读取 {topic} 失败: {e}")

    def _reader_loop(self, proc, topic, handler, tag):
        """子进程 stdout 输出读取循环"""
        try:
            for line in proc.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                    handler(data)
                except json.JSONDecodeError as e:
                    self.get_logger().debug(f"JSON 解析错误 [{tag}]: {e}")
                except Exception as e:
                    self.get_logger().warning(f"处理数据错误 [{tag}]: {e}")
        except Exception as e:
            self.get_logger().error(f"读取线程错误 [{tag}] ({topic}): {e}")

    def _stderr_reader(self, proc, topic, tag):
        """读取子进程 stderr，捕获错误信息"""
        try:
            for line in proc.stderr:
                line = line.strip()
                if line:
                    self.get_logger().error(f"gz stderr [{tag}]: {line}")
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────
    #  cmd_vel 桥接 (ROS2 → Gazebo)
    # ──────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg):
        """ROS2 Twist → gz.msgs.Twist, 直接发布到 Gazebo"""
        payload = f'linear: {{x: {msg.linear.x}, y: {msg.linear.y}, z: {msg.linear.z}}}, angular: {{x: {msg.angular.x}, y: {msg.angular.y}, z: {msg.angular.z}}}'
        cmd = [
            "gz", "topic", "-t", self._cmd_vel_topic,
            "-m", "gz.msgs.Twist", "-p", payload,
        ]
        try:
            subprocess.run(
                cmd, capture_output=True, text=True, timeout=2,
                env=dict(os.environ, GZ_IP="127.0.0.1"),
            )
        except Exception as e:
            self.get_logger().warning(f"cmd_vel 发送失败: {e}")

    # ──────────────────────────────────────────────────────────────
    #  生命周期
    # ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        """清理子进程"""
        for proc in self._processes:
            try:
                proc.terminate()
                proc.wait(timeout=2)
            except Exception:
                proc.kill()
        super().destroy_node()

    # ──────────────────────────────────────────────────────────────
    #  JSON → ROS2 辅助方法（兼容 camelCase / snake_case）
    # ──────────────────────────────────────────────────────────────

    def _secs_to_ros_time(self, sec, nsec):
        t = TimeMsg()
        t.sec = int(sec) if sec else 0
        t.nanosec = int(nsec) if nsec else 0
        return t

    def _get_field(self, data, *keys):
        """安全地从嵌套 dict 中取值，同时尝试 camelCase 和 snake_case"""
        d = data
        for k in keys:
            if isinstance(d, dict):
                # 先直接匹配
                if k in d:
                    d = d[k]
                # 再尝试 camelCase 变体
                elif "_" in k:
                    camel = _to_camel(k)
                    if camel in d:
                        d = d[camel]
                    else:
                        return None
                else:
                    return None
            else:
                return None
        return d

    def _safe_get(self, data, key, default=None):
        """安全取值，同时尝试 camelCase 和 snake_case"""
        if key in data:
            return data[key]
        if "_" in key:
            camel = _to_camel(key)
            if camel in data:
                return data[camel]
        return default

    def _get_vec3(self, data, *keys):
        """从 JSON 中提取 Vector3"""
        v = self._get_field(data, *keys)
        if v and isinstance(v, dict):
            return (
                float(self._safe_get(v, "x", 0)),
                float(self._safe_get(v, "y", 0)),
                float(self._safe_get(v, "z", 0)),
            )
        return (0.0, 0.0, 0.0)

    def _get_quat(self, data, *keys):
        """从 JSON 中提取 Quaternion"""
        q = self._get_field(data, *keys)
        if q and isinstance(q, dict):
            return (
                float(self._safe_get(q, "x", 0)),
                float(self._safe_get(q, "y", 0)),
                float(self._safe_get(q, "z", 0)),
                float(self._safe_get(q, "w", 1)),
            )
        return (0.0, 0.0, 0.0, 1.0)

    def _get_stamp(self, data):
        """从 JSON 中提取时间戳"""
        stamp = self._get_field(data, "header", "stamp")
        if stamp:
            return self._secs_to_ros_time(
                stamp.get("sec"), stamp.get("nsec")
            )
        return self.get_clock().now().to_msg()

    # ──────────────────────────────────────────────────────────────
    #  各传感器处理函数
    # ──────────────────────────────────────────────────────────────

    def _handle_gps(self, data):
        """GPS JSON → NavSatFix (camelCase: latitudeDeg, longitudeDeg)"""
        msg = NavSatFix()
        msg.header = Header(stamp=self._get_stamp(data), frame_id="gps_link")

        msg.latitude = float(self._safe_get(data, "latitude_deg", 0))
        msg.longitude = float(self._safe_get(data, "longitude_deg", 0))
        msg.altitude = float(self._safe_get(data, "altitude", 0))

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.position_covariance[0] = 1.0
        msg.position_covariance[4] = 1.0
        msg.position_covariance[8] = 1.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if self._first_msg["gps"]:
            self._first_msg["gps"] = False
            self.get_logger().info(
                f"🌐 首帧 GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}"
            )

        self.pub_gps.publish(msg)

    def _handle_imu(self, data):
        """IMU JSON → Imu (camelCase: angularVelocity, linearAcceleration)"""
        msg = Imu()
        msg.header = Header(stamp=self._get_stamp(data), frame_id="imu_link")

        ox, oy, oz, ow = self._get_quat(data, "orientation")
        msg.orientation = Quaternion(x=ox, y=oy, z=oz, w=ow)

        ax, ay, az = self._get_vec3(data, "angular_velocity")
        msg.angular_velocity = Vector3(x=ax, y=ay, z=az)

        lx, ly, lz = self._get_vec3(data, "linear_acceleration")
        msg.linear_acceleration = Vector3(x=lx, y=ly, z=lz)

        msg.orientation_covariance[0] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.linear_acceleration_covariance[0] = 0.01

        if self._first_msg["imu"]:
            self._first_msg["imu"] = False
            self.get_logger().info(
                f"📡 首帧 IMU: orientation=({ox:.3f},{oy:.3f},{oz:.3f},{ow:.3f}), "
                f"ang_vel=({ax:.3f},{ay:.3f},{az:.3f})"
            )

        self.pub_imu.publish(msg)

    def _handle_mag(self, data):
        """磁力计 JSON → MagneticField (camelCase: fieldTesla)"""
        msg = MagneticField()
        msg.header = Header(
            stamp=self._get_stamp(data), frame_id="magnetometer_link"
        )

        fx, fy, fz = self._get_vec3(data, "field_tesla")
        msg.magnetic_field = Vector3(x=fx, y=fy, z=fz)

        if self._first_msg["mag"]:
            self._first_msg["mag"] = False
            self.get_logger().info(
                f"🧲 首帧 Mag: field=({fx:.6f},{fy:.6f},{fz:.6f})"
            )

        self.pub_mag.publish(msg)

    def _handle_odom(self, data):
        """里程计 JSON → Odometry"""
        msg = Odometry()
        msg.header = Header(stamp=self._get_stamp(data), frame_id="odom")
        msg.child_frame_id = "base_link"

        px, py, pz = self._get_vec3(data, "pose", "position")
        msg.pose.pose.position = Point(x=px, y=py, z=pz)

        ox, oy, oz, ow = self._get_quat(data, "pose", "orientation")
        msg.pose.pose.orientation = Quaternion(x=ox, y=oy, z=oz, w=ow)

        vx, vy, vz = self._get_vec3(data, "twist", "linear")
        msg.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)

        wx, wy, wz = self._get_vec3(data, "twist", "angular")
        msg.twist.twist.angular = Vector3(x=wx, y=wy, z=wz)

        if self._first_msg["odom"]:
            self._first_msg["odom"] = False
            self.get_logger().info(
                f"🚗 首帧 Odom: pos=({px:.3f},{py:.3f},{pz:.3f}), "
                f"vel=({vx:.3f},{vy:.3f},{vz:.3f})"
            )

        self.pub_odom.publish(msg)

    def _handle_scan(self, data):
        """LaserScan JSON → LaserScan"""
        msg = LaserScan()
        msg.header = Header(stamp=self._get_stamp(data), frame_id="lidar_link")

        msg.angle_min = float(self._safe_get(data, "angle_min", -3.14159))
        msg.angle_max = float(self._safe_get(data, "angle_max", 3.14159))
        msg.angle_increment = float(self._safe_get(data, "angle_step", 0.01745))
        msg.time_increment = float(self._safe_get(data, "time_increment", 0.0))
        msg.scan_time = float(self._safe_get(data, "scan_time", 0.1))
        msg.range_min = float(self._safe_get(data, "range_min", 0.1))
        msg.range_max = float(self._safe_get(data, "range_max", 10.0))

        ranges_raw = self._safe_get(data, "ranges")
        if ranges_raw and isinstance(ranges_raw, list):
            msg.ranges = [float(r) if r is not None else float('inf') for r in ranges_raw]
        else:
            msg.ranges = []

        intensities_raw = self._safe_get(data, "intensities")
        if intensities_raw and isinstance(intensities_raw, list):
            msg.intensities = [float(i) for i in intensities_raw]
        else:
            msg.intensities = []

        if self._first_msg["scan"]:
            self._first_msg["scan"] = False
            self.get_logger().info(
                f"📏 首帧 Scan: {len(msg.ranges)} ranges, "
                f"angle=[{msg.angle_min:.2f},{msg.angle_max:.2f}]"
            )

        self.pub_scan.publish(msg)

    def _handle_camera(self, data):
        """Camera Image JSON → Image"""
        msg = Image()
        msg.header = Header(stamp=self._get_stamp(data), frame_id="camera_link")

        msg.height = int(self._safe_get(data, "height", 480))
        msg.width = int(self._safe_get(data, "width", 640))
        msg.encoding = self._safe_get(data, "pixel_format_type", "rgb8")
        # 统一 encoding 格式
        if msg.encoding in ("R8G8B8", "rgb8", "RGB8"):
            msg.encoding = "rgb8"
        elif msg.encoding in ("B8G8R8", "bgr8", "BGR8"):
            msg.encoding = "bgr8"

        msg.is_bigendian = False
        msg.step = msg.width * 3  # RGB=3 bytes/pixel

        data_raw = self._safe_get(data, "data")
        if data_raw:
            if isinstance(data_raw, str):
                # Base64 编码的图片数据
                import base64
                try:
                    msg.data = base64.b64decode(data_raw)
                except Exception:
                    msg.data = data_raw.encode('latin-1') if hasattr(data_raw, 'encode') else b''
            elif isinstance(data_raw, list):
                msg.data = bytes(data_raw)
            else:
                msg.data = b''
        else:
            msg.data = b''

        if self._first_msg["camera"]:
            self._first_msg["camera"] = False
            self.get_logger().info(
                f"📷 首帧 Camera: {msg.width}x{msg.height}, "
                f"encoding={msg.encoding}, data_len={len(msg.data)}"
            )

        self.pub_camera.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
