#!/usr/bin/env python3
"""PointCloud2 -> livox_ros_driver2/CustomMsg 转换节点。

Isaac RTX Lidar 出 sensor_msgs/PointCloud2（/lidar/points，10Hz 全帧）；
arise_slam_mid360 的 livox 分支要 CustomMsg（/lidar/scan，带每点 offset_time）。
offset_time 合成：按点序均匀铺满一个扫描周期（rotary 近似下与方位角单调对应）。
跑在导航栈容器里（需要已 colcon build 出 livox_ros_driver2 的消息包）。
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from livox_ros_driver2.msg import CustomMsg, CustomPoint

SCAN_PERIOD_NS = 200_000_000  # 后备值。实际周期随 RTF 变（扫描完成与渲染帧耦合，
# 坑17深层）：标定日 0.2，另日实测 0.21-0.25 且抖动——硬编码会让去畸变时间轴
# 错位、SLAM z 乱漂（2026-07-06 实证 +0.5m/sim-min 游走）。故动态测：用相邻帧
# header 差作为本帧周期，钳 [0.12, 0.40] 防野值，首帧用后备值。
PERIOD_MIN_NS = 120_000_000
PERIOD_MAX_NS = 400_000_000


class Pc2ToLivox(Node):
    def __init__(self):
        super().__init__("pc2_to_livox")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(PointCloud2, "/lidar/points", self.cb, qos)
        self.pub = self.create_publisher(CustomMsg, "/lidar/scan", 5)
        self.get_logger().info("pc2_to_livox: /lidar/points -> /lidar/scan (CustomMsg)")

    def cb(self, msg: PointCloud2):
        # 解析 x/y/z(+intensity) 字段偏移
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ("x", "y", "z")):
            return
        has_i = "intensity" in offs
        n = msg.width * msg.height
        step = msg.point_step
        data = bytes(msg.data)

        out = CustomMsg()
        out.header = msg.header
        out.header.frame_id = "livox_frame"
        # RTX helper 的帧戳是扫描完成时刻，点 offset 又向后铺 -> 帧总在"未来"，
        # SLAM 等 IMU 覆盖等不到。回溯一个周期：帧戳=扫描起始，offset 铺到完成时刻。
        end_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        prev = getattr(self, "_prev_end_ns", None)
        self._prev_end_ns = end_ns
        period_ns = SCAN_PERIOD_NS
        if prev is not None:
            measured = end_ns - prev
            if PERIOD_MIN_NS <= measured <= PERIOD_MAX_NS:
                period_ns = measured
        start_ns = max(end_ns - period_ns, 0)
        out.header.stamp.sec = start_ns // 1_000_000_000
        out.header.stamp.nanosec = start_ns % 1_000_000_000
        out.timebase = start_ns
        pts = []
        unpack = struct.unpack_from
        dt = period_ns // max(n, 1)
        for i in range(n):
            base = i * step
            x = unpack("<f", data, base + offs["x"])[0]
            y = unpack("<f", data, base + offs["y"])[0]
            z = unpack("<f", data, base + offs["z"])[0]
            if x == 0.0 and y == 0.0 and z == 0.0:
                continue
            p = CustomPoint()
            p.x, p.y, p.z = x, y, z
            p.reflectivity = int(min(max(
                unpack("<f", data, base + offs["intensity"])[0] if has_i else 100.0, 0), 255))
            p.offset_time = i * dt
            p.line = i % 4          # Mid-360 4 线
            p.tag = 16              # 正常回波
            pts.append(p)
        out.points = pts
        out.point_num = len(pts)
        out.lidar_id = 1
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Pc2ToLivox()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
