#!/usr/bin/env python3
"""PointCloud2 -> livox_ros_driver2/CustomMsg 转换节点（增量聚合版）。

Isaac RTX Lidar 以 fullScan=False 增量模式发 /lidar/points：每个渲染拍一条消息，
内容是该拍扫过的方位片——**到达顺序即时间顺序**（帧戳=该拍仿真时刻）。
本节点把增量片缓冲聚合成 ~0.1 s 的 CustomMsg 帧（/lidar/scan），每个点的
offset_time 用其所属增量片的真实时戳计算——运动中的去畸变时间轴因此正确。

为什么不用整帧模式：fullScan=True 的点序完全非时序（2026-07-06 解剖一帧实测
方位角前向率 48.5%=随机——发射器状态交织），按索引铺 offset_time 等于给每个点
随机时间戳；静止无感，一运动去畸变毁灭性出错，SLAM z 俯冲 0.4m/3s，地形窗口
被打空、localPlanner 只能发空路径（坑34，走走停停爬行的根因）。
跑在导航栈容器里（需要已 colcon build 出 livox_ros_driver2 的消息包）。
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from livox_ros_driver2.msg import CustomMsg, CustomPoint

# 聚合窗口（仿真纳秒）：攒够该时长的增量片就发一帧。0.1s ≈ Mid-360 原生 10Hz 帧率，
# 前端特征提取的点数密度与真实驱动同量级。
FRAME_SPAN_NS = 100_000_000
# 缓冲保护：增量片长时间断流（Isaac 暂停/重启）时丢弃陈旧缓冲，避免跨断点拼帧。
STALE_GAP_NS = 500_000_000


class Pc2ToLivox(Node):
    def __init__(self):
        super().__init__("pc2_to_livox")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(PointCloud2, "/lidar/points", self.cb, qos)
        self.pub = self.create_publisher(CustomMsg, "/lidar/scan", 5)
        self._buf = []          # [(tick_ns, [CustomPoint,...]), ...] 按到达序
        self._buf_start = None  # 当前聚合窗口起始时戳（ns）
        self.get_logger().info(
            "pc2_to_livox(incremental): /lidar/points -> /lidar/scan (CustomMsg)")

    def cb(self, msg: PointCloud2):
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ("x", "y", "z")):
            return
        has_i = "intensity" in offs
        n = msg.width * msg.height
        step = msg.point_step
        data = bytes(msg.data)
        tick_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

        # 断流保护：与缓冲末片间隔过大 -> 弃旧起新
        if self._buf and abs(tick_ns - self._buf[-1][0]) > STALE_GAP_NS:
            self._buf = []
            self._buf_start = None

        unpack = struct.unpack_from
        pts = []
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
                unpack("<f", data, base + offs["intensity"])[0] if has_i else 100.0,
                0), 255))
            p.line = i % 4   # Mid-360 4 线
            p.tag = 16       # 正常回波
            pts.append(p)
        if pts:
            if self._buf_start is None:
                self._buf_start = tick_ns
            self._buf.append((tick_ns, pts))

        # 攒满窗口 -> 出帧。帧戳=窗口首片时刻（扫描起始语义，SLAM 的 IMU 覆盖成立），
        # 每点 offset = 其增量片时刻 - 帧戳（同片共享，粒度=渲染拍 10ms，足够去畸变）。
        if self._buf_start is not None and tick_ns - self._buf_start >= FRAME_SPAN_NS:
            out = CustomMsg()
            out.header = msg.header
            out.header.frame_id = "livox_frame"
            out.header.stamp.sec = self._buf_start // 1_000_000_000
            out.header.stamp.nanosec = self._buf_start % 1_000_000_000
            out.timebase = self._buf_start
            all_pts = []
            for t_ns, chunk in self._buf:
                off = max(t_ns - self._buf_start, 0)
                for p in chunk:
                    p.offset_time = off
                all_pts.extend(chunk)
            out.points = all_pts
            out.point_num = len(all_pts)
            out.lidar_id = 1
            self.pub.publish(out)
            self._buf = []
            self._buf_start = None


def main():
    rclpy.init()
    node = Pc2ToLivox()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
