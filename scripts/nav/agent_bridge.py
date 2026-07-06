#!/usr/bin/env python3
"""Agent <-> 机器人 HTTP 桥（跑在 navstack 容器，supervisor 托管）。

让 vector_os_nano（无 ROS 依赖的进程）通过 localhost HTTP 操控/感知 Go2W：
  GET  /pose      -> SLAM 位姿 {x,y,z,stamp}（机器人自己的估计）
  GET  /gt        -> 地面真值位姿（来自 SIM 的 /ground_truth/pose——verify 谓词专用，
                     执行者无法伪造）
  GET  /health    -> 聚合状态 {pose/gt/grasp 各自 present + age_s（墙钟秒）}，始终 200
  POST /waypoint  -> {"x":..,"y":..} 发布 /way_point 导航目标
监听 127.0.0.1:8042。
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry

STATE = {"pose": None, "gt": None}


def ros_thread():
    rclpy.init()
    node = rclpy.create_node("agent_bridge")

    def _yaw(q):
        import math
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(m: Odometry):
        p = m.pose.pose.position
        STATE["pose"] = {"x": p.x, "y": p.y, "z": p.z, "yaw": _yaw(m.pose.pose.orientation),
                         "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}
        # /health 用的墙钟收帧时刻：age = now_wall - recv_wall（避 sim-clock 与墙钟偏斜；
        # msg.header.stamp 是仿真时钟，直接拿它算 age 会错）。仅新增键，不动 /pose 返回体。
        STATE["pose_recv"] = time.time()

    def on_gt(m: PoseStamped):
        p = m.pose.position
        STATE["gt"] = {"x": p.x, "y": p.y, "z": p.z, "yaw": _yaw(m.pose.orientation),
                       "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}
        STATE["gt_recv"] = time.time()

    node.create_subscription(Odometry, "/state_estimation", on_odom, 5)
    node.create_subscription(PoseStamped, "/ground_truth/pose", on_gt, 5)
    STATE["wp_pub"] = node.create_publisher(PointStamped, "/way_point", 5)
    rclpy.spin(node)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # 静默访问日志
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _age(self, recv_key):
        """墙钟秒数：自上次收到该话题至今；从未收到返回 None。"""
        t = STATE.get(recv_key)
        return None if t is None else round(time.time() - t, 3)

    def do_GET(self):
        key = self.path.strip("/")
        if key in ("pose", "gt"):
            v = STATE.get(key)
            self._json(200 if v else 503, v or {"error": f"no {key} yet"})
        elif key == "health":
            # 聚合桥自己可见的状态；始终 200（探针要能读到"各话题多久没数据"）。
            # schema 只增不改：现有端点行为完全不动，向后兼容。
            self._json(200, {
                "ok": True,
                "pose": {"present": STATE.get("pose") is not None,
                         "age_s": self._age("pose_recv")},
                "gt": {"present": STATE.get("gt") is not None,
                       "age_s": self._age("gt_recv")},
                "grasp": {"present": False, "age_s": None},  # 抓取话题占位（任务③接入后填）
            })
        else:
            self._json(404, {"error": "unknown"})

    def do_POST(self):
        if self.path.rstrip("/") == "/waypoint":
            n = int(self.headers.get("Content-Length", 0))
            try:
                req = json.loads(self.rfile.read(n))
                wp = PointStamped()
                wp.header.frame_id = "map"
                wp.point.x, wp.point.y = float(req["x"]), float(req["y"])
                STATE["wp_pub"].publish(wp)
                self._json(200, {"ok": True, "x": wp.point.x, "y": wp.point.y})
            except Exception as e:  # noqa: BLE001 — 桥边界，回错误给调用方
                self._json(400, {"error": str(e)})
        else:
            self._json(404, {"error": "unknown"})


def main():
    threading.Thread(target=ros_thread, daemon=True).start()
    print("[BRIDGE] http://127.0.0.1:8042  (/pose /gt POST /waypoint)", flush=True)
    HTTPServer(("127.0.0.1", 8042), Handler).serve_forever()


if __name__ == "__main__":
    main()
