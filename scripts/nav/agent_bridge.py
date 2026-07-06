#!/usr/bin/env python3
"""Agent <-> 机器人 HTTP 桥（跑在 navstack 容器，supervisor 托管）。

让 vector_os_nano（无 ROS 依赖的进程）通过 localhost HTTP 操控/感知 Go2W：
  GET  /pose            -> SLAM 位姿 {x,y,z,yaw,stamp}（机器人自己的估计）
  GET  /gt              -> 地面真值位姿（来自 SIM 的 /ground_truth/pose——verify 谓词
                           专用，执行者无法伪造；仅 sim 有此话题，real 无）
  GET  /health          -> 聚合状态 {pose/gt/grasp 各自 present + age_s（墙钟秒）}，始终 200
  GET  /explore_progress-> {explored_volume, volume_stamp, finished, nav_owner}
  POST /waypoint {x,y}  -> 发布 /way_point 手动导航目标（互斥：explore 占用时 409）
  POST /explore  {}     -> 发布 /start_exploration(Bool true) 触发 TARE 探索（互斥见下）
  POST /explore_stop {} -> 发布 /start_exploration(Bool false) + owner 归 idle
                           （注意：当前 TARE 忽略 Bool false，这是 best-effort，见文件末注）
监听 127.0.0.1:8042。schema 只增不改——P5.1 的 /pose /gt /health /waypoint 行为完全不动。

单一生产者冲突 —— /way_point 有两个生产者：localPlanner（手动 waypoint 经桥）与
TARE（探索时自动发）。二者会互抢同一话题，故桥维护 nav_owner 互斥状态机，见下。
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

# nav_owner 互斥：谁在驱动 /way_point。
#   idle   —— 无人占用，waypoint 与 explore 都可抢
#   goto   —— 手动 waypoint 导航中（POST /waypoint 置入；记 waypoint_recv 时刻）
#   explore—— TARE 探索中（POST /explore 置入；POST /explore_stop 释放回 idle）
# 迁移规则（全部在 HTTP 线程串行发生，GIL 下对单键读改写原子，无需额外锁）：
#   /waypoint : owner==explore          -> 409（探索中不许手动抢）
#               否则                     -> 发 waypoint, owner<-goto, 记 waypoint_recv
#   /explore  : owner==goto 且 waypoint <30s -> 409（刚下手动目标，冷却期内不许探索抢）
#               否则                     -> 发 start=true, owner<-explore
#   /explore_stop : 任意               -> 发 start=false, owner<-idle
# 说明 goto->idle 不自动：桥无"到达"信号（到达检测在导航栈内，不在桥）。故 goto 是软占用
#   ——30s 冷却过后 /explore 即可抢占，等价于软释放；需要硬释放时调用方显式发 /explore_stop。
EXPLORE_GOTO_COOLDOWN_S = 30.0
STALE_S = 5.0  # pose/gt 超龄即 503（僵尸桥/断流防护）

STATE = {
    "pose": None, "gt": None,
    "explored_volume": None, "volume_stamp": None,
    "exploration_finished": False,
    "nav_owner": "idle",
    "waypoint_recv": None,  # 墙钟：最近一次 POST /waypoint 成功发布的时刻
    # 抓取管线（任务③）：箱子 GT / 夹持中心 GT / 臂关节实测+目标 / 状态机状态
    "object": None, "ee": None, "arm": None, "grasp_status": None,
}


def ros_main():
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

    def on_explored_volume(m: Float32):
        # visualization_tools 发 /explored_volume(Float32)：已探索体积 m^3，独立裁判
        # （执行者无法伪造——由感知栈按点云增量算）。stamp 用墙钟收帧时刻（话题无 header）。
        STATE["explored_volume"] = float(m.data)
        STATE["volume_stamp"] = time.time()

    def on_exploration_finish(m: Bool):
        # TARE 发 /exploration_finish(Bool)：探索完成信号。一旦 true 记住不回退（本轮探索
        # 已达完成；下次 POST /explore 重新触发时由调用方语义决定是否复位——见 do_POST）。
        # 完成即释放互斥：TARE 完成后不再发 /way_point，owner 卡在 explore 会让手动
        # 导航被 409 拒绝到永远（2026-07-06 实证：探索完成后机器人静止、goto 被锁）。
        if m.data:
            STATE["exploration_finished"] = True
            if STATE.get("nav_owner") == "explore":
                STATE["nav_owner"] = "idle"

    def on_box(m: Odometry):
        p, v = m.pose.pose.position, m.twist.twist.linear
        STATE["object"] = {"x": p.x, "y": p.y, "z": p.z,
                           "vx": v.x, "vy": v.y, "vz": v.z,
                           "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}
        STATE["object_recv"] = time.time()

    def on_ee(m: PoseStamped):
        p = m.pose.position
        STATE["ee"] = {"x": p.x, "y": p.y, "z": p.z, "yaw": _yaw(m.pose.orientation),
                       "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}
        STATE["ee_recv"] = time.time()

    def on_js(m: JointState):
        cur = STATE.get("arm") or {}
        STATE["arm"] = {"names": list(m.name), "pos": list(m.position),
                        "cmd": cur.get("cmd"),
                        "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}
        STATE["arm_recv"] = time.time()

    def on_jc(m: JointState):
        cur = STATE.get("arm") or {}
        cur["cmd"] = list(m.position)
        STATE["arm"] = cur

    def on_gs(m: String):
        STATE["grasp_status"] = {"status": m.data.split(";")[0], "raw": m.data}
        STATE["grasp_recv"] = time.time()

    node.create_subscription(Odometry, "/objects/box/odom", on_box, 5)
    node.create_subscription(PoseStamped, "/piper/ee_pose", on_ee, 5)
    node.create_subscription(JointState, "/piper/state", on_js, 5)
    node.create_subscription(JointState, "/piper/cmd", on_jc, 5)
    node.create_subscription(String, "/piper/grasp_status", on_gs, 5)
    STATE["grasp_pub"] = node.create_publisher(String, "/piper/grasp_cmd", 5)
    node.create_subscription(Odometry, "/state_estimation", on_odom, 5)
    node.create_subscription(PoseStamped, "/ground_truth/pose", on_gt, 5)
    node.create_subscription(Float32, "/explored_volume", on_explored_volume, 5)
    node.create_subscription(Bool, "/exploration_finish", on_exploration_finish, 5)
    STATE["wp_pub"] = node.create_publisher(PointStamped, "/way_point", 5)
    # 启动话题真名 /start_exploration —— indoor_small.yaml 的 sub_start_exploration_topic_
    # 覆盖了 C++ 里的 code default /exploration_start；运行时 TARE 订阅的是 yaml 里这个。
    STATE["explore_pub"] = node.create_publisher(Bool, "/start_exploration", 5)
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

    def _read_json_body(self):
        """读 POST body 为 dict；空 body 视作 {}（/explore、/explore_stop 无参）。"""
        n = int(self.headers.get("Content-Length", 0))
        if n <= 0:
            return {}
        raw = self.rfile.read(n)
        if not raw.strip():
            return {}
        return json.loads(raw)

    def do_GET(self):
        key = self.path.strip("/")
        if key in ("pose", "gt", "object", "ee", "arm", "grasp_status"):
            v = STATE.get(key)
            # 陈旧守卫（护城河）：ROS 侧断流时绝不供陈旧数据给 verify 谓词——
            # 位姿与抓取 oracle 数据源超过 STALE_S 秒未更新返回 503（2026-07-06
            # 僵尸桥实证：rclpy 接管 SIGTERM 只死 ROS 线程，HTTP 带冻结状态应答）。
            if v and key in ("pose", "gt", "object", "ee", "arm"):
                recv = STATE.get(f"{key}_recv")
                age = None if recv is None else time.time() - recv
                if age is None or age > STALE_S:
                    self._json(503, {"error": f"{key} stale", "age_s": age})
                    return
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
                "grasp": {"present": STATE.get("arm") is not None,
                          "age_s": self._age("arm_recv")},
            })
        elif key == "explore_progress":
            # 探索进度（独立裁判读值）：explored_volume 来自 visualization_tools（执行者不可
            # 伪造），finished 来自 TARE /exploration_finish。始终 200——值可能为 null（未探索）。
            self._json(200, {
                "explored_volume": STATE.get("explored_volume"),
                "volume_stamp": STATE.get("volume_stamp"),
                "volume_age_s": self._age("volume_stamp"),
                "finished": STATE.get("exploration_finished", False),
                "nav_owner": STATE.get("nav_owner", "idle"),
            })
        else:
            self._json(404, {"error": "unknown"})

    def do_POST(self):
        path = self.path.rstrip("/")
        if path == "/waypoint":
            self._post_waypoint()
        elif path == "/explore":
            self._post_explore()
        elif path == "/explore_stop":
            self._post_explore_stop()
        elif path == "/grasp":
            # 触发 Isaac 侧 PiPER 抓取状态机（臂动作不占 /way_point，无互斥交互；
            # 进度经 GET /grasp_status 轮询——状态机自报，最终裁决靠 holding_object
            # oracle 读 GT，永不采信状态机的 done）。
            try:
                req = self._read_json_body()
                msg = String()
                msg.data = str(req.get("object", "box"))
                STATE["grasp_pub"].publish(msg)
                self._json(200, {"ok": True, "object": msg.data})
            except Exception as e:  # noqa: BLE001 — 桥边界
                self._json(400, {"error": str(e)})
        else:
            self._json(404, {"error": "unknown"})

    def _post_waypoint(self):
        # 互斥：探索占用 /way_point 时，手动 waypoint 会与 TARE 互抢——拒绝。
        if STATE.get("nav_owner") == "explore":
            self._json(409, {"error": "exploration running"})
            return
        try:
            req = self._read_json_body()
            wp = PointStamped()
            wp.header.frame_id = "map"
            wp.point.x, wp.point.y = float(req["x"]), float(req["y"])
            STATE["wp_pub"].publish(wp)
            STATE["nav_owner"] = "goto"          # 手动导航占用（软占用，见文件头）
            STATE["waypoint_recv"] = time.time()  # 记时刻用于 /explore 的 30s 冷却
            self._json(200, {"ok": True, "x": wp.point.x, "y": wp.point.y})
        except Exception as e:  # noqa: BLE001 — 桥边界，回错误给调用方
            self._json(400, {"error": str(e)})

    def _post_explore(self):
        # 互斥：刚下过手动 waypoint 且在 30s 冷却期内，不许探索抢占（狗可能正朝目标走）。
        owner = STATE.get("nav_owner")
        last_wp = STATE.get("waypoint_recv")
        if owner == "goto" and last_wp is not None \
                and (time.time() - last_wp) < EXPLORE_GOTO_COOLDOWN_S:
            self._json(409, {"error": "goto in progress",
                             "cooldown_s": round(EXPLORE_GOTO_COOLDOWN_S - (time.time() - last_wp), 1)})
            return
        try:
            self._read_json_body()  # 无参，但容错解析（畸形 body -> 400）
            msg = Bool()
            msg.data = True
            STATE["explore_pub"].publish(msg)
            STATE["nav_owner"] = "explore"
            STATE["exploration_finished"] = False  # 新一轮探索：复位上一轮的 finished
            self._json(200, {"ok": True, "nav_owner": "explore"})
        except Exception as e:  # noqa: BLE001
            self._json(400, {"error": str(e)})

    def _post_explore_stop(self):
        # best-effort 停止：发 /start_exploration(Bool false) + owner 归 idle。
        # 当前 TARE 的 ExplorationStartCallback 只在 data==true 时置位，忽略 false——
        # 即这条 false 不会真正停下 TARE（需实测验证 / 未来栈支持）。owner 归 idle 让
        # 桥侧互斥解锁（waypoint 可再抢），但 TARE 仍可能继续发 /way_point。见文件末注 [1]。
        try:
            self._read_json_body()
            msg = Bool()
            msg.data = False
            STATE["explore_pub"].publish(msg)
            STATE["nav_owner"] = "idle"
            self._json(200, {"ok": True, "nav_owner": "idle",
                             "note": "best-effort: TARE may ignore Bool(false)"})
        except Exception as e:  # noqa: BLE001
            self._json(400, {"error": str(e)})


def main():
    # HTTP 在守护线程、rclpy.spin 在主线程：rclpy 接管 SIGTERM/SIGINT，若 spin 在
    # 子线程，信号只会杀 ROS 半边、HTTP 半边带冻结 STATE 继续应答（僵尸桥，
    # 2026-07-06 实证 55s 陈旧数据仍 200）。spin 在主线程则信号 -> spin 抛
    # ExternalShutdownException -> 进程整体退出 -> supervisor 干净重生。
    srv = HTTPServer(("127.0.0.1", 8042), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("[BRIDGE] http://127.0.0.1:8042  (/pose /gt /health /explore_progress "
          "POST /waypoint /explore /explore_stop)", flush=True)
    try:
        ros_main()
    finally:
        srv.shutdown()


if __name__ == "__main__":
    main()

# 注 [1] TARE 停止能力（真相源核对 sensor_coverage_planner_ground.cpp）：
#   ExplorationStartCallback 只有 `if (start_msg->data) start_exploration_ = true;`——
#   收到 Bool(false) 不做任何事，且没有任何话题/参数能把 start_exploration_ 复位。
#   故 /explore_stop 发的 false 对 TARE 是 no-op；桥侧仅解锁互斥。真正停止 TARE 目前只能
#   靠 NAV_MODE 切回 waypoint 重启 system 链（换掉 launch）。需实测确认，见 STATUS 遗留。
