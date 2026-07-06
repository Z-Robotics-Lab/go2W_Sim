# agent_bridge HTTP API 合同

`scripts/nav/agent_bridge.py` 暴露的 localhost HTTP 接口。这是 **z-agent（或任何机器人
后端）与 Go2W 之间的正式接口合同**：agent 侧是无 ROS 依赖的进程，全部经此 HTTP 桥
操控/感知机器人。桥跑在 navstack 容器内，由 supervisor（`run_all_forever.sh`）托管、
死后重生。

- 监听：`127.0.0.1:8042`（仅回环，容器内 `--net=host` 故宿主同址可达）
- 编码：请求/响应均 `application/json`
- 版本纪律：schema **只增不改**（新字段追加、旧字段行为不动），向后兼容。

## sim / real 对称性

| 端点 | sim | real | 说明 |
|------|-----|------|------|
| `/pose` `/health` `/waypoint` `/explore*` | 有 | 有 | 机器人自身估计 + 控制，两侧对称 |
| `/gt` | **仅 sim** | 无 | 地面真值来自 SIM 的 `/ground_truth/pose`；真机没有全知真值 |

`/gt` 是 verify 谓词专用的独立裁判——执行者（agent/策略）无法伪造它。真机上验收改用
外部裁判（第三方定位/人工），合同里 `/gt` 在 real 返回 503。

---

## GET 端点

### `GET /pose`
机器人自己的 SLAM 位姿估计。
- 200: `{"x":f,"y":f,"z":f,"yaw":f,"stamp":f}` — `stamp` 是 sim 时钟秒（msg header）
- 503: `{"error":"no pose yet"}` — 尚未收到 `/state_estimation`

### `GET /gt`  （仅 sim）
地面真值位姿（`/ground_truth/pose`）。字段同 `/pose`。
- 200: `{"x":f,"y":f,"z":f,"yaw":f,"stamp":f}`
- 503: `{"error":"no gt yet"}` — real 环境或 sim 未发真值时

### `GET /health`
聚合桥自己可见的状态，**始终 200**（探针要能读到"各话题多久没数据"）。
```json
{
  "ok": true,
  "pose":  {"present": true,  "age_s": 0.12},
  "gt":    {"present": true,  "age_s": 0.13},
  "grasp": {"present": false, "age_s": null}
}
```
- `age_s` 是**墙钟**秒（`now_wall - 收帧时刻`），不是 sim 时钟——避免 sim/墙钟偏斜。
- `present=false` 表示该话题从未收到过一帧。
- `grasp` 为占位（present=false, age_s=null），抓取接入后填（任务③）。

### `GET /explore_progress`
探索进度与 nav 归属，**始终 200**（值可能为 null 表示未探索）。
```json
{
  "explored_volume": 12.5,
  "volume_stamp": 1751760000.4,
  "volume_age_s": 0.4,
  "finished": false,
  "nav_owner": "explore"
}
```
- `explored_volume` (m³): 来自 `visualization_tools` 的 `/explored_volume`(Float32)——
  独立裁判，执行者不可伪造（感知栈按点云增量算）。未探索时 `null`。
- `volume_stamp`: 墙钟收帧时刻；`volume_age_s`: 距今墙钟秒。
- `finished`: 来自 TARE 的 `/exploration_finish`(Bool)；一旦 true 记住不回退，
  直到下次 `POST /explore` 复位。
- `nav_owner`: `idle` | `goto` | `explore`（见互斥语义）。

---

## POST 端点

### `POST /waypoint`
发布 `/way_point`(map 帧) 手动导航目标。
- 请求: `{"x":f,"y":f}`
- 200: `{"ok":true,"x":f,"y":f}`
- 409: `{"error":"exploration running"}` — `nav_owner==explore` 时（见互斥）
- 400: `{"error":"<msg>"}` — 缺 x/y 或 body 畸形

副作用：成功后 `nav_owner<-goto`，记录发布墙钟时刻（用于 `/explore` 的 30s 冷却）。

### `POST /explore`
发布 `/start_exploration`(Bool true) 触发 TARE 探索规划器。
- 请求: `{}`（无参；空 body 亦可）
- 200: `{"ok":true,"nav_owner":"explore"}`
- 409: `{"error":"goto in progress","cooldown_s":f}` — `nav_owner==goto` 且最近
  waypoint < 30s 时（狗可能正朝手动目标走，冷却期内不许探索抢占）
- 400: `{"error":"<msg>"}` — body 畸形

副作用：成功后 `nav_owner<-explore`，`finished` 复位为 false（新一轮探索）。

> **话题真名**：TARE 运行时订阅 `/start_exploration`（`indoor_small.yaml` 的
> `sub_start_exploration_topic_` 覆盖了 C++ code default `/exploration_start`）。
> 桥发的就是 `/start_exploration`。

### `POST /explore_stop`
发布 `/start_exploration`(Bool false) + `nav_owner<-idle`。
- 请求: `{}`
- 200: `{"ok":true,"nav_owner":"idle","note":"best-effort: TARE may ignore Bool(false)"}`

> **⚠ best-effort，非硬停**：当前 TARE 的 `ExplorationStartCallback` 只在 `data==true`
> 时置位、**忽略 Bool(false)**，且无任何话题能把 `start_exploration_` 复位。故这条 false
> 对 TARE 是 no-op——桥侧仅解锁互斥（waypoint 可再抢），TARE 仍可能继续发 `/way_point`。
> 真正停止 TARE 目前只能靠 `NAV_MODE=waypoint` 重启 system 链换掉 launch。**需实测确认。**

---

## 互斥语义（nav_owner 状态机）

`/way_point` 有**两个生产者**：localPlanner（手动 waypoint 经桥）与 TARE（探索时自动发）。
二者会互抢同一话题。桥维护 `nav_owner ∈ {idle, goto, explore}` 做互斥。

```
         POST /waypoint (owner!=explore)
   ┌──────────────────────────────────────┐
   │                                       ▼
[idle]───POST /explore────────────────►[explore]
   ▲                                       │
   │  POST /explore_stop  ◄────────────────┘
   │                                       
   │         POST /waypoint                
[idle]────────────────────────────────►[goto]
   ▲                                       │
   └──────  30s 冷却过后 /explore 抢占 ─────┘
            （或显式 /explore_stop 硬释放）
```

迁移规则（全部在 HTTP 线程串行发生，GIL 下单键读改写原子）：

| 当前 owner | 请求 | 结果 |
|-----------|------|------|
| any | `POST /waypoint` 且 owner==explore | **409** exploration running |
| idle / goto | `POST /waypoint` | 200，owner←goto，记 waypoint 时刻 |
| goto 且 waypoint<30s | `POST /explore` | **409** goto in progress |
| idle / explore / goto(冷却过) | `POST /explore` | 200，owner←explore，finished←false |
| any | `POST /explore_stop` | 200，owner←idle（best-effort 停 TARE） |

**goto 是软占用**：桥没有"到达目标"信号（到达检测在导航栈内，不在桥），所以 owner 不会
自动从 goto 回 idle。30s 冷却过后 `/explore` 即可抢占（等价软释放）；需要硬释放时调用方
显式发 `/explore_stop`。冷却常量 `EXPLORE_GOTO_COOLDOWN_S=30.0`。

---

## 话题映射速查（桥 ↔ ROS）

| HTTP | ROS 话题 | 类型 | 方向 | 生产者/消费者 |
|------|---------|------|------|--------------|
| GET /pose | `/state_estimation` | nav_msgs/Odometry | 订阅 | SLAM |
| GET /gt | `/ground_truth/pose` | geometry_msgs/PoseStamped | 订阅 | SIM（仅 sim） |
| GET /explore_progress.explored_volume | `/explored_volume` | std_msgs/Float32 | 订阅 | visualization_tools |
| GET /explore_progress.finished | `/exploration_finish` | std_msgs/Bool | 订阅 | TARE |
| POST /waypoint | `/way_point` | geometry_msgs/PointStamped | 发布 | → localPlanner |
| POST /explore, /explore_stop | `/start_exploration` | std_msgs/Bool | 发布 | → TARE |

> 注：`/explored_areas`(PointCloud2) 也由 visualization_tools 发，但桥不订阅它
> （体积标量 `/explored_volume` 已足够做进度裁判）。
