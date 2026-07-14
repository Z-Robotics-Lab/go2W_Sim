#!/usr/bin/env bash
# 幂等拉起全链（宿主机执行）。铁律：Isaac 桥与 navstack 必须成对重启
#   （单侧重启后 fastdds 不重新发现旧对端，SLAM 空转——见 README 坑 23/restart_all.sh 头注）。
# 用法:
#   bash scripts/nav/bringup.sh            # 幂等拉起（已 green 直接短路退出）
#   bash scripts/nav/bringup.sh teardown   # 拆链（scoped，绝不 rosm nuke / 无差别 pkill）
#
# 相对 restart_all.sh 的增强（本脚本内联配对重启主体，替换其无限等待）:
#   - 幂等短路: 已 green 不动
#   - 防双开守卫: pgrep kit-python 查重 + free 可用内存 <20G 拒启
#   - Isaac 就绪 until 循环加 600s 硬上限，超时打 nav_bridge.log 尾 20 行后退出 1
#   - POLICY 路径参数化（GO2W_POLICY）+ 启动前容器内 test -f
#   - 每阶段落 logs/.bringup.phase（阶段名+时间戳，覆盖写）
#   - xhost +local: 起 RViz 前放行（失败只 warn）
# 注意: 不开 set -u（ROS 场景惯例；本脚本不 source setup.bash，但守此约定避免误伤）。
set -e

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
HERE="$(cd "$(dirname "$0")" && pwd)"
NAV="$REPO/refs/Navigation-Physical-Experiment"
LOG="$REPO/logs/nav_bridge.log"
PHASE_FILE="$REPO/logs/.bringup.phase"
MEM_MIN_GB="${GO2W_MEM_MIN_GB:-20}"     # 可用内存低于此值拒绝启动（防双开 OOM 共享 64G 宿主）
ISAAC_TIMEOUT_S="${GO2W_ISAAC_TIMEOUT_S:-600}"  # Isaac 就绪硬上限
NAV_TIMEOUT_S="${GO2W_NAV_TIMEOUT_S:-180}"       # 低 RTF Office 下 SLAM 首次收敛墙钟上限
REQUIRE_GUI="${GO2W_REQUIRE_GUI:-0}"              # 1=RViz/X11 也是启动成功合同
# 184 是合法且不常见的 Fast DDS domain；调用方仍可显式覆盖，但 Isaac、
# navstack、RViz、Z-Manip 和诊断命令必须共享同一个值。
ROS_DOMAIN_ID="${GO2W_ROS_DOMAIN_ID:-184}"
export ROS_DOMAIN_ID

# Resolve one bounded profile before either simulator or navigation starts. The
# exact values and robot config are forwarded to both XML (sim) and Python
# (real) child launches.
GO2W_NAV_PROFILE="${GO2W_NAV_PROFILE:-manipulation_tracking}"
_nav_profile_error=""
case "$GO2W_NAV_PROFILE" in
  general)
    _profile_max_speed=0.75
    _profile_autonomy_speed=0.60
    _profile_max_accel=1.50
    _profile_max_yaw_rate_deg_s=40.0
    _profile_stop_distance_threshold=0.10
    ;;
  manipulation_tracking)
    _profile_max_speed=0.25
    _profile_autonomy_speed=0.20
    _profile_max_accel=0.35
    _profile_max_yaw_rate_deg_s=15.0
    _profile_stop_distance_threshold=0.10
    ;;
  *)
    # Keep teardown available even if the caller misspelled an up-only profile.
    _nav_profile_error="unknown GO2W_NAV_PROFILE='$GO2W_NAV_PROFILE'"
    _profile_max_speed=0.75
    _profile_autonomy_speed=0.60
    _profile_max_accel=1.50
    _profile_max_yaw_rate_deg_s=40.0
    _profile_stop_distance_threshold=0.10
    ;;
esac
NAV_MAX_SPEED="${GO2W_NAV_MAX_SPEED:-$_profile_max_speed}"
NAV_AUTONOMY_SPEED="${GO2W_NAV_AUTONOMY_SPEED:-$_profile_autonomy_speed}"
NAV_MAX_ACCEL="${GO2W_NAV_MAX_ACCEL:-$_profile_max_accel}"
NAV_MAX_YAW_RATE_DEG_S="${GO2W_NAV_MAX_YAW_RATE_DEG_S:-$_profile_max_yaw_rate_deg_s}"
PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD="${GO2W_NAV_STOP_DISTANCE_THRESHOLD:-$_profile_stop_distance_threshold}"
NAV_SPEED="$NAV_AUTONOMY_SPEED"
NAV_ROBOT_CONFIG="${GO2W_NAV_ROBOT_CONFIG:-${NAV_ROBOT_CONFIG:-unitree/unitree_go2}}"
LOCAL_PLANNER_GOAL_REACHED_THRESHOLD="${LOCAL_PLANNER_GOAL_REACHED_THRESHOLD:-0.15}"
LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE="${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE:-0.20}"
# RL locomotion 策略（容器内路径；README 坑 26：差速在 Go2W 物理不可行，必须 RL 策略）。
# 默认指向 git 追踪的 assets/policies/；容器 bind-mount 仓库到 /workspace/go2w，
# 故此路径在容器内直接可见。
# 回滚锚: /workspace/go2w/assets/policies/go2w_flat_payload_3497/model_3497.pt
# 出厂锚: /workspace/go2w/assets/policies/go2w_flat_factory_1999/model_1999.pt
# 切换记录: 2026-07-07 载荷轮 model_5495 落地(⑤门形修正后全门过),见 docs/sim-plan.md
POLICY="${GO2W_POLICY:-/workspace/go2w/assets/policies/go2w_flat_payload_5495/model_5495.pt}"

_phase() {  # 记录当前阶段（覆盖写，供外部/事后诊断读）
  mkdir -p "$REPO/logs"
  echo "$1  $(date -Is)" > "$PHASE_FILE"
  echo "[bringup] $1"
}

_validate_nav_profile() {
  if [ -n "$_nav_profile_error" ]; then
    echo "[bringup] $_nav_profile_error" >&2
    return 2
  fi
  python3 "$HERE/validate_nav_profile.py" \
    --max-speed "$NAV_MAX_SPEED" \
    --autonomy-speed "$NAV_AUTONOMY_SPEED" \
    --bridge-speed "$NAV_SPEED" \
    --command-ramp "$NAV_MAX_ACCEL" \
    --yaw-rate-deg-s "$NAV_MAX_YAW_RATE_DEG_S" \
    --stop-distance "$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD" \
    --goal-threshold "$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD" \
    --obstacle-height "$LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE" \
    --robot-config "$NAV_ROBOT_CONFIG" \
    --config-root "$NAV/src/base_autonomy/local_planner/config"
}

_ros_graph_unique() {
  # 远端同 domain 的第二套传感器/SLAM 会让时间戳倒退并清空聚合缓冲。
  # 关键链路必须各自只有一个发布者，缺失或多发布者都 fail closed。
  docker exec navstack bash -c '
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    for topic in /clock /imu/data /lidar/points /lidar/scan /registered_scan /state_estimation /odom_base_link; do
      count="$(ros2 topic info "$topic" 2>/dev/null | sed -n "s/^Publisher count: //p")"
      if [ "$count" != "1" ]; then
        echo "$topic publisher_count=${count:-missing}" >&2
        exit 1
      fi
    done
  ' > "$REPO/logs/ros_graph_gate.log" 2>&1
}

_ros_stream_healthy() {
  docker exec navstack bash -c '
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    python3 /ws/ros_stream_gate.py --duration 30 \
      --min-clock 10 --min-imu 2 --min-raw 2 --min-custom 2 \
      --min-regscan 2 --min-state 2 --min-odom 2
  ' > "$REPO/logs/ros_stream_gate.log" 2>&1
}

_gui_healthy() {
  [ "$REQUIRE_GUI" != "1" ] || docker exec navstack pgrep -x rviz2 >/dev/null 2>&1
}

_local_planner_contract() {
  docker exec navstack bash -c '
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    value() {
      ros2 param get "$1" "$2" 2>/dev/null | sed -n "s/^Double value is: //p"
    }
    python3 - \
      "$(value /localPlanner maxSpeed)" \
      "$(value /localPlanner autonomySpeed)" \
      "$(value /localPlanner goalReachedThreshold)" \
      "$(value /localPlanner obstacleHeightThre)" \
      "$(value /pathFollower maxSpeed)" \
      "$(value /pathFollower autonomySpeed)" \
      "$(value /pathFollower maxAccel)" \
      "$(value /pathFollower maxYawRate)" \
      "$(value /pathFollower stopDisThre)" \
      "$NAV_MAX_SPEED" "$NAV_AUTONOMY_SPEED" "$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD" \
      "$LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE" "$NAV_MAX_ACCEL" \
      "$NAV_MAX_YAW_RATE_DEG_S" "$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD" <<"PY"
import math
import sys

actual = tuple(map(float, sys.argv[1:10]))
expected = tuple(map(float, (
    sys.argv[10], sys.argv[11], sys.argv[12], sys.argv[13],
    sys.argv[10], sys.argv[11], sys.argv[14], sys.argv[15], sys.argv[16],
)))
if not all(math.isfinite(value) for value in (*actual, *expected)):
    raise SystemExit(1)
goal_threshold = float(sys.argv[12])
stop_distance = float(sys.argv[16])
if goal_threshold <= 0.0 or goal_threshold >= 0.20:
    raise SystemExit(1)
if stop_distance <= 0.0 or stop_distance > 0.15 or stop_distance >= goal_threshold:
    raise SystemExit(1)
if any(abs(left - right) > 1e-9 for left, right in zip(actual, expected)):
    raise SystemExit(1)
PY
  ' > "$REPO/logs/local_planner_contract_gate.log" 2>&1
}

_runtime_dds_matches() {
  # 防止旧容器在不同 domain/scope 下仍因 HTTP freshness 被误判 already-up。
  local nav_env isaac_env
  nav_env="$(docker inspect navstack --format '{{range .Config.Env}}{{println .}}{{end}}' 2>/dev/null)" || return 1
  grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" <<<"$nav_env" || return 1
  for expected in \
    "GO2W_NAV_PROFILE=$GO2W_NAV_PROFILE" \
    "NAV_MAX_SPEED=$NAV_MAX_SPEED" \
    "NAV_AUTONOMY_SPEED=$NAV_AUTONOMY_SPEED" \
    "NAV_MAX_ACCEL=$NAV_MAX_ACCEL" \
    "NAV_MAX_YAW_RATE_DEG_S=$NAV_MAX_YAW_RATE_DEG_S" \
    "PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD=$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD" \
    "NAV_SPEED=$NAV_SPEED" \
    "NAV_ROBOT_CONFIG=$NAV_ROBOT_CONFIG" \
    "LOCAL_PLANNER_GOAL_REACHED_THRESHOLD=$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD" \
    "LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE=$LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE"; do
    grep -qx "$expected" <<<"$nav_env" || return 1
  done
  isaac_env="$(docker exec -u 0 go2w-isaac bash -c '
    pids="$(pgrep -f "kit/pytho[n].*warehouse_nav.py")"
    [ "$(wc -w <<<"$pids")" -eq 1 ] || exit 1
    pid="$pids"
    tr "\0" "\n" < "/proc/$pid/environ"
  ' 2>/dev/null)" || return 1
  grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" <<<"$isaac_env" || return 1
}

# ---------------------------------------------------------------------------
# teardown: scoped 拆链——精确打击 + 分级升级 + 逐级复核（NEVER-KILL-INFRA）
# ---------------------------------------------------------------------------
# 背景（2026-07-06 活体僵尸取证 DEBUG.md）：旧版 `pkill -9 …||true` 从不复核，
# status.sh 退出码被 `||true` 吞——工具"说关了"实为"发了信号没验证"，正是 CEO 抱怨点。
# kit 进程的死法多为逻辑活锁（Rl），偶发 D 态（-9 跳过 simulation_app.close() →
# GL/CUDA 上下文不释放）。故本 teardown：
#   1) 先 docker rm -f navstack（PID-1 supervisor 随容器消亡，RViz 不再被拉回）；
#   2) 容器内对 kit-python 分级升级：TERM→等→KILL→等→复核；
#   3) 仍存活（D 态兜底）→ docker restart go2w-isaac（终结整个 PID namespace，保留
#      容器满足"重建代价高"约束）→ 复核；
#   4) 成功判据 = 宿主 pgrep -f "kit/pytho[n]" 为空 且 status.sh l0=false；
#      失败退非零 + 打印残留进程表——绝不静默假成功。
# 铁律：字符类 kit/pytho[n] 防脚本自杀；只 exec 进目标容器 / 只 docker 容器级操作，
# 绝不在宿主上 pkill（会连坐 sibling 会话，NEVER-KILL-INFRA）。

KILL_PATTERN='kit/pytho[n]'                       # 目标 = Isaac kit python（字符类防自杀）
TD_GRACE_S="${GO2W_TD_GRACE_S:-5}"                # 每级 kill 后的复核等待秒数
DOCKER_STOP_GRACE_S="${GO2W_DOCKER_STOP_GRACE_S:-10}"  # docker restart 的 SIGTERM grace

# 宿主侧存活探针：pgrep 扫 /proc（宿主与容器共享内核，能看到容器进程）。
# 排除 pgrep 自身/本脚本（-f 会匹配到含 pattern 的命令行）用字符类已规避脚本自匹配，
# 但仍显式排除当前脚本 PID 以防边界情况。返回匹配到的 PID（可能多行），空=已灭。
_isaac_live_pids() {
  pgrep -f "$KILL_PATTERN" 2>/dev/null | grep -vx "$$" || true
}

# 容器内对目标进程发信号（scoped，只在 go2w-isaac 内）。$1 = 信号（TERM/KILL）。
_isaac_signal() {
  local sig="$1"
  docker exec -u 0 go2w-isaac bash -c \
    "pkill -${sig} -f \"${KILL_PATTERN}\" 2>/dev/null; true" 2>/dev/null || true
}

# 打印宿主+容器双侧残留进程表（失败诊断，绝不静默）。
_dump_residual() {
  echo "[teardown] ===== 残留进程诊断 =====" >&2
  echo "[teardown] 宿主侧 pgrep -af '$KILL_PATTERN':" >&2
  pgrep -af "$KILL_PATTERN" 2>/dev/null | grep -v "$$ " >&2 || echo "  （宿主侧空）" >&2
  echo "[teardown] 容器内 ps（kit/python/warehouse 相关，含 STAT/wchan）:" >&2
  docker exec go2w-isaac ps -eo pid,ppid,stat,wchan:20,cmd 2>/dev/null \
    | grep -Ei "python|kit|warehouse" | grep -v grep >&2 || echo "  （容器不可达或空）" >&2
}

teardown() {
  echo "[teardown] 拆链开始（scoped 精确打击 + 分级升级 + 逐级复核）"

  # ── 1) 先杀 navstack supervisor（RViz 不再回弹）──────────────────────────
  # RViz 挂在 navstack 的 PID-1 supervisor(run_all_forever.sh) 下、死后自动重生；
  # 必须先移除整个 navstack 容器（supervisor 随容器消亡），否则先杀 Isaac 时
  # RViz 仍被 supervisor 拉回（用户抱怨"关掉的 RViz 一直弹回"的根因）。
  echo "[teardown] [1/3] docker rm -f navstack（PID-1 supervisor 随容器消亡，RViz 停止回弹）"
  docker rm -f navstack >/dev/null 2>&1 || true

  # ── 2) 对 kit-python 分级升级：TERM → KILL → docker restart ──────────────
  # go2w-isaac 容器保留（重建代价高），只终结其中的 kit sim 进程。
  if [ -z "$(_isaac_live_pids)" ]; then
    echo "[teardown] [2/3] kit-python 已不存在（宿主 pgrep 为空），跳过击杀"
  else
    # 2a. SIGTERM（给 Isaac 机会走 simulation_app.close() 释放 GL/CUDA 上下文）
    echo "[teardown] [2/3a] SIGTERM kit-python（宿主残留 PID: $(_isaac_live_pids | tr '\n' ' ')）"
    _isaac_signal TERM
    sleep "$TD_GRACE_S"
    # 2b. 仍在 → SIGKILL
    if [ -n "$(_isaac_live_pids)" ]; then
      echo "[teardown] [2/3b] TERM 后仍存活 → SIGKILL（残留 PID: $(_isaac_live_pids | tr '\n' ' ')）"
      _isaac_signal KILL
      sleep "$TD_GRACE_S"
    else
      echo "[teardown] [2/3b] TERM 已生效，无需 SIGKILL"
    fi
    # 2c. 仍在（D 态不可中断 / GL 上下文卡死）→ 容器级兜底重启
    if [ -n "$(_isaac_live_pids)" ]; then
      echo "[teardown] [3/3] SIGKILL 后仍存活（疑 D 态 / GL 卡死）→ docker restart go2w-isaac"
      echo "[teardown]        （容器级重启终结整个 PID namespace，保留容器满足'重建代价高'）"
      # docker restart = docker stop(SIGTERM,grace 后 SIGKILL) + docker start；
      # -t 指定 grace，容器 config/id 保留（非 --rm，AutoRemove=false 已核）。
      docker restart -t "$DOCKER_STOP_GRACE_S" go2w-isaac >/dev/null 2>&1 || true
      sleep "$TD_GRACE_S"
    fi
  fi

  # ── 3) 清理阶段/诊断文件（含谎报 green 的 phase 文件）────────────────────
  rm -f "$REPO"/logs/.bringup.* 2>/dev/null || true

  # ── 4) 成功判据复核：宿主 pgrep 空 且 status.sh l0=false，绝不静默假成功 ──
  local residual l0
  residual="$(_isaac_live_pids)"
  # status.sh l0 = 两容器都在跑；teardown 后 navstack 已移除 → 期望 l0=false。
  l0="$(bash "$HERE/status.sh" 2>/dev/null | sed -n 's/.*"l0":\([a-z]*\).*/\1/p')"
  if [ -z "$residual" ] && [ "$l0" != "true" ]; then
    echo "[teardown] SUCCESS: kit-python 已灭（宿主 pgrep 空）且 l0=false（navstack 已拆）"
    return 0
  fi
  echo "[teardown] FAILED: 拆链未达判据 — residual_pids='${residual:-<空>}' l0='${l0:-?}'" >&2
  _dump_residual
  return 1
}

# ---------------------------------------------------------------------------
# up: 幂等拉起
# ---------------------------------------------------------------------------
up() {
  _validate_nav_profile
  # 0) 幂等短路：健康、DDS 配置一致且关键发布者唯一才允许直接返回。
  _phase "probe (idempotency check)"
  if bash "$HERE/status.sh" >/dev/null 2>&1 \
     && _runtime_dds_matches \
     && _ros_graph_unique \
     && _local_planner_contract; then
    echo "already-up"
    _phase "already-up"
    exit 0
  fi

  # 1) 防双开守卫 -----------------------------------------------------------
  _phase "guard (double-launch / memory)"
  # 1a. `up` 已知当前栈未 green。若仍有 Kit，可能是 editor-quit 后 ROS 已死的
  # 假存活实例；必须先成对拆掉 navstack/RViz 和 Isaac，再继续同一次 up。
  if pgrep -f "kit/pytho[n]" >/dev/null 2>&1; then
    echo "[bringup] 检测到非 green 的 Kit 实例；执行成对 teardown 后自愈重启。" >&2
    teardown
  fi
  # 1b. 可用内存不足 => 拒绝（共享 64G 宿主，双 sim 会 OOM 连累其他会话/桌面）
  local avail_gb
  avail_gb="$(free -g | awk '/^Mem:/{print $7}')"   # 第7列=available（计入可回收 buff/cache）
  if [ -z "$avail_gb" ] || [ "$avail_gb" -lt "$MEM_MIN_GB" ]; then
    echo "[bringup] 拒绝启动：可用内存 ${avail_gb:-?}G < ${MEM_MIN_GB}G 阈值。" >&2
    echo "[bringup] Isaac(40g)+navstack(20g) 需宽裕内存；等其它占用释放后再试（free -g 查）。" >&2
    exit 1
  fi

  # 2) go2w-isaac 容器就位 --------------------------------------------------
  _phase "isaac container start"
  local state
  state="$(docker inspect -f '{{.State.Status}}' go2w-isaac 2>/dev/null || echo "missing")"
  case "$state" in
    running) echo "[bringup] go2w-isaac 已在运行" ;;
    exited|created|paused|dead|stopped)
      echo "[bringup] go2w-isaac 处于 '$state'，docker start"
      docker start go2w-isaac >/dev/null ;;
    missing|*)
      echo "[bringup] 错误：go2w-isaac 容器不存在。请先建容器：bash scripts/setup_container.sh" >&2
      exit 1 ;;
  esac

  # 2b. POLICY 存在性校验（容器内路径，故用 docker exec test -f）
  if ! docker exec go2w-isaac test -f "$POLICY"; then
    echo "[bringup] 错误：RL 策略不存在于容器内: $POLICY" >&2
    echo "[bringup] 用 GO2W_POLICY=<容器内路径> 覆盖，或先训练/放置 ckpt。" >&2
    exit 1
  fi

  # 3) 配对重启（内联 restart_all.sh 主体；铁律：navstack 与 Isaac 桥一起重启）------
  #    顺序: Isaac 桥 -> 8s 仿真稳定标记 -> navstack/RViz -> 门控。
  #    ARISE 只在启动时估计一次重力；navstack 先起会把机器人落地瞬态固化成地图倾角。
  bash "$HERE/sync_navstack_files.sh" "$NAV"   # 真相源 scripts/nav -> refs（防旧拷贝）
  if [ "${NAV_MODE:-waypoint}" = "explore" ] && \
     [ ! -x "$NAV/install/tare_planner/lib/tare_planner/tare_planner_node" ]; then
    echo "[bringup] FAILED: explore 模式需要 tare_planner，但 install/ 里没有（镜像原始"
    echo "  编译不含它）。补编（一次性，~3s，or-tools 为 vendored 预编译库）："
    echo "  docker run --rm --memory 8g -v $NAV:/ws -w /ws navstack:ready bash -c \\"
    echo "    'source /opt/ros/jazzy/setup.bash && colcon build --packages-select tare_planner'"
    exit 1
  fi
  _phase "isaac bridge (paired restart)"
  # 先清 Isaac 里的旧 sim（字符类防自杀），再起新实例
  docker exec -u 0 go2w-isaac bash -c 'pkill -9 -f "kit/pytho[n]" 2>/dev/null; sleep 2' || true
  : > "$LOG"
  rm -f "$REPO/logs/.isaac_heartbeat"
  docker exec -d -u 0 -e DISPLAY="${DISPLAY:-:0}" -e ROS_DISTRO=jazzy \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib -e PYTHONUNBUFFERED=1 \
    -e GO2W_STANDSTILL="${GO2W_STANDSTILL:-1}" -e GO2W_FAST_RENDER="${GO2W_FAST_RENDER:-0}" \
    -e GO2W_POLICY_WHEEL_DAMPING="${GO2W_POLICY_WHEEL_DAMPING:-5.0}" \
    -e GO2W_STANDSTILL_WHEEL_KP="${GO2W_STANDSTILL_WHEEL_KP:-20.0}" \
    -e GO2W_STANDSTILL_WHEEL_DAMPING="${GO2W_STANDSTILL_WHEEL_DAMPING:-8.0}" \
    -e GO2W_STANDSTILL_GAIN_TICKS="${GO2W_STANDSTILL_GAIN_TICKS:-10}" \
    -e GO2W_STANDSTILL_LINEAR_ENTER_MPS="${GO2W_STANDSTILL_LINEAR_ENTER_MPS:-0.005}" \
    -e GO2W_STANDSTILL_LINEAR_EXIT_MPS="${GO2W_STANDSTILL_LINEAR_EXIT_MPS:-0.015}" \
    -e GO2W_STANDSTILL_ANGULAR_ENTER_RPS="${GO2W_STANDSTILL_ANGULAR_ENTER_RPS:-0.005}" \
    -e GO2W_STANDSTILL_ANGULAR_EXIT_RPS="${GO2W_STANDSTILL_ANGULAR_EXIT_RPS:-0.015}" \
    -e GO2W_STANDSTILL_ENTER_DURATION_S="${GO2W_STANDSTILL_ENTER_DURATION_S:-0.50}" \
    -e GO2W_STANDSTILL_EXIT_DURATION_S="${GO2W_STANDSTILL_EXIT_DURATION_S:-0.04}" \
    -e GO2W_VIEWPORT_SLIM="${GO2W_VIEWPORT_SLIM:-0}" -e GO2W_CAM_SLOW="${GO2W_CAM_SLOW:-0}" \
    -e GO2W_DLSS_PERF="${GO2W_DLSS_PERF:-0}" -e GO2W_SCENE="${GO2W_SCENE:-office}" \
    -e GO2W_MANIP_SCENE_CONFIG="${GO2W_MANIP_SCENE_CONFIG:-/workspace/go2w/configs/manip_office_scene.json}" \
    -e GO2W_CAM_TF_PARENT="${GO2W_CAM_TF_PARENT:-base_link}" \
    go2w-isaac bash -c "cd /workspace/go2w/scripts/sim && TERM=xterm \
    /isaac-sim/python.sh warehouse_nav.py --env warehouse --enable_cameras --policy $POLICY \
    --shot_dir /workspace/go2w/logs/shots > /workspace/go2w/logs/nav_bridge.log 2>&1"

  # 4) 等 Isaac 站稳——有界等待（替换 restart_all 的无限 until）------------------
  _phase "wait isaac sensor settle (<=${ISAAC_TIMEOUT_S}s)"
  local deadline=$(( $(date +%s) + ISAAC_TIMEOUT_S ))
  # step=800 是 8 s 仿真时间；此后才允许 ARISE 做一次性重力初始化。
  until grep -qa "imu settled" "$LOG" 2>/dev/null; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
      echo "[bringup] 超时：${ISAAC_TIMEOUT_S}s 内 Isaac 未站稳（nav_bridge.log 无 imu settled）。" >&2
      echo "[bringup] ===== nav_bridge.log 尾 20 行诊断 =====" >&2
      tail -20 "$LOG" 2>/dev/null >&2 || echo "[bringup] （日志文件不存在: $LOG）" >&2
      _phase "FAILED: isaac sensor settle timeout"
      exit 1
    fi
    sleep 2
  done

  # 5) xhost 放行后再起 navstack/RViz。
  _phase "xhost +local:"
  if ! xhost +local: >/dev/null 2>&1; then
    if [ "$REQUIRE_GUI" = "1" ]; then
      echo "[bringup] FAILED: GO2W_REQUIRE_GUI=1 但 DISPLAY='${DISPLAY:-}' 无法通过 xhost 访问" >&2
      _phase "FAILED: X11 authorization"
      exit 1
    fi
    echo "[bringup] warn: xhost +local: 失败（无 X/headless？RViz 可能起不来，不阻塞链路）"
  fi

  _phase "navstack supervisor (paired restart)"
  docker rm -f navstack >/dev/null 2>&1 || true
  docker run -d --name navstack --net=host --ipc=host --init --memory 20g --user 0 \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
    -e NAV_MODE="${NAV_MODE:-waypoint}" \
    -e GO2W_NAV_PROFILE="$GO2W_NAV_PROFILE" \
    -e NAV_MAX_SPEED="$NAV_MAX_SPEED" \
    -e NAV_AUTONOMY_SPEED="$NAV_AUTONOMY_SPEED" \
    -e NAV_MAX_ACCEL="$NAV_MAX_ACCEL" \
    -e NAV_MAX_YAW_RATE_DEG_S="$NAV_MAX_YAW_RATE_DEG_S" \
    -e PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD="$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD" \
    -e NAV_SPEED="$NAV_SPEED" \
    -e NAV_ROBOT_CONFIG="$NAV_ROBOT_CONFIG" \
    -e LOCAL_PLANNER_GOAL_REACHED_THRESHOLD="$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD" \
    -e LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE="$LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$NAV":/ws -w /ws \
    navstack:ready bash /ws/run_all_forever.sh >/dev/null

  # 6) 门控复核：navstack 刚创建时需要数秒完成发现和 SLAM 初始化。
  _phase "gate (wait green <=${NAV_TIMEOUT_S}s)"
  deadline=$(( $(date +%s) + NAV_TIMEOUT_S ))
  until bash "$HERE/status.sh" >/dev/null 2>&1 \
        && _ros_graph_unique \
        && _ros_stream_healthy \
        && _local_planner_contract \
        && _gui_healthy; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
      echo "[bringup] GATE-FAILED: ${NAV_TIMEOUT_S}s 内链路未达 green。" >&2
      bash "$HERE/status.sh" >&2 || true
      echo "[bringup] ===== system.log 尾 30 行诊断 =====" >&2
      tail -30 "$NAV/system.log" 2>/dev/null >&2 || true
      echo "[bringup] ===== ROS graph gate =====" >&2
      cat "$REPO/logs/ros_graph_gate.log" 2>/dev/null >&2 || true
      echo "[bringup] ===== ROS stream gate =====" >&2
      cat "$REPO/logs/ros_stream_gate.log" 2>/dev/null >&2 || true
      _phase "FAILED: gate timeout"
      exit 1
    fi
    sleep 2
  done
  bash "$HERE/status.sh"
  echo "[bringup] DDS domain=$ROS_DOMAIN_ID; critical publishers unique and streams advancing; require_gui=$REQUIRE_GUI"
  echo "[bringup] nav_profile=$GO2W_NAV_PROFILE robot_config=$NAV_ROBOT_CONFIG speed=$NAV_AUTONOMY_SPEED/$NAV_MAX_SPEED m/s command_ramp_100hz=$NAV_MAX_ACCEL yaw_cap=$NAV_MAX_YAW_RATE_DEG_S deg/s stop=$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD m goal=$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD m"
  _phase "up (green)"
  echo "[bringup] ALL-GREEN: 全链就绪"
  exit 0
}

# ---------------------------------------------------------------------------
case "${1:-up}" in
  up)       up ;;
  # teardown 的退出码是合同的一部分（0=拆净, 非0=残留）——显式捕获并 exit 传播给
  # 调用方（zeno Go2WBringupTool 据此设 is_error）。临时关 set -e 包住调用，否则
  # teardown 内部的 `return 1` 会让 set -e 在 exit $? 前就以未定值中断。
  teardown) set +e; teardown; rc=$?; set -e; exit "$rc" ;;
  *) echo "用法: $0 [up|teardown]" >&2; exit 2 ;;
esac
