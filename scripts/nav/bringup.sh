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
NAV_TIMEOUT_S="${GO2W_NAV_TIMEOUT_S:-60}"        # navstack/RViz 启动及 SLAM 收敛上限
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
  # 0) 幂等短路：已 green 直接返回
  _phase "probe (idempotency check)"
  if bash "$HERE/status.sh" >/dev/null 2>&1; then
    echo "already-up"
    _phase "already-up"
    exit 0
  fi

  # 1) 防双开守卫 -----------------------------------------------------------
  _phase "guard (double-launch / memory)"
  # 1a. 已有 kit-python（Isaac sim）在跑 => 疑似另一实例，拒绝（README 坑 15：一次一个 sim）
  if pgrep -f "kit/pytho[n]" >/dev/null 2>&1; then
    echo "[bringup] 拒绝启动：检测到 kit-python（Isaac sim）已在运行——一次只跑一个 sim 实例。" >&2
    echo "[bringup] 若确为僵尸残留，请先 bash scripts/nav/bringup.sh teardown。" >&2
    exit 1
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
  docker exec -d -u 0 -e DISPLAY="${DISPLAY:-:0}" -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib -e PYTHONUNBUFFERED=1 \
    -e GO2W_STANDSTILL="${GO2W_STANDSTILL:-1}" -e GO2W_FAST_RENDER="${GO2W_FAST_RENDER:-0}" \
    -e GO2W_VIEWPORT_SLIM="${GO2W_VIEWPORT_SLIM:-0}" -e GO2W_CAM_SLOW="${GO2W_CAM_SLOW:-0}" \
    -e GO2W_DLSS_PERF="${GO2W_DLSS_PERF:-0}" -e GO2W_SCENE="${GO2W_SCENE:-warehouse}" \
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
  xhost +local: >/dev/null 2>&1 || echo "[bringup] warn: xhost +local: 失败（无 X/headless？RViz 可能起不来，不阻塞链路）"

  _phase "navstack supervisor (paired restart)"
  docker rm -f navstack >/dev/null 2>&1 || true
  docker run -d --name navstack --net=host --ipc=host --init --memory 20g --user 0 \
    -e ROS_DOMAIN_ID=42 -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
    -e NAV_MODE="${NAV_MODE:-waypoint}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$NAV":/ws -w /ws \
    navstack:ready bash /ws/run_all_forever.sh >/dev/null

  # 6) 门控复核：navstack 刚创建时需要数秒完成发现和 SLAM 初始化。
  _phase "gate (wait green <=${NAV_TIMEOUT_S}s)"
  deadline=$(( $(date +%s) + NAV_TIMEOUT_S ))
  until bash "$HERE/status.sh" >/dev/null 2>&1; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
      echo "[bringup] GATE-FAILED: ${NAV_TIMEOUT_S}s 内链路未达 green。" >&2
      bash "$HERE/status.sh" >&2 || true
      echo "[bringup] ===== system.log 尾 30 行诊断 =====" >&2
      tail -30 "$NAV/system.log" 2>/dev/null >&2 || true
      _phase "FAILED: gate timeout"
      exit 1
    fi
    sleep 2
  done
  bash "$HERE/status.sh"
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
