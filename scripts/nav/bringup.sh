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
# RL locomotion 策略（容器内路径；README 坑 26：差速在 Go2W 物理不可行，必须 RL 策略）
POLICY="${GO2W_POLICY:-/workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-04_15-52-42/model_1999.pt}"

_phase() {  # 记录当前阶段（覆盖写，供外部/事后诊断读）
  mkdir -p "$REPO/logs"
  echo "$1  $(date -Is)" > "$PHASE_FILE"
  echo "[bringup] $1"
}

# ---------------------------------------------------------------------------
# teardown: scoped 拆链，绝不 rosm nuke、绝不无差别 pkill（NEVER-KILL-INFRA）
# ---------------------------------------------------------------------------
teardown() {
  echo "[teardown] 拆 navstack + Isaac 桥进程（scoped）"
  # navstack 整个容器移除（PID-1 supervisor 随容器消亡，干净）
  docker rm -f navstack >/dev/null 2>&1 || true
  # go2w-isaac 容器保留（重建代价高），只杀其中的 kit-python sim 进程。
  # 字符类 kit/pytho[n] 防脚本自杀（同 restart_all.sh 铁律），且只 exec 进目标容器，
  # 绝不在宿主上跑 pkill（宿主 pkill 会连坐 sibling 会话）。
  docker exec -u 0 go2w-isaac bash -c 'pkill -9 -f "kit/pytho[n]" 2>/dev/null; true' || true
  # 清理本脚本落的阶段/诊断文件
  rm -f "$REPO"/logs/.bringup.* 2>/dev/null || true
  echo "[teardown] 复核 L0（期望 l0=false）:"
  bash "$HERE/status.sh" || true   # green=false 时 status.sh 退出非零，属正常，别让 set -e 挂住
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
  #    顺序: navstack(PID-1 supervisor) -> Isaac 桥 -> 门控。
  _phase "navstack supervisor (paired restart)"
  docker rm -f navstack >/dev/null 2>&1 || true
  docker run -d --name navstack --net=host --ipc=host --init --memory 20g --user 0 \
    -e ROS_DOMAIN_ID=42 -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$NAV":/ws -w /ws \
    navstack:ready bash /ws/run_all_forever.sh >/dev/null

  _phase "isaac bridge (paired restart)"
  # 先清 Isaac 里的旧 sim（字符类防自杀），再起新实例
  docker exec -u 0 go2w-isaac bash -c 'pkill -9 -f "kit/pytho[n]" 2>/dev/null; sleep 2' || true
  docker exec -d -u 0 -e DISPLAY="${DISPLAY:-:0}" -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib -e PYTHONUNBUFFERED=1 \
    go2w-isaac bash -c "cd /workspace/go2w/scripts/sim && TERM=xterm \
    /isaac-sim/python.sh warehouse_nav.py --env warehouse --enable_cameras --policy $POLICY \
    --shot_dir /workspace/go2w/logs/shots > /workspace/go2w/logs/nav_bridge.log 2>&1"

  # 4) 等 Isaac 就绪——有界等待（替换 restart_all 的无限 until）------------------
  _phase "wait isaac ready (<=${ISAAC_TIMEOUT_S}s)"
  local deadline=$(( $(date +%s) + ISAAC_TIMEOUT_S ))
  # 就绪判据同 status.sh L2：出现 imu sample（sim 在真发传感器数据）
  until grep -qa "imu sample" "$LOG" 2>/dev/null; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
      echo "[bringup] 超时：${ISAAC_TIMEOUT_S}s 内 Isaac 未就绪（nav_bridge.log 无 imu sample）。" >&2
      echo "[bringup] ===== nav_bridge.log 尾 20 行诊断 =====" >&2
      tail -20 "$LOG" 2>/dev/null >&2 || echo "[bringup] （日志文件不存在: $LOG）" >&2
      _phase "FAILED: isaac ready timeout"
      exit 1
    fi
    sleep 15
  done

  # 5) xhost 放行（起 RViz 前；失败只 warn，headless 无 X 时不阻塞）
  _phase "xhost +local:"
  xhost +local: >/dev/null 2>&1 || echo "[bringup] warn: xhost +local: 失败（无 X/headless？RViz 可能起不来，不阻塞链路）"

  # 6) 门控复核：用 status.sh 判 green -----------------------------------------
  _phase "gate (status.sh green check)"
  if bash "$HERE/status.sh"; then
    _phase "up (green)"
    echo "[bringup] ALL-GREEN: 全链就绪"
    exit 0
  else
    echo "[bringup] GATE-FAILED: Isaac 已出数据但链路未达 green（SLAM 未收敛？查 status.sh 各层）。" >&2
    _phase "FAILED: gate not green"
    exit 1
  fi
}

# ---------------------------------------------------------------------------
case "${1:-up}" in
  up)       up ;;
  teardown) teardown ;;
  *) echo "用法: $0 [up|teardown]" >&2; exit 2 ;;
esac
