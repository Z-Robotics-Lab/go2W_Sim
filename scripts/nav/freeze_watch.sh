#!/usr/bin/env bash
# 冻结哨兵：守着 Isaac 栈，自动取证"自发冻结"（12-19min 系统性僵死嫌疑）。
#
# 背景（2026-07-06，DEBUG.md）：僵尸 #1/#2 均在拉起后 ~11-14min 全话题一瞬同冻
# （kit STAT=Rl、CPU 5xx%、无 traceback、GUI 僵）。人守 19 分钟不现实——本哨兵
# 每 10s 轮询桥 /health 的 age_s，>15s 判冻结，自动抓双栈（py-spy Python 栈 +
# gdb native 栈）、GPU/内核证据，落 var/evidence/freeze_watch/<时间戳>/，然后
# **停住不杀**（留标本给人验尸）。
#
# 用法:
#   nohup bash scripts/nav/freeze_watch.sh >/dev/null 2>&1 &   # 部署（PID 自动落盘）
#   tail -f var/evidence/freeze_watch/watch.log                # 观察
#   kill "$(cat var/evidence/freeze_watch/watch.pid)"          # 撤哨（只杀哨兵自己）
#
# 铁律：只读取证，绝不杀 sim/容器（NEVER-KILL-INFRA）；证据不进 /tmp；
# 栈 down/桥不可达时安静等待（跨越 teardown/bringup 周期存活）。

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
EVID_ROOT="$REPO/var/evidence/freeze_watch"
WATCH_LOG="$EVID_ROOT/watch.log"
PID_FILE="$EVID_ROOT/watch.pid"
PYSPY="$EVID_ROOT/py-spy"                 # 从 go2w-isaac 容器拷出的 manylinux 二进制
HEALTH_URL="${GO2W_BRIDGE:-http://127.0.0.1:8042}/health"
POLL_S="${FW_POLL_S:-10}"                 # 轮询间隔
FREEZE_AGE_S="${FW_FREEZE_AGE_S:-15}"     # age_s 超此值判冻结
KILL_PATTERN='kit/pytho[n]'               # 目标 sim 进程（字符类防自匹配）

mkdir -p "$EVID_ROOT"

_log() { echo "[$(date -Is)] $*" >> "$WATCH_LOG"; }

# ── 单实例守卫 ───────────────────────────────────────────────────────────────
if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE" 2>/dev/null)" 2>/dev/null; then
  echo "freeze_watch 已在运行 (PID $(cat "$PID_FILE"))，拒绝双开" >&2
  exit 1
fi
echo $$ > "$PID_FILE"
_log "SENTINEL START pid=$$ poll=${POLL_S}s freeze_age>${FREEZE_AGE_S}s url=$HEALTH_URL"

# ── 启动自检：取证工具就位否（缺了先报，别等冻结时抓瞎）─────────────────────
sudo -n true 2>/dev/null       || _log "WARN: sudo -n 不可用——py-spy/gdb/dmesg 取证将失败"
[ -x "$PYSPY" ]                || _log "WARN: $PYSPY 缺失——Python 栈取证降级为 gdb native"
command -v gdb >/dev/null 2>&1 || _log "WARN: 宿主无 gdb——native 栈取证不可用"

# ── /health 的最大 age_s（pose/gt/grasp 取最大；解析失败返回空）──────────────
_max_age() {
  # 输入: /health JSON。抠所有 "age_s": <num> 取最大（无 jq 依赖）。
  grep -oE '"age_s"[[:space:]]*:[[:space:]]*[0-9.]+' \
    | grep -oE '[0-9.]+' | sort -g | tail -1
}

# ── 冻结取证（核心）──────────────────────────────────────────────────────────
_capture() {
  local age="$1" ts dir pid
  ts="$(date +%Y%m%dT%H%M%S)"
  dir="$EVID_ROOT/$ts"
  mkdir -p "$dir"
  _log "FREEZE DETECTED age=${age}s -> 取证目录 $dir"

  {
    echo "freeze_detected_at: $(date -Is)"
    echo "max_age_s: $age"
    echo "health: $(curl -s --max-time 3 "$HEALTH_URL" 2>&1)"
    echo "status_sh: $(bash "$REPO/scripts/nav/status.sh" 2>&1)"
    echo "phase_file: $(cat "$REPO/logs/.bringup.phase" 2>/dev/null)"
  } > "$dir/meta.txt"

  pid="$(pgrep -f "$KILL_PATTERN" 2>/dev/null | head -1)"
  echo "kit_host_pid: ${pid:-NOT-FOUND}" >> "$dir/meta.txt"
  if [ -z "$pid" ]; then
    _log "kit 进程不存在（栈可能被拆而非冻结）——只留 meta，不抓栈"
    return
  fi

  # 1) 线程级 CPU 两拍快照（3s 间隔）——527% 在哪些 TID 上烧
  ps -Lp "$pid" -o tid,pcpu,state,wchan:22,comm > "$dir/threads_t0.txt" 2>&1
  sleep 3
  ps -Lp "$pid" -o tid,pcpu,state,wchan:22,comm > "$dir/threads_t1.txt" 2>&1

  # 2) py-spy Python 栈 ×2（5s 间隔，对比看栈是否在动）
  if [ -x "$PYSPY" ]; then
    sudo -n "$PYSPY" dump --pid "$pid"              > "$dir/pyspy_1.txt" 2>&1
    sleep 5
    sudo -n "$PYSPY" dump --pid "$pid"              > "$dir/pyspy_2.txt" 2>&1
    # 跨 mount-ns 找不到解释器时降级 --nonblocking 再试一发
    grep -q "Error" "$dir/pyspy_1.txt" 2>/dev/null && \
      sudo -n "$PYSPY" dump --nonblocking --pid "$pid" > "$dir/pyspy_nonblocking.txt" 2>&1
  fi

  # 3) gdb native 全线程栈（root 可跨 ns attach；232 线程约 30-90s，capped 180s）
  if command -v gdb >/dev/null 2>&1; then
    timeout 180 sudo -n gdb -p "$pid" -batch \
      -ex "set pagination off" -ex "set confirm off" \
      -ex "thread apply all bt" > "$dir/gdb_all_threads.txt" 2>&1
  fi

  # 4) GPU 证据：利用率（0%=CPU 侧活锁，高=GPU 侧卡）+ 持存
  nvidia-smi                                   > "$dir/nvidia_smi.txt" 2>&1
  nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv \
                                               >> "$dir/nvidia_smi.txt" 2>&1
  nvidia-smi dmon -c 3                         > "$dir/nvidia_dmon.txt" 2>&1

  # 5) 内核侧：dmesg 尾部 + Xid（GPU 挂死铁证）
  sudo -n dmesg 2>/dev/null | tail -200        > "$dir/dmesg_tail.txt"
  grep -iE "xid|nvrm" "$dir/dmesg_tail.txt"    > "$dir/dmesg_xid.txt" 2>&1

  # 6) 日志面：kit 会话日志尾 + nav_bridge 尾 + docker logs 尾 + 截图节奏
  docker exec go2w-isaac bash -c \
    'f=$(ls -t /root/.nvidia-omniverse/logs/Kit/*/*.log 2>/dev/null | head -1); \
     [ -n "$f" ] && { echo "== $f =="; tail -300 "$f"; } || echo "no kit log"' \
                                               > "$dir/kit_log_tail.txt" 2>&1
  tail -150 "$REPO/logs/nav_bridge.log"        > "$dir/nav_bridge_tail.txt" 2>&1
  docker logs --tail 50 go2w-isaac             > "$dir/docker_logs_isaac.txt" 2>&1
  ls -la --time-style='+%m-%d %H:%M:%S' "$REPO/logs/shots/" 2>/dev/null | head -20 \
                                               > "$dir/shots_cadence.txt"

  _log "取证完成 -> $dir （标本未杀，停止轮询；人工验尸后自行 teardown）"
  echo "$dir" > "$EVID_ROOT/LAST_FREEZE"
}

# ── 主循环：安静等待，冻结即取证然后停住 ─────────────────────────────────────
consecutive_ok=0
while true; do
  health="$(curl -s --max-time 3 "$HEALTH_URL" 2>/dev/null)"
  if [ -z "$health" ]; then
    # 桥不可达：栈 down / bringup 中——安静等待（每 6 拍记一行心跳）
    consecutive_ok=0
    [ $(( $(date +%s) % 60 )) -lt "$POLL_S" ] && _log "idle: 桥不可达（栈 down/启动中）"
  else
    age="$(printf '%s' "$health" | _max_age)"
    if [ -n "$age" ] && awk -v a="$age" -v m="$FREEZE_AGE_S" 'BEGIN{ exit !(a > m) }'; then
      _capture "$age"
      _log "SENTINEL HALT（留标本）。重新部署前先处理 $EVID_ROOT/LAST_FREEZE"
      rm -f "$PID_FILE"
      exit 0
    fi
    consecutive_ok=$((consecutive_ok + 1))
    # 每 ~5min 记一行活着的心跳（避免刷屏）
    [ $(( consecutive_ok % 30 )) -eq 1 ] && _log "ok: age=${age:-?}s"
  fi
  sleep "$POLL_S"
done
