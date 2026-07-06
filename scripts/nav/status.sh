#!/usr/bin/env bash
# 分层健康探针（纯只读，可重复调用；不启动/不重启/不杀任何东西）。
# 输出单行 JSON: {"l0":bool,...,"phase":"最高已达层","green":bool}；退出码 0=green(=L4)。
#
# 层级阶梯（越高越接近可用）：
#   L0 容器  : go2w-isaac 与 navstack 两容器都在跑
#   L1 Isaac 起 : nav_bridge.log 出现 "joints(46) ready"（关节加载完）
#   L2 Isaac 出数据 : nav_bridge.log 出现 "imu sample"（传感器在发）
#   L3 桥+GT : 127.0.0.1:8042/gt 返回 200（HTTP 桥活 + 地面真值可读）
#   L4 SLAM 收敛 : 127.0.0.1:8042/pose 返回 200（SLAM 有位姿估计）
#   L5 RViz  : navstack 里 rviz2 在跑（软状态，不参与 green 判定）
#
# 铁律注解：green 只认 L4——SLAM 收敛才算"链路可用"；RViz 挂了不算链路故障。
# 日志路径相对仓库根（容器 /workspace/go2w == 宿主仓库根，见 setup_container.sh -v 挂载）。

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
LOG="$REPO/logs/nav_bridge.log"

# --- L0: 两容器都在跑 ---------------------------------------------------------
# docker inspect 对不存在的容器会报错退出，2>/dev/null 吞掉；只有输出恰好 "true" 才算起。
_running() { [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" = "true" ]; }
if _running go2w-isaac && _running navstack; then l0=true; else l0=false; fi

# --- L1: Isaac 关节就绪 -------------------------------------------------------
# grep -qa: -q 只看有无匹配（静默），-a 把日志当文本（防二进制字符打断匹配）。
# 日志文件不存在时 grep 返回非零 -> false，不报错。
# 注意：标记是 "[NAV] joints(N) ready"，N=运行时 articulation 关节数，随 nav 配置变
# （实测 warehouse_nav 跑出 joints(24)，非 README 的整机 46）。故按结构匹配 joints(<数字>)
# ready，不写死数量——否则关节数一变 L1 永远假阴性（活基线实证：日志是 joints(24)）。
if [ "$l0" = true ] && grep -qaE 'joints\([0-9]+\) ready' "$LOG" 2>/dev/null; then l1=true; else l1=false; fi

# --- L2: Isaac 在出传感器数据 -------------------------------------------------
if [ "$l1" = true ] && grep -qa "imu sample" "$LOG" 2>/dev/null; then l2=true; else l2=false; fi

# --- L3: HTTP 桥活 + 地面真值可读 --------------------------------------------
# curl -sf: -s 静默，-f 非 2xx 返回非零退出码（--max-time 3 防桥卡死时探针挂住）。
if [ "$l2" = true ] && curl -sf --max-time 3 127.0.0.1:8042/gt >/dev/null 2>&1; then l3=true; else l3=false; fi

# --- L4: SLAM 有位姿（收敛）--------------------------------------------------
if [ "$l3" = true ] && curl -sf --max-time 3 127.0.0.1:8042/pose >/dev/null 2>&1; then l4=true; else l4=false; fi

# --- L5: RViz（软状态，pgrep -x 精确匹配进程名，避免误匹配）------------------
# 仅在 L0 起时才 docker exec 探（容器没起时 exec 会报错刷噪音）。
if [ "$l0" = true ] && docker exec navstack pgrep -x rviz2 >/dev/null 2>&1; then l5=true; else l5=false; fi

# --- phase: 最高"连续"已达层（遇到第一个 false 即停）--------------------------
phase="none"
for lvl in l0 l1 l2 l3 l4 l5; do
  if [ "${!lvl}" = true ]; then phase="$lvl"; else break; fi
done

green="$l4"   # green == L4（SLAM 收敛）

printf '{"l0":%s,"l1":%s,"l2":%s,"l3":%s,"l4":%s,"l5":%s,"phase":"%s","green":%s}\n' \
  "$l0" "$l1" "$l2" "$l3" "$l4" "$l5" "$phase" "$green"

[ "$green" = true ] && exit 0 || exit 1
