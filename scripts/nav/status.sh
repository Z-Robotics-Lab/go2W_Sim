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

# --- L4: SLAM 有位姿（收敛）+ 位姿新鲜（防"green 假象"）----------------------
# 双闸：/pose 返回 200（桥自带陈旧守卫 >5s 会 503，坑 31）**且** /health 自报的
# pose age_s < 阈值。为什么要第二道独立闸：僵尸桥/桥守卫回归时 /pose 可能返回
# 陈旧 200，仅凭 HTTP 码 green 会被冻结的 sim 骗过（2026-07-06 僵尸现场：仿真循环
# 冻结 ~19min，桥仍 ok:true，phase 文件谎报 up(green)）。故这里不信任桥的 200，
# 独立读 /health age_s 判新鲜——green 只在"位姿存在且是刚更新的"时为真。
POSE_MAX_AGE_S="${GO2W_POSE_MAX_AGE_S:-5}"   # 位姿最大允许陈旧秒数（与桥守卫同量级）
_pose_fresh() {
  # 读 /health 的 pose.age_s；无 python/jq 依赖，用 grep -o 抠数值（浮点）。
  # 桥不可达 / 字段缺失 / age 超阈 → 一律判不新鲜（fail-closed，宁缺毋假）。
  local health age
  health="$(curl -sf --max-time 3 127.0.0.1:8042/health 2>/dev/null)" || return 1
  # 抠 "pose": {"present": .., "age_s": <数字>} 里的 age_s
  age="$(printf '%s' "$health" | grep -oE '"pose"[^}]*"age_s"[[:space:]]*:[[:space:]]*[0-9.]+' \
         | grep -oE '[0-9.]+$' | head -1)"
  [ -n "$age" ] || return 1
  # 浮点比较用 awk（bash 只能整数比较）；age < 阈值 → 新鲜(0)，否则陈旧(1)。
  awk -v a="$age" -v m="$POSE_MAX_AGE_S" 'BEGIN{ exit !(a < m) }'
}
if [ "$l3" = true ] && curl -sf --max-time 3 127.0.0.1:8042/pose >/dev/null 2>&1 \
   && _pose_fresh; then l4=true; else l4=false; fi

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
