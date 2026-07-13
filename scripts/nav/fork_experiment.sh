#!/usr/bin/env bash
# 5m 长航点叉子实验 runner(低 RTF 时域适配轮预注册门的度量工具)。
# POST /waypoint 远点 -> 容器内 ab_cmdgt_sampler 采 30s /cmd_vel+GT -> 宿主侧算门。
# 只读+发一个航点;不重启/不杀任何东西。证据入 var/evidence/lowrtf_round/。
#
# 用法: bash fork_experiment.sh <label> [wx] [wy] [dur_s]
set -euo pipefail
LABEL="${1:?need label}"
WX="${2:-2.0}"; WY="${3:-0.0}"; DUR="${4:-30}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="$REPO/var/evidence/lowrtf_round"
mkdir -p "$OUT"
CSV="$OUT/fork_${LABEL}.csv"

echo "[fork] label=$LABEL wp=($WX,$WY) dur=${DUR}s"
# 前核 GT(起点)
GT0=$(timeout 15 curl -s "http://127.0.0.1:8042/gt" || echo '{}')
echo "[fork] GT_before=$GT0"
# 发远点航点(经桥,带矫正 joy)
POST=$(timeout 15 curl -s -X POST "http://127.0.0.1:8042/waypoint" \
  -H 'Content-Type: application/json' -d "{\"x\":$WX,\"y\":$WY}" || echo '{}')
echo "[fork] POST /waypoint -> $POST"
# 容器内采样(sampler 写到容器 /ws/... = 宿主 refs/...,但我们让它写宿主挂载可见路径)
# ab_cmdgt_sampler.py 在 scripts/nav(=host);容器里路径经 -v 挂载不同,故拷进容器 /tmp 跑。
docker cp "$HERE/ab_cmdgt_sampler.py" navstack:/tmp/ab_cmdgt_sampler.py
docker exec navstack bash -lc "source /ws/install/setup.bash 2>/dev/null; \
  export ROS_DOMAIN_ID=${GO2W_ROS_DOMAIN_ID:-184}; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; \
  timeout $((DUR+20)) python3 /tmp/ab_cmdgt_sampler.py /tmp/fork_${LABEL}.csv $DUR" 2>&1 | tail -3
docker cp "navstack:/tmp/fork_${LABEL}.csv" "$CSV"
# 后核 GT(终点)
GT1=$(timeout 15 curl -s "http://127.0.0.1:8042/gt" || echo '{}')
echo "[fork] GT_after=$GT1"
echo "[fork] csv=$CSV rows=$(wc -l < "$CSV")"
