#!/usr/bin/env bash
# record_rollout.sh <duration_s> <label> — 被动录制 Isaac 窗口 + 同步位姿采样
# 产出: var/evidence/rollouts/<label>/{rollout.mp4, pose.csv, frames/f_XXX.jpg}
# 零侵入：只录屏 + GET 桥，绝不发指令；可与任何在跑实验共存。
set -e
DUR="${1:-60}"
LABEL="${2:-seg_$(date +%H%M%S)}"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$REPO/var/evidence/rollouts/$LABEL"
BRIDGE="${GO2W_BRIDGE:-http://127.0.0.1:8042}"
mkdir -p "$OUT/frames"

# Isaac 主窗口 = 匹配名中面积最大者（Isaac 有多个隐藏子窗口）
WIN=""; AREA=0
for id in $(xdotool search --name "Isaac Sim" 2>/dev/null); do
  eval "$(xdotool getwindowgeometry --shell "$id" 2>/dev/null || true)"
  a=$((WIDTH * HEIGHT))
  if [ "$a" -gt "$AREA" ]; then AREA=$a; WIN=$id; fi
done
[ -n "$WIN" ] || { echo "未找到 Isaac 窗口" >&2; exit 1; }
xdotool windowactivate "$WIN"; sleep 1
eval "$(xdotool getwindowgeometry --shell "$WIN")"
# x11grab 要求偶数尺寸
WIDTH=$((WIDTH / 2 * 2)); HEIGHT=$((HEIGHT / 2 * 2))
echo "[record] win=$WIN ${WIDTH}x${HEIGHT}+${X}+${Y} dur=${DUR}s -> $OUT"

# 位姿采样（5Hz, wall_t,pose_xyz_yaw_stamp,gt_xy）后台跑
(
  echo "wall_t,px,py,pz,yaw,stamp,gx,gy" > "$OUT/pose.csv"
  end=$(( $(date +%s) + DUR ))
  while [ "$(date +%s)" -lt "$end" ]; do
    p=$(curl -s -m 1 "$BRIDGE/pose" 2>/dev/null || echo '{}')
    g=$(curl -s -m 1 "$BRIDGE/gt" 2>/dev/null || echo '{}')
    python3 - "$p" "$g" >> "$OUT/pose.csv" 2>/dev/null <<'EOF' || true
import json, sys, time
p = json.loads(sys.argv[1] or "{}"); g = json.loads(sys.argv[2] or "{}")
print(f"{time.time():.2f},{p.get('x','')},{p.get('y','')},{p.get('z','')},"
      f"{p.get('yaw','')},{p.get('stamp','')},{g.get('x','')},{g.get('y','')}")
EOF
    sleep 0.2
  done
) &
SAMPLER=$!

ffmpeg -loglevel error -f x11grab -framerate 15 -video_size "${WIDTH}x${HEIGHT}" \
  -i "${DISPLAY:-:0}+${X},${Y}" -t "$DUR" -pix_fmt yuv420p "$OUT/rollout.mp4"
wait "$SAMPLER" 2>/dev/null || true

# 1fps 帧条，供逐帧目检
ffmpeg -loglevel error -i "$OUT/rollout.mp4" -vf fps=1 "$OUT/frames/f_%03d.jpg"
echo "[record] done: $(ls "$OUT/frames" | wc -l) frames, csv $(wc -l < "$OUT/pose.csv") rows"
