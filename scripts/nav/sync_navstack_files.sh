#!/usr/bin/env bash
# 把 scripts/nav/ 的运行时文件同步进 refs 克隆（容器挂 /ws 跑的是 refs 里的拷贝）。
# 真相源永远是 scripts/nav/——本脚本被 bringup.sh / restart_all.sh 在起容器前调用，
# 保证仓库更新后容器不会跑旧拷贝（2026-07-06 集成实证：桥/supervisor 旧拷贝导致
# /health 404 + RViz 不起）。幂等，可重复调用。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
NAV="${1:-$HERE/../../refs/Navigation-Physical-Experiment}"
NAV="$(cd "$NAV" && pwd)"
for f in pc2_to_livox.py run_navstack.sh agent_bridge.py run_all_forever.sh; do
  cp "$HERE/$f" "$NAV/$f"
done
# SLAM IMU<->Laser 外参（单一真源，CEO A线指令 2026-07-07）：此前仅在 gitignore 的 refs/ 里，
# 仓库更新带不上 -> 纳入 sync。目标=容器 /ws 里 arise_slam 生效 config 路径。
SLAM_CALIB_DST="$NAV/src/slam/arise_slam_mid360/config/livox/livox_mid360_calibration.yaml"
if [ -d "$(dirname "$SLAM_CALIB_DST")" ]; then
  cp "$HERE/livox_mid360_calibration.yaml" "$SLAM_CALIB_DST"
  # 主配置（含 sensor_mount_pitch_deg=20：odom/TF 车体系化，A线第二层修复 2026-07-08）
  cp "$HERE/livox_mid360.yaml" "$NAV/src/slam/arise_slam_mid360/config/livox_mid360.yaml"
  echo "[sync] SLAM 外参+主配置 -> $NAV/src/slam/arise_slam_mid360/config/"
else
  echo "[sync] WARN: SLAM config 目录不存在（refs 未克隆？）跳过外参同步" >&2
fi
# P5.2 探索变体 launch：.reference 快照即真相（与 patch 第5步产物逐字节一致已验证）
cp "$HERE/system_isaac_sim_with_exploration.launch.py.reference" \
   "$NAV/system_isaac_sim_with_exploration.launch.py"
# RViz 面板去毒（坑29/32）：从 stock 配置生成去掉 TeleopPanel 的 go2w.rviz——
# 面板发的 /joy 会关自主模式并锁死速度；操作者面板不属于 agent 产品面。
python3 - "$NAV" <<'PYEOF'
import sys, re
nav = sys.argv[1]
src = f"{nav}/src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz"
txt = open(src).read()
# Panels 段里移除 teleop 面板条目（缩进块）；显示区不动
txt = re.sub(r"  - Class: teleop_rviz_plugin.*?(?=  - Class: |Visualization Manager:)",
             "", txt, flags=re.S)
open(f"{nav}/go2w.rviz", "w").write(txt)
print("[sync] go2w.rviz (TeleopPanel removed)")
PYEOF
echo "[sync] scripts/nav -> $NAV OK"
