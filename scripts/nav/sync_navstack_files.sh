#!/usr/bin/env bash
# 把 scripts/nav/ 的运行时文件同步进 refs 克隆（容器挂 /ws 跑的是 refs 里的拷贝）。
# 真相源永远是 scripts/nav/——本脚本被 bringup.sh / restart_all.sh 在起容器前调用，
# 保证仓库更新后容器不会跑旧拷贝（2026-07-06 集成实证：桥/supervisor 旧拷贝导致
# /health 404 + RViz 不起）。幂等，可重复调用。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
NAV="${1:-$HERE/../../refs/Navigation-Physical-Experiment}"
NAV="$(cd "$NAV" && pwd)"
for f in pc2_to_livox.py run_navstack.sh agent_bridge.py diagnostic_level.py \
         manip_rviz_bridge.py \
         ros_stream_gate.py run_all_forever.sh; do
  cp "$HERE/$f" "$NAV/$f"
done
# One tracked mux contract is shared by simulation and real-robot launch files.
# The cmd_vel_mux install is symlinked to this source path in the nav workspace.
MUX_CONTRACT="$HERE/../../configs/nav/cmd_vel_mux_params.yaml"
MUX_RUNTIME="$NAV/src/utilities/cmd_vel_mux/config/cmd_vel_mux_params.yaml"
install -D -m 0644 "$MUX_CONTRACT" "$MUX_RUNTIME"
# ARISE's calibration install rule creates a regular file rather than a symlink.
# Validate the sim-only zero relative rotation, then refresh the exact file loaded
# by ros2 launch. This prevents source/install drift from silently restoring +20 deg.
python3 "$HERE/sync_runtime_config.py" "$NAV"
# Isaac sim launch：.reference 快照即真相（与 patch 产物逐字节一致已验证）
cp "$HERE/system_isaac_sim.launch.py.reference" \
   "$NAV/system_isaac_sim.launch.py"
cp "$HERE/system_isaac_sim_with_exploration.launch.py.reference" \
   "$NAV/system_isaac_sim_with_exploration.launch.py"
# Combined RViz: keep every upstream navigation display, remove the unsafe
# TeleopPanel structurally, and add measured manipulation/perception displays.
# The copied JSON is also the runtime status bridge's topic contract.
RVIZ_CONTRACT="$HERE/../../configs/rviz/manipulation_topics.json"
cp "$RVIZ_CONTRACT" "$NAV/manipulation_rviz_topics.json"
python3 "$HERE/build_rviz_config.py" \
  "$NAV/src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz" \
  "$RVIZ_CONTRACT" "$NAV/go2w.rviz"
echo "[sync] scripts/nav -> $NAV OK"
