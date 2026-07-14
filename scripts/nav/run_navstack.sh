#!/usr/bin/env bash
# Compatibility entry point for older `docker exec ... /ws/run_navstack.sh`
# workflows. The PID-1 supervisor is the single owner of profile validation,
# launch arguments, bridge processes, and restart behavior.
set -e

if pgrep -f '[/]ws/run_all_forever.sh' >/dev/null 2>&1; then
  if pgrep -f '[r]os2 launch /ws/system_isaac_sim' >/dev/null 2>&1 \
     && pgrep -x localPlanner >/dev/null 2>&1 \
     && pgrep -x pathFollower >/dev/null 2>&1; then
    echo "[ORCH] run_all_forever owns a running navigation stack"
    exit 0
  fi
  echo "[ORCH] supervisor exists but navigation children are not healthy" >&2
  echo "[ORCH] inspect the paired bringup status before retrying" >&2
  exit 1
fi

# Clean only processes created by the legacy orchestrator before handing
# ownership to the shared supervisor. This branch is unreachable in normal
# bringup.sh containers, where run_all_forever is already PID 1.
pkill -f "pc2_to_livox.py" 2>/dev/null || true
pkill -f "system_isaac_sim.launch" 2>/dev/null || true
pkill -f "feature_extraction_node|laser_mapping_node|imu_preintegration_node" \
  2>/dev/null || true
pkill -f "localPlanner|pathFollower|terrainAnalysis|sensorScanGeneration|visualizationTools" \
  2>/dev/null || true
sleep 3

exec bash /ws/run_all_forever.sh
