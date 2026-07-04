#!/usr/bin/env bash
# navstack 容器内的一键编排：清理旧进程 -> 转换节点 -> SLAM+base_autonomy 系统。
# 用法（宿主机）: docker exec -d navstack bash /ws/run_navstack.sh
# 日志: /ws/converter.log  /ws/system.log  /ws/orchestrator.log
# set -u  # ROS setup.bash 有 unbound 变量，不能开
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

echo "[ORCH] $(date) cleanup" > /ws/orchestrator.log
pkill -f "pc2_to_livox.py" 2>/dev/null
pkill -f "system_isaac_sim.launch" 2>/dev/null
pkill -f "feature_extraction_node|laser_mapping_node|imu_preintegration_node" 2>/dev/null
pkill -f "localPlanner|pathFollower|terrainAnalysis|sensorScanGeneration|visualizationTools" 2>/dev/null
sleep 3

echo "[ORCH] start converter" >> /ws/orchestrator.log
nohup bash -c 'while true; do
  python3 /ws/pc2_to_livox.py >> /ws/converter.log 2>&1
  echo "CONVERTER-DIED exit=$? $(date)" >> /ws/converter.log
  sleep 2
done' >/dev/null 2>&1 &

sleep 2
echo "[ORCH] start system (autonomyMode on)" >> /ws/orchestrator.log
nohup ros2 launch /ws/system_isaac_sim.launch.py \
  sensorOffsetX:=0.27 sensorOffsetY:=0.0 > /ws/system.log 2>&1 &

sleep 5
echo "[ORCH] up. pids:" >> /ws/orchestrator.log
pgrep -af "pc2_to_livox|ros2 launch" >> /ws/orchestrator.log 2>&1
echo "[ORCH] done" >> /ws/orchestrator.log
