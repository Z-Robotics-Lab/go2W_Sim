#!/usr/bin/env bash
# navstack 容器的 PID-1 主管进程：转换器 + SLAM 系统，各带无限重生。
# 作为 docker run 的 CMD 运行（不走 docker exec —— exec 会话树会被神秘连坐 SIGKILL，
# 多轮取证：只有 exec 树下的进程死，PID-1 树下的从不死）。
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4  # 禁 SHM：击杀留下的僵尸段会让 SHM 传输静默瘫痪
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

echo "[SUPERVISOR] $(date) start" > /ws/supervisor.log

(
  while true; do
    python3 -u /ws/pc2_to_livox.py >> /ws/converter.log 2>&1
    echo "CONVERTER-DIED exit=$? $(date)" >> /ws/converter.log
    sleep 2
  done
) &

(
  while true; do
    python3 -u /ws/agent_bridge.py >> /ws/bridge.log 2>&1
    echo "BRIDGE-DIED exit=$? $(date)" >> /ws/bridge.log
    sleep 2
  done
) &

(
  while true; do
    ros2 launch /ws/system_isaac_sim.launch.py sensorOffsetX:=0.27 sensorOffsetY:=0.0 \
      >> /ws/system.log 2>&1
    echo "SYSTEM-DIED exit=$? $(date)" >> /ws/system.log
    sleep 3
  done
) &

# RViz 可视化（软能力，死后重生）。headless 环境（DISPLAY 空）不空转刷日志——
# 直接 sleep infinity 挂着，让 supervisor 的 wait 不提前返回。
(
  if [ -z "$DISPLAY" ]; then
    echo "[RVIZ] DISPLAY 为空，headless——不启动 RViz（sleep infinity）" >> /ws/rviz.log
    sleep infinity
  fi
  while true; do
    ros2 run rviz2 rviz2 \
      -d /ws/src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz \
      >> /ws/rviz.log 2>&1
    echo "RVIZ-DIED exit=$? $(date)" >> /ws/rviz.log
    sleep 3
  done
) &

wait
