#!/usr/bin/env bash
# navstack 容器的 PID-1 主管进程：转换器 + SLAM 系统，各带无限重生。
# 作为 docker run 的 CMD 运行（不走 docker exec —— exec 会话树会被神秘连坐 SIGKILL，
# 多轮取证：只有 exec 树下的进程死，PID-1 树下的从不死）。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-184}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4  # 禁 SHM：击杀留下的僵尸段会让 SHM 传输静默瘫痪
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

# NAV_MODE 选 system launch 变体（由容器 docker run -e NAV_MODE=.. 注入；缺省 waypoint）：
#   waypoint = system_isaac_sim.launch.py（纯手动 waypoint 导航，无探索规划器）
#   explore  = system_isaac_sim_with_exploration.launch.py（额外拉 TARE 探索规划器）
# 未知值一律回退 waypoint（fail-safe：不因拼写错就整条 system 链起不来）。
NAV_MODE="${NAV_MODE:-waypoint}"
case "$NAV_MODE" in
  explore) SYSTEM_LAUNCH="system_isaac_sim_with_exploration.launch.py" ;;
  waypoint) SYSTEM_LAUNCH="system_isaac_sim.launch.py" ;;
  *) echo "[SUPERVISOR] 未知 NAV_MODE='$NAV_MODE'，回退 waypoint" ; SYSTEM_LAUNCH="system_isaac_sim.launch.py" ;;
esac

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

# Standard-message visualization adapter.  It consumes only the Z-Manip
# DiagnosticArray contract and PiPER execution status, never task-object GT.
(
  while true; do
    GO2W_RVIZ_TOPICS_CONFIG=/ws/manipulation_rviz_topics.json \
      python3 -u /ws/manip_rviz_bridge.py --ros-args -p use_sim_time:=true \
      >> /ws/manip_rviz_bridge.log 2>&1
    echo "MANIP-RVIZ-BRIDGE-DIED exit=$? $(date)" >> /ws/manip_rviz_bridge.log
    sleep 2
  done
) &

echo "[SUPERVISOR] NAV_MODE=$NAV_MODE -> $SYSTEM_LAUNCH" >> /ws/supervisor.log
(
  THRE="${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE:-0.2}"
  APPLIED=""
  while true; do
    if ros2 node list 2>/dev/null | grep -qx "/localPlanner"; then
      if ros2 param set /localPlanner obstacleHeightThre "$THRE" >/dev/null 2>&1; then
        if [ "$APPLIED" != "$THRE" ]; then
          echo "[SUPERVISOR] localPlanner obstacleHeightThre=$THRE" >> /ws/supervisor.log
          APPLIED="$THRE"
        fi
        sleep 30
      else
        sleep 2
      fi
    else
      APPLIED=""
      sleep 2
    fi
  done
) &
(
  while true; do
    ros2 launch "/ws/$SYSTEM_LAUNCH" sensorOffsetX:=0.27 sensorOffsetY:=0.0 \
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
      -d /ws/go2w.rviz \
      >> /ws/rviz.log 2>&1
    echo "RVIZ-DIED exit=$? $(date)" >> /ws/rviz.log
    sleep 3
  done
) &

wait
