#!/usr/bin/env bash
# navstack 容器的 PID-1 主管进程：转换器 + SLAM 系统，各带无限重生。
# 作为 docker run 的 CMD 运行（不走 docker exec —— exec 会话树会被神秘连坐 SIGKILL，
# 多轮取证：只有 exec 树下的进程死，PID-1 树下的从不死）。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-184}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4  # 禁 SHM：击杀留下的僵尸段会让 SHM 传输静默瘫痪
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

# NAV_MODE 选 system launch 变体（由容器 docker run -e NAV_MODE=.. 注入；缺省 waypoint）：
#   waypoint = system_isaac_sim.launch.py（纯手动 waypoint 导航，无探索规划器）
#   explore  = system_isaac_sim_with_exploration.launch.py（额外拉 TARE 探索规划器）
# 未知值一律回退 waypoint（fail-safe：不因拼写错就整条 system 链起不来）。
NAV_MODE="${NAV_MODE:-waypoint}"
NAV_MAX_SPEED="${NAV_MAX_SPEED:-0.25}"
NAV_AUTONOMY_SPEED="${NAV_AUTONOMY_SPEED:-0.20}"
NAV_MAX_ACCEL="${NAV_MAX_ACCEL:-0.35}"
NAV_MAX_YAW_RATE_DEG_S="${NAV_MAX_YAW_RATE_DEG_S:-15.0}"
PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD="${PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD:-0.10}"
NAV_ROBOT_CONFIG="${NAV_ROBOT_CONFIG:-unitree/unitree_go2}"
LOCAL_PLANNER_GOAL_REACHED_THRESHOLD="${LOCAL_PLANNER_GOAL_REACHED_THRESHOLD:-0.15}"
LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE="${LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE:-0.20}"
NAV_SPEED="${NAV_SPEED:-$NAV_AUTONOMY_SPEED}"
export NAV_MAX_SPEED NAV_AUTONOMY_SPEED NAV_MAX_ACCEL NAV_MAX_YAW_RATE_DEG_S
export PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD
export NAV_SPEED NAV_ROBOT_CONFIG LOCAL_PLANNER_GOAL_REACHED_THRESHOLD
export LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE
if ! python3 /ws/validate_nav_profile.py \
    --max-speed "$NAV_MAX_SPEED" \
    --autonomy-speed "$NAV_AUTONOMY_SPEED" \
    --bridge-speed "$NAV_SPEED" \
    --command-ramp "$NAV_MAX_ACCEL" \
    --yaw-rate-deg-s "$NAV_MAX_YAW_RATE_DEG_S" \
    --stop-distance "$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD" \
    --goal-threshold "$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD" \
    --obstacle-height "$LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE" \
    --robot-config "$NAV_ROBOT_CONFIG" \
    --config-root /ws/src/base_autonomy/local_planner/config; then
  echo "[SUPERVISOR] invalid navigation profile; refusing to start" >&2
  exit 2
fi
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
echo "[SUPERVISOR] startup nav profile=${GO2W_NAV_PROFILE:-direct} robot_config=$NAV_ROBOT_CONFIG speed=$NAV_AUTONOMY_SPEED/$NAV_MAX_SPEED command_ramp_100hz=$NAV_MAX_ACCEL yaw=$NAV_MAX_YAW_RATE_DEG_S stop=$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD goal=$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD obstacle=$LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE" \
  >> /ws/supervisor.log
(
  while true; do
    ros2 launch "/ws/$SYSTEM_LAUNCH" sensorOffsetX:=0.27 sensorOffsetY:=0.0 \
      robotConfig:="$NAV_ROBOT_CONFIG" \
      maxSpeed:="$NAV_MAX_SPEED" autonomySpeed:="$NAV_AUTONOMY_SPEED" \
      maxAccel:="$NAV_MAX_ACCEL" maxYawRate:="$NAV_MAX_YAW_RATE_DEG_S" \
      stopDisThre:="$PATH_FOLLOWER_STOP_DISTANCE_THRESHOLD" \
      goalReachedThreshold:="$LOCAL_PLANNER_GOAL_REACHED_THRESHOLD" \
      obstacleHeightThre:="$LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE" \
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
