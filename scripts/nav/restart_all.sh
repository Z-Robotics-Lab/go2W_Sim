#!/usr/bin/env bash
# 全链配对重启（宿主机执行）。铁律：Isaac 桥与 navstack 必须成对重启——
# 单侧重启后 fastdds 不会重新发现旧对端（多轮实证：传感器流单向断绝，SLAM 空转）。
# SHM 已全链禁用（FASTDDS_BUILTIN_TRANSPORTS=UDPv4）：进程被杀留下的僵尸段会让
# SHM 传输静默瘫痪（/dev/shm 实测 100+ 尸块）。
# 顺序：navstack(PID-1 supervisor) -> Isaac 桥 -> 门控验证。
set -e
GO2W="$(cd "$(dirname "$0")/../.." && pwd)"
NAV="$GO2W/refs/Navigation-Physical-Experiment"
# RL locomotion 策略（robot_lab 训练；差速在 Go2W 上物理不可行——README 坑 26）
# 旧默认(出厂 ckpt,回滚即换回): /workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-04_15-52-42/model_1999.pt
# 切换记录: 2026-07-07 配方 v2 Round-3(model_3497)产品裁定落地,见 docs/sim-plan.md
# 回滚点(2026-07-07 载荷轮前默认;已在 6.46kg 新体重锚验证 ①0.0049/④0.0337+0.0063): /workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-07_06-51-14/model_3497.pt
# 切换记录: 2026-07-07 载荷轮 model_5495 落地(⑤门形修正后全门过),见 docs/sim-plan.md
POLICY="${GO2W_POLICY:-/workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-07_07-53-57/model_5495.pt}"

bash "$GO2W/scripts/nav/sync_navstack_files.sh" "$NAV"  # 真相源同步（防旧拷贝）
echo "[1/4] navstack supervisor 重启"
docker rm -f navstack >/dev/null 2>&1 || true
docker run -d --name navstack --net=host --ipc=host --init --memory 20g --user 0 \
  -e ROS_DOMAIN_ID=42 -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e NAV_MODE="${NAV_MODE:-waypoint}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$NAV":/ws -w /ws \
  navstack:ready bash /ws/run_all_forever.sh >/dev/null

echo "[2/4] Isaac 桥重启"
docker exec -u 0 go2w-isaac bash -c 'pkill -9 -f "kit/pytho[n]" 2>/dev/null; sleep 2' || true
docker exec -d -u 0 -e DISPLAY="${DISPLAY:-:0}" -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib -e PYTHONUNBUFFERED=1 \
  go2w-isaac bash -c "cd /workspace/go2w/scripts/sim && TERM=xterm \
  /isaac-sim/python.sh warehouse_nav.py --env warehouse --enable_cameras --policy $POLICY \
  --shot_dir /workspace/go2w/logs/shots > /workspace/go2w/logs/nav_bridge.log 2>&1"

echo "[3/4] 等 Isaac 就绪（IMU 样本出现）"
until grep -qa "imu sample" "$GO2W/logs/nav_bridge.log" 2>/dev/null; do sleep 15; done

echo "[4/4] 门控验证（传感器到达 + SLAM 输出）"
docker exec navstack bash -c "export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
  timeout 15 ros2 topic hz /imu/data 2>&1 | grep average && \
  timeout 25 ros2 topic hz /state_estimation 2>&1 | grep average" \
  && echo "ALL-GREEN: 全链就绪" || { echo "GATE-FAILED: 传感器/SLAM 未通"; exit 1; }
