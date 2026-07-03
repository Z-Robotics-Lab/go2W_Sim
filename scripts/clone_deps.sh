#!/usr/bin/env bash
# 拉取三个第三方依赖并锁定到验证过的版本组合。
# 版本组合依据 robot_lab README 的兼容表：robot_lab v2.3.2 <-> Isaac Lab v2.3.2 <-> Isaac Sim 5.1
set -euo pipefail
cd "$(dirname "$0")/.."

if [ ! -d IsaacLab ]; then
  git clone --filter=blob:none https://github.com/isaac-sim/IsaacLab.git
fi
git -C IsaacLab checkout v2.3.2

if [ ! -d robot_lab ]; then
  git clone --filter=blob:none https://github.com/fan-ziqi/robot_lab.git
fi
git -C robot_lab checkout v2.3.2

mkdir -p assets
if [ ! -d assets/unitree_ros ]; then
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/unitreerobotics/unitree_ros.git assets/unitree_ros
  git -C assets/unitree_ros sparse-checkout set robots/go2w_description
fi

echo "OK: IsaacLab v2.3.2 / robot_lab v2.3.2 / unitree_ros(go2w_description)"
