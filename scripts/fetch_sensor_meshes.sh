#!/usr/bin/env bash
# 下载传感器真实网格（不入库：d435 是 Intel Apache-2.0 官方资产；mid360 来自社区仓库，
# 许可不明，故只提供下载脚本不再分发）。
#   d435.dae   IntelRealSense/realsense-ros (Apache-2.0)，米单位
#   mid360.stl SCNU-RM-Sentry (pb_rm_simulation 系)，毫米单位、角点原点，走 Git-LFS 直链
set -uo pipefail
cd "$(dirname "$0")/.." && mkdir -p assets/sensor_meshes && cd assets/sensor_meshes

fetch() {
  for i in $(seq 1 30); do wget -c -q --timeout=60 --tries=1 "$1" && return 0; sleep 3; done
  echo "FAILED: $1"; return 1
}
fetch "https://raw.githubusercontent.com/IntelRealSense/realsense-ros/ros2-development/realsense2_description/meshes/d435.dae"
fetch "https://media.githubusercontent.com/media/CodeAlanqian/SCNU-RM-Sentry/main/src/rm_simulation/pb_rm_simulation/meshes/mid360.stl"
ls -la --block-size=K *.dae *.stl
