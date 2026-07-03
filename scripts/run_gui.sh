#!/usr/bin/env bash
# 在桌面上打开 Isaac Sim GUI 跑仓库场景（X11 直通容器）。
# 用法: bash scripts/run_gui.sh [warehouse_scene.py 的其他参数...]
set -euo pipefail
xhost +local: >/dev/null
docker exec -u 0 -e DISPLAY="${DISPLAY:-:0}" go2w-isaac bash -c \
  "cd /workspace/go2w/scripts/sim && TERM=xterm /isaac-sim/python.sh warehouse_scene.py $*"
