#!/usr/bin/env bash
# 启动 vector_os_nano 的 vector-cli REPL，挂载 isaac-go2w world（BYO，零内核修改）。
# 前置：bash scripts/nav/restart_all.sh 已 ALL-GREEN（Isaac + 导航栈 + HTTP 桥）。
# 用法：bash scripts/vector_os/run_agent.sh
#   REPL 里自然语言下达目标，如："把狗开到坐标 (2, -1) 附近"
#   验收谓词写法：go2w_at(2.0, -1.0) == True
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VON="${VECTOR_OS_NANO_DIR:-$HOME/Desktop/vector_os_nano}"

curl -sf --max-time 3 http://127.0.0.1:8042/gt >/dev/null \
  || { echo "桥不在线——先跑 scripts/nav/restart_all.sh"; exit 1; }

cd "$VON"
PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}" exec .venv/bin/python -c "
import sys
import isaac_go2w_world  # 注册 isaac-go2w world（BYO）
import vector_os_nano.vcli.cli as cli
from vector_os_nano.vcli.worlds.registry import get_world_registry

# 世界解析 shim：固定用 isaac-go2w（仍是 registry 正门，不碰内核代码）
cli._resolve_active_world = (
    lambda args, agent: get_world_registry().resolve('isaac-go2w'))
sys.argv = ['vector-cli'] + sys.argv[1:]
cli.main()
" "$@"
