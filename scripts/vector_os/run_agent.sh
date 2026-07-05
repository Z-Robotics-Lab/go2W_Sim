#!/usr/bin/env bash
# 启动 vector_os_nano 的 vector-cli，挂载 isaac-go2w world（BYO，零内核修改）。
# 前置：bash scripts/nav/restart_all.sh 已 ALL-GREEN（Isaac + 导航栈 + HTTP 桥）。
# 用法：bash scripts/vector_os/run_agent.sh                       # 交互 REPL
#       bash scripts/vector_os/run_agent.sh --no-permission -p "导航到 (2,0)"  # 单发
# 两个 shim（都在本 runner 内，vector_os_nano 仓库零改动）：
#   1. 世界解析固定 isaac-go2w（walk registry 正门）
#   2. 工具发现附加 go2w 工具（其 CLI 的 world.register_tools 接线属 Phase C 未落地）
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VON="${VECTOR_OS_NANO_DIR:-$HOME/Desktop/vector_os_nano}"

curl -sf --max-time 3 http://127.0.0.1:8042/gt >/dev/null \
  || { echo "桥不在线——先跑 scripts/nav/restart_all.sh"; exit 1; }

cd "$VON"
PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}" exec .venv/bin/python -c "
import sys
import isaac_go2w_world as w  # 注册 isaac-go2w world（BYO）
import vector_os_nano.vcli.cli as cli
from vector_os_nano.vcli.worlds.registry import get_world_registry

# shim 1: 世界解析固定 isaac-go2w
cli._resolve_active_world = (
    lambda args, agent: get_world_registry().resolve('isaac-go2w'))

# shim 2: 工具发现附加 go2w 工具（补 CLI 未接的 world.register_tools 线）
_orig = cli.discover_categorized_tools
def _with_go2w():
    tools, cat_map = _orig()
    tools = list(tools) + [w.Go2WNavigateTool(), w.Go2WWhereTool()]
    cat_map = dict(cat_map)
    cat_map['go2w'] = ['go2w_navigate', 'go2w_where']
    return tools, cat_map
cli.discover_categorized_tools = _with_go2w

# shim 3: _init_agent 正门——无 sim 参数时返回 isaac-go2w embodiment，
# 技能表/VGG 就绪/skill 包装全部走 CLI 原生路径
_orig_init_agent = cli._init_agent
def _init_agent_go2w(args):
    a = _orig_init_agent(args)
    return a if a is not None else w.IsaacGo2WEmbodiment()
cli._init_agent = _init_agent_go2w

sys.argv = ['vector-cli'] + sys.argv[1:]
cli.main()
" "$@"
