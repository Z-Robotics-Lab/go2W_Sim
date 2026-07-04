#!/usr/bin/env bash
# 把 Isaac-sim 集成所需的全部修改应用到一份干净的 Navigation-Physical-Experiment 克隆。
# 用法: bash scripts/nav/patch_navstack.sh <导航栈仓库路径>
# 修改清单（与 docs/sim-plan.md M3 对应）:
#   1. SLAM yaml + launch 全部切 use_sim_time:=true（Isaac 发 /clock，全链路仿真时钟）
#   2. base_autonomy 各 XML launch 的非自闭合 node 注入 use_sim_time param
#   3. 生成 system_isaac_sim.launch.py（去掉手柄和真机雷达驱动，autonomyMode:=true）
#   4. 放入 pc2_to_livox.py 转换节点与 run_navstack.sh 编排脚本
set -e
NAV="${1:?用法: patch_navstack.sh <导航栈仓库路径>}"
HERE="$(cd "$(dirname "$0")" && pwd)"

# 1. SLAM 配置与 launch 的仿真时钟
sed -i 's/use_sim_time: false/use_sim_time: true/g' "$NAV"/src/slam/arise_slam_mid360/config/*.yaml
sed -i "s/SetParameter(name='use_sim_time', value='false')/SetParameter(name='use_sim_time', value='true')/" \
  "$NAV"/src/slam/arise_slam_mid360/launch/arize_slam.launch.py

# 2. XML launch 注入（只对非自闭合 <node ...> 块）
python3 - "$NAV" <<'PYEOF'
import re, sys
from pathlib import Path
nav = Path(sys.argv[1])
for f in (nav / "src/base_autonomy").rglob("launch/*.launch"):
    t = f.read_text()
    if "use_sim_time" in t:
        continue
    nt = re.sub(r'(<node\b[^/>]*[^/]>)',
                r'\1\n    <param name="use_sim_time" value="true" />', t)
    if nt != t:
        f.write_text(nt)
        print("patched:", f.name)
PYEOF

# 3. sim 变体 launch（无手柄/真机驱动 + 自主模式）
python3 - "$NAV" <<'PYEOF'
import sys
from pathlib import Path
nav = Path(sys.argv[1])
src = (nav / "src/base_autonomy/vehicle_simulator/launch/system_real_robot.launch").read_text()
out = src.replace("  ld.add_action(start_joy)\n", "").replace("  ld.add_action(start_mid360)\n", "")
out = out.replace("""      'realRobot': 'true',""", """      'realRobot': 'true',
      'autonomyMode': 'true',
      'maxSpeed': '0.6',
      'autonomySpeed': '0.6',""")
(nav / "system_isaac_sim.launch.py").write_text(out)
print("system_isaac_sim.launch.py written")
PYEOF

# 4. 转换节点 + 编排脚本
cp "$HERE/pc2_to_livox.py" "$NAV/pc2_to_livox.py"
cp "$HERE/run_navstack.sh" "$NAV/run_navstack.sh"
echo "OK: nav stack patched for Isaac sim integration"
