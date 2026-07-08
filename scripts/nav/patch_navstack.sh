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

# 1b. 斜装雷达补偿：雷达相对（虚拟平装）IMU 前倾 20 度（单位: 度）。
# 背景：栈的 imu_acc 限幅假设 IMU 水平，Isaac 侧 IMU 已用 -20° 姿态偏置拉平。
sed -i 's/data: \[0.0, 0.0, 0.0\] #\[0.0, 0.5, 0.0\]/data: [0.0, 20.0, 0.0] # lidar pitched 20 deg vs level imu/' \
  "$NAV"/src/slam/arise_slam_mid360/config/livox/livox_mid360_calibration.yaml

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
# realRobot=false：关掉 pathFollower 的 /dev/ttyACM0 串口重试（T-Bot 真机底盘用，
# 我们走 /cmd_vel topic）；autonomyMode=true：无手柄自主使能
out = out.replace("""      'realRobot': 'true',""", """      'realRobot': 'false',
      'autonomyMode': 'true',
      'maxSpeed': '0.6',
      'autonomySpeed': '0.6',""")
(nav / "system_isaac_sim.launch.py").write_text(out)
print("system_isaac_sim.launch.py written")
PYEOF

# 5. 探索变体 launch：从 sim 基线 system_isaac_sim.launch.py 派生
#    system_isaac_sim_with_exploration.launch.py —— 追加 TARE 探索规划器块（照抄
#    system_*_with_exploration_planner.launch 的 start_tare_planner），并把 kAutoStart
#    改成 false（改由 agent 发一次 /start_exploration(Bool true) 显式触发探索）。
python3 - "$NAV" <<'PYEOF'
import sys
from pathlib import Path
nav = Path(sys.argv[1])

src = (nav / "system_isaac_sim.launch.py").read_text()
out = src

# 5a. 顶部加 exploration_planner_config 这个 LaunchConfiguration（照抄参考 with_exploration）
out = out.replace(
    "  world_name = LaunchConfiguration('world_name')\n",
    "  world_name = LaunchConfiguration('world_name')\n"
    "  exploration_planner_config = LaunchConfiguration('exploration_planner_config')\n",
    1)

# 5b. declare 块加 exploration_planner_config 声明（default indoor_small）
out = out.replace(
    "  declare_world_name = DeclareLaunchArgument('world_name', default_value='real_world', description='')\n",
    "  declare_world_name = DeclareLaunchArgument('world_name', default_value='real_world', description='')\n"
    "  declare_exploration_planner_config = DeclareLaunchArgument('exploration_planner_config', default_value='indoor_small', description='')\n",
    1)

# 5c. 在 ld = LaunchDescription() 前插入 start_tare_planner 定义。
# 关键修正（核对真相源得出）：参考的 start_tare_planner 块只传 scenario、不传
# use_sim_time；explore_world.launch 的 use_sim_time 默认 false。Isaac 全链走 /clock
# 仿真时钟，这里必须显式传 use_sim_time:='true'，否则 TARE 用墙钟、与 SLAM/terrain
# 时间戳错乱。scenario 用 indoor_small（kAutoStart 已 sed 成 false，agent 显式触发）。
tare_block = (
    "  # TARE 探索规划器（照抄 system_*_with_exploration_planner.launch 的 start_tare_planner，\n"
    "  # 追加 use_sim_time:='true' —— Isaac /clock 全链仿真时钟，缺它 TARE 时间戳错乱）。\n"
    "  start_tare_planner = IncludeLaunchDescription(\n"
    "    PythonLaunchDescriptionSource(\n"
    "      [get_package_share_directory('tare_planner'), '/explore_world.launch']),\n"
    "    launch_arguments={\n"
    "      'scenario': exploration_planner_config,\n"
    "      'use_sim_time': 'true',\n"
    "    }.items()\n"
    "  )\n\n"
    "  ld = LaunchDescription()\n"
)
out = out.replace("  ld = LaunchDescription()\n", tare_block, 1)

# 5d. add_action：declare 挂在 declare 段末尾（checkTerrainConn 之后），
# start_tare_planner 挂在动作段末尾（return ld 前，探索链最后一环）。
out = out.replace(
    "  ld.add_action(declare_checkTerrainConn)\n",
    "  ld.add_action(declare_checkTerrainConn)\n"
    "  ld.add_action(declare_exploration_planner_config)\n",
    1)
out = out.replace(
    "\n  return ld\n",
    "  ld.add_action(start_tare_planner)\n\n  return ld\n",
    1)

(nav / "system_isaac_sim_with_exploration.launch.py").write_text(out)
print("system_isaac_sim_with_exploration.launch.py written")

# 5e. indoor_small.yaml: kAutoStart true->false（agent 显式触发探索）。
# kRushHome / kNoExplorationReturnHome 保持原值不动（探索结束回家行为不改）。
cfg = nav / "src/exploration_planner/tare_planner/config/indoor_small.yaml"
y = cfg.read_text()
assert "kAutoStart : true" in y, "kAutoStart : true 未找到（config 格式变了？）"
cfg.write_text(y.replace("kAutoStart : true", "kAutoStart : false", 1))
print("indoor_small.yaml kAutoStart -> false")
PYEOF

# 6. 转换节点 + 编排脚本
bash "$HERE/sync_navstack_files.sh" "$NAV"

# 7. refs-only 定制补丁集（既非本脚本 sed/python 产物、也非 sync 真源的直改）：
#    imuPreintegration.cpp sensor_mount_pitch_deg（36b8e1b C++ 半）、local_planner.launch
#    obstacleHeightThre=0.20（fix-c）、arize_slam.launch.py respawn。幂等；补丁基线=本脚本
#    产物之上，故必须在此步（sed/inject 之后）应用。归属见 navstack_patch/manifest.md。
#    末尾会打印需容器内重建的包清单（arise_slam_mid360）。
bash "$HERE/navstack_patch/apply.sh" "$NAV"

echo "OK: nav stack patched for Isaac sim integration"
