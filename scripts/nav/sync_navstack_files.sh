#!/usr/bin/env bash
# 把 scripts/nav/ 的运行时文件同步进 refs 克隆（容器挂 /ws 跑的是 refs 里的拷贝）。
# 真相源永远是 scripts/nav/——本脚本被 bringup.sh / restart_all.sh 在起容器前调用，
# 保证仓库更新后容器不会跑旧拷贝（2026-07-06 集成实证：桥/supervisor 旧拷贝导致
# /health 404 + RViz 不起）。幂等，可重复调用。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
NAV="${1:-$HERE/../../refs/Navigation-Physical-Experiment}"
NAV="$(cd "$NAV" && pwd)"
for f in pc2_to_livox.py run_navstack.sh agent_bridge.py run_all_forever.sh; do
  cp "$HERE/$f" "$NAV/$f"
done
# SLAM IMU<->Laser 外参（单一真源，CEO A线指令 2026-07-07）：此前仅在 gitignore 的 refs/ 里，
# 仓库更新带不上 -> 纳入 sync。目标=容器 /ws 里 arise_slam 生效 config 路径。
SLAM_CALIB_DST="$NAV/src/slam/arise_slam_mid360/config/livox/livox_mid360_calibration.yaml"
if [ -d "$(dirname "$SLAM_CALIB_DST")" ]; then
  cp "$HERE/livox_mid360_calibration.yaml" "$SLAM_CALIB_DST"
  # 主配置（含 sensor_mount_pitch_deg=20：odom/TF 车体系化，A线第二层修复 2026-07-08）
  cp "$HERE/livox_mid360.yaml" "$NAV/src/slam/arise_slam_mid360/config/livox_mid360.yaml"
  echo "[sync] SLAM 外参+主配置 -> $NAV/src/slam/arise_slam_mid360/config/"
else
  echo "[sync] WARN: SLAM config 目录不存在（refs 未克隆？）跳过外参同步" >&2
fi
# P5.2 探索变体 launch：.reference 快照即真相（与 patch 第5步产物逐字节一致已验证）
cp "$HERE/system_isaac_sim_with_exploration.launch.py.reference" \
   "$NAV/system_isaac_sim_with_exploration.launch.py"
# RViz 面板去毒（坑29/32）：从 stock 配置生成去掉 TeleopPanel 的 go2w.rviz——
# 面板发的 /joy 会关自主模式并锁死速度；操作者面板不属于 agent 产品面。
# 另（CEO 2026-07-10 眼见要求）：stock 的两个 Image 面板订的 /camera/image 与
# /camera/semantic_image 在本链不存在（面板全黑）——改指腕相机彩色/对齐深度，
# RViz 里即是"狗眼第一视角"，抓取测试全程可肉眼盯。
python3 - "$NAV" <<'PYEOF'
import sys, re
nav = sys.argv[1]
src = f"{nav}/src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz"
txt = open(src).read()
# Panels 段里移除 teleop 面板条目（缩进块）；显示区不动
txt = re.sub(r"  - Class: teleop_rviz_plugin.*?(?=  - Class: |Visualization Manager:)",
             "", txt, flags=re.S)
# Image 面板重定向到腕相机（M0 realsense2 口径话题）+ 面板改名见名知意
txt = txt.replace("Value: /camera/image", "Value: /camera/color/image_raw")
txt = txt.replace("Name: Image", "Name: WristColor")
txt = txt.replace("Value: /camera/semantic_image",
                  "Value: /camera/aligned_depth_to_color/image_raw")
txt = txt.replace("Name: SemanticImage", "Name: WristDepth")
# Z-Manip M1 感知可视化（CEO 点名：分割掩码叠加/目标点云/追踪框）：向 Visualization
# Manager 的 Displays 列表尾追加三个面板——Perception(overlay Image)/TargetCloud
# (PointCloud2)/TargetMarker(Marker)。缩进必须对齐 stock Display 列表项（4 空格 '-'、
# 6 空格子键、8 空格 Topic 子键），错一格 RViz 静默不加载（坑）。QoS：图像流用 Best
# Effort 对齐感知节点高频发布；Marker 低频用 Reliable。锚点=Displays 列表末尾唯一的
# "\n  Enabled: true\n  Global Options:"（2 空格顶层 Global Options 全文件仅一处）。
M1_DISPLAYS = (
    "    - Class: rviz_default_plugins/Image\n"
    "      Enabled: true\n"
    "      Max Value: 1\n"
    "      Median window: 5\n"
    "      Min Value: 0\n"
    "      Name: Perception\n"
    "      Normalize Range: true\n"
    "      Topic:\n"
    "        Depth: 5\n"
    "        Durability Policy: Volatile\n"
    "        History Policy: Keep Last\n"
    "        Reliability Policy: Best Effort\n"
    "        Value: /perception/overlay\n"
    "      Value: true\n"
    "    - Class: rviz_default_plugins/PointCloud2\n"
    "      Enabled: true\n"
    "      Name: TargetCloud\n"
    "      Size (Pixels): 4\n"
    "      Style: Points\n"
    "      Color: 255; 0; 255\n"
    "      Color Transformer: FlatColor\n"
    "      Topic:\n"
    "        Depth: 5\n"
    "        Durability Policy: Volatile\n"
    "        History Policy: Keep Last\n"
    "        Reliability Policy: Best Effort\n"
    "        Value: /perception/target_cloud\n"
    "      Value: true\n"
    "    - Class: rviz_default_plugins/Marker\n"
    "      Enabled: true\n"
    "      Name: TargetMarker\n"
    "      Topic:\n"
    "        Depth: 5\n"
    "        Durability Policy: Volatile\n"
    "        History Policy: Keep Last\n"
    "        Reliability Policy: Reliable\n"
    "        Value: /perception/target_marker\n"
    "      Value: true\n"
)
new_txt, n_inj = re.subn(r"(\n  Enabled: true\n  Global Options:)",
                         "\n" + M1_DISPLAYS.rstrip("\n") + r"\1", txt, count=1)
if n_inj != 1:
    raise SystemExit(f"[sync] FATAL: M1 感知面板注入锚点未命中(n={n_inj})——stock rviz 结构变了？")
txt = new_txt
open(f"{nav}/go2w.rviz", "w").write(txt)
print("[sync] go2w.rviz (TeleopPanel removed; Image panels -> wrist cam; "
      "+M1 Perception/TargetCloud/TargetMarker)")
PYEOF
echo "[sync] scripts/nav -> $NAV OK"
