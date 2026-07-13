# STATUS — go2W_Sim 会话恢复锚点（覆盖式，≤40 行）

更新：2026-07-10。分支 `feat/go2w-office-scene`，未 push；office+waypoint 活栈已重启验证。

## 当前里程碑
- Isaac+CMU导航、RL策略、agent E2E、P5.1一键拉起、P5.2探索均已跑通。
- Office 迁移门 a/b/c/d/e/g PASS；门 f 因到点后 idle 漂移未 hold，仍依赖 fix-a/W1-W3。
- 载荷策略 `model_5495` 已落地；抓取 WIP 暂停在 `feat/grasp-wip`。

## 2026-07-10 Mid-360 倾斜根因与修复
- 在线基线：原始 `/lidar/points` 地面倾角 2.11°，旧 `/registered_scan` 20.54°，旧 IMU重力 17.94°。
- 根因（2026-07-13 frame 语义复核）：原始 LiDAR/IMU 共享物理 +20° 测量系，ARISE 已用
  同一重力旋转水平化二者；桥手工 Ry(+20°) + SLAM offset(+20°)属于重复补偿。
- 修复：原始点云显式标为 `mid360_raw`；ARISE 虚拟 `sensor` 保持水平；IMU桥恒等传递，
  仿真 `imu_laser_rotation_offset` 强制归零。
- 二次根因：ARISE install 中残留旧 +20° 普通文件，且 navstack 先于机器人站稳启动，固化落地瞬态。
- 启动现校验 source→install 配置一致，并等 Isaac step=800 后再起 navstack/RViz；green 门控有界等60s。
- `patch_navstack.sh` 同时修复当前 GitHub 已改名为 `system_real_robot.launch.py` 的兼容问题。
- 新增坐标契约测试；干净 `origin/dev` worktree 全量 patch 成功，两个生成 launch 可解析。

## 验证状态
- `tests/test_sensor_frame_contract.py`：5/5 PASS；bash语法、py_compile、runtime config cmp PASS。
- 修后在线拟合：raw 2.110°、sensor_scan 0.657°、RViz `/registered_scan` 0.711°；IMU pitch -2.064°。
- `status.sh`：L0-L5、upright、green 全 true；RViz/SLAM/Isaac 均为本轮新进程。

## 运行拓扑/铁律
- `go2w-isaac`：warehouse_nav.py（RL/传感桥/GT，DDS 42）。
- `navstack`：supervisor（转换器+SLAM+HTTP :8042+RViz）。
- 只能配对重启：`GO2W_SCENE=office bash scripts/nav/bringup.sh teardown|up`；单重启 navstack 会毁低RTF SLAM。
- NEVER-KILL-INFRA；`scripts/nav` 是运行时真相源。

## 下一步
1. 可视确认当前 RViz 点云；数值门 `<2°` 已通过。
2. 复跑短 waypoint 与默认 warehouse 等价回归。
3. 之后再处理 fix-a/W1-W3；本轮不碰 planner C++、`pc2_to_livox`、render_interval/fullScan 红线。
