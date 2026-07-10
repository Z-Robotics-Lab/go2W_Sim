# go2W_Sim — Unitree Go2W 仿真与开发环境

传感器版 Go2W（轮足 + PiPER 机械臂 + Livox Mid-360 前倾 20° + 手眼 D435 + NUC 配重，
背部总载 6.5kg）的 Isaac Sim 数字孪生。基座：Isaac Sim 5.1（Docker）+ Isaac Lab v2.3.2 +
robot_lab v2.3.2（内置 Go2W RL 任务）。

- 整体技术路线（真机接入 / SLAM+Nav2 / VLN / 抓取）：[docs/go2w-dev-roadmap.md](docs/go2w-dev-roadmap.md)
- 仿真里程碑计划（M1 机器人+场景 / M2 传感器真实读数）：[docs/sim-plan.md](docs/sim-plan.md)
- **使用手册（仿真+真机，从开机到对话）**：[docs/user-manual.md](docs/user-manual.md)
- **NUC 配置（真机大脑就绪清单）**：[docs/nuc-setup.md](docs/nuc-setup.md)
- 踩坑清单（27+ 条，改脚本前必读）：[docs/pitfalls.md](docs/pitfalls.md)

## 日常使用（环境已装好后）

```bash
bash scripts/run_gui.sh --env flat              # GUI 打开平地场景看机器人
bash scripts/run_gui.sh --env flat --drive 2.0  # 轮子转起来
bash scripts/run_gui.sh --env warehouse         # 内置仓库场景（首次联网下资产）
# 定点截图（GUI 模式，photo 落在 logs/）：
bash scripts/run_gui.sh --env flat --screenshot /workspace/go2w/logs/shot.png \
    --cam_eye 1.1 0.5 0.75 --cam_target 0.27 0 0.45
```

窗口操作：Alt+左键旋转视角，滚轮缩放；左侧 Stage 树可查看/选中每个 link 和 joint。

## 资产地图（改机器人去哪里）

| 内容 | 位置 |
|---|---|
| **合成 URDF（机器人本体+全部挂载）** | `assets/urdf/go2w_sensored.urdf`（生成产物，勿手改）|
| **挂载布局常量（位置/姿态/质量）** | `scripts/tools/compose_sensored_urdf.py` 的 `MOUNTS` 字典 |
| 改完重新生成 | `python3 scripts/tools/compose_sensored_urdf.py` 然后重开 GUI |
| go2w 官方 URDF/网格 | `assets/unitree_ros/robots/go2w_description/`（clone_deps.sh 拉取）|
| PiPER 官方 URDF/网格 | `assets/piper_ros/src/piper_description/`（clone_deps.sh 拉取）|
| 传感器网格 d435.dae / mid360.stl | `assets/sensor_meshes/`（fetch_sensor_meshes.sh 下载）|
| 场景脚本（环境/执行器/相机参数）| `scripts/sim/warehouse_scene.py` |
| Go2W RL 训练任务（robot_lab）| `robot_lab/.../config/wheeled/unitree_go2w/` |

已验证的机器人事实：46 关节（12 腿 revolute + 4 轮 continuous `*_foot_joint` +
PiPER 6 revolute + 夹爪 2 prismatic + 固定挂载）；夹爪逼近轴 = `gripper_base` +Z；
Mid-360 前倾 19.6°（quat 实测）；整机 26.02kg，背部载荷 6.500kg。

## 从零重建环境

```bash
git clone https://github.com/Z-Robotics-Lab/go2W_Sim.git && cd go2W_Sim
bash scripts/clone_deps.sh          # IsaacLab v2.3.2 / robot_lab v2.3.2 / go2w URDF
bash scripts/fetch_wheels.sh        # triton+warp 大 wheel 断点续传预下载
bash scripts/fetch_sensor_meshes.sh # d435/mid360 真实网格
python3 scripts/tools/compose_sensored_urdf.py
bash scripts/setup_container.sh     # 建容器+装全套（内置全部坑修复）
docker commit go2w-isaac go2w-isaac:ready   # 固化，之后重建容器秒级
bash scripts/run_gui.sh --env flat
```

冒烟验证（Go2W 平地 RL 任务 5 个迭代，headless 无渲染，可跑）：
```bash
docker exec -u 0 go2w-isaac bash -c "cd /workspace/go2w/robot_lab && TERM=xterm \
  /isaac-sim/python.sh scripts/reinforcement_learning/rsl_rl/train.py \
  --task RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0 --headless --num_envs 64 --max_iterations 5"
```

## 踩坑清单

27+ 条实测坑（pip/torch/时间线/DDS/内存/进程连坐…）全部迁至 **[docs/pitfalls.md](docs/pitfalls.md)**——改任何脚本前必读。


## 导航栈集成（M3，已验收）

Isaac 当机器人、CMU 导航栈（refs 见 docs/sim-plan.md）当大脑，全链路已跑通：
waypoint → FAR/local planner → cmd_vel → 差速轮速 → Go2W 在 full_warehouse 自主行驶
至目标（SLAM 与地面真值交叉验证一致，静止漂移 5cm/20s）。

传感器坐标约束：Mid-360 的物理 20° 姿态只存在于 URDF/USD；当前 Isaac ROS 输出已经
导航对齐，桥与 `imu_laser_rotation_offset` 均不得再次叠加 20°。

```bash
# 1) Isaac 侧（传感器桥：/lidar/points /imu/data /clock 出、/cmd_vel 入，DDS 域 42）
docker exec -d -u 0 -e DISPLAY=:0 -e ROS_DISTRO=jazzy -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib -e PYTHONUNBUFFERED=1 \
  go2w-isaac bash -c "cd /workspace/go2w/scripts/sim && TERM=xterm \
  /isaac-sim/python.sh warehouse_nav.py --env warehouse --shot_dir /workspace/go2w/logs/shots"
# 2) 导航栈容器（首次：clone 导航栈到 refs/ 并 bash scripts/nav/patch_navstack.sh refs/<repo>，
#    再按其 docker/README 构建 jazzy-dev 镜像 + colcon build）
docker exec -d navstack bash /ws/run_navstack.sh
# 3) 发导航目标
docker exec navstack bash -c "export ROS_DOMAIN_ID=42 && source /opt/ros/jazzy/setup.bash && \
  source /ws/install/setup.bash && ros2 topic pub --once /way_point \
  geometry_msgs/msg/PointStamped '{header: {frame_id: map}, point: {x: 2.5, y: 0.0, z: 0.0}}'"
```

## 状态

- [x] 容器 + 全链安装（镜像已固化 go2w-isaac:ready）
- [x] 基线冒烟：Go2W 平地 RL 任务 5 迭代跑通（rsl_rl PPO，reward 正常）
- [x] 传感器版 URDF：真实网格、FK 校准 D435 位姿、FRAME 数值验证（Mid-360 19.6°）
- [x] GUI 平地场景人工目检通过
- [x] M1：full_warehouse 场景导入（云资产已缓存，站高 0.375m 稳定）
- [x] M2（部分）：Mid-360 RTX 点云 + 物理正确 IMU（斜装重力投影）真实读数
- [x] M3 完成：方形巡航回归 4/4（34s/139s/12s/80s，含转弯与回原点闭环）
- [x] RL locomotion：robot_lab Go2W 速度策略（2000 iters, reward 114.5）替代差速——
      弧线转向 2.3°→95°（41 倍），wheeled_sport 仿真等价物，附带 vy 全向
- [x] M2 收尾：D435 RGB+深度已出 ROS2 topic（/camera/image /camera/depth，进 RViz）
- [ ] M2 尾巴：PiPER 关节 ROS2 控制接口（M4 前置）
- [x] M5/任务②：vector_os_nano agent 控狗——NL 目标 -> agent 规划 -> navigate ->
      导航栈 -> RL 策略 -> GT 谓词 GROUNDED，VECTOR_VERDICT verified=true (EXIT 0)。
      入口：scripts/vector_os/run_agent.sh（BYO world，vector_os_nano 零修改）
- [ ] 后续：接入抓取（任务③）、RViz 可视化操作、真机部署包
