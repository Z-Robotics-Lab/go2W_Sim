# go2W_Sim — Unitree Go2W 仿真与开发环境

传感器版 Go2W（轮足 + PiPER 机械臂 + Livox Mid-360 前倾 20° + 手眼 D435 + NUC 配重，
背部总载 6.5kg）的 Isaac Sim 数字孪生。基座：Isaac Sim 5.1（Docker）+ Isaac Lab v2.3.2 +
robot_lab v2.3.2（内置 Go2W RL 任务）。

- 整体技术路线（真机接入 / SLAM+Nav2 / VLN / 抓取）：[docs/go2w-dev-roadmap.md](docs/go2w-dev-roadmap.md)
- 仿真里程碑计划（M1 机器人+场景 / M2 传感器真实读数）：[docs/sim-plan.md](docs/sim-plan.md)

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

## 已踩坑清单（scripts 已内置规避，改脚本前必读）

1. pip 直装 isaacsim 不可行（pypi.nvidia.com 大 wheel 断流且 pip 不能续传）→ 用 Docker 镜像。
2. kit python 缺 setuptools（pkg_resources）→ setup 脚本已补。
3. pip 运行中自升级会把 pip 弄成新旧混合损坏态 → get-pip 全新重装。
4. `TERM=dumb` 会让 isaaclab.sh 直接退出 → 一律 `TERM=xterm`。
5. **不要用 isaaclab.sh --install**：会强制在线下载 torch → 手动 pip 装六个源码包。
6. **绝不 pip install/uninstall torch**：镜像 prebundle 自带 torch/torchvision/torchaudio
   2.7.0+cu128（CUDA 配好）；pip 碰它必坏（亲测卸载会把 prebundle 拆成空壳）。
7. 大 wheel（triton 150M/warp 133M）用 `wget -c` 预下载再本地装；aria2 多连接会被
   阿里云 403。
8. **rsl-rl-lib 必须钉 3.1.2**（isaaclab_rl 官方 pin）；装最新 5.x 报 `KeyError: 'actor'`。
9. 容器缺 git → rsl_rl 日志器 `ImportError: Bad git executable`。
10. **headless + --enable_cameras 会静默死**（两次复现：卡在启动 10 秒处然后无声消失）
    → 截图/迭代一律用 GUI 模式（`run_gui.sh --screenshot ...`）。
11. 容器内 Python 输出有缓冲，日志看不到 print → `PYTHONUNBUFFERED=1`。
12. 轮关节命名：URDF/USD 是 `*_foot_joint`，MuJoCo 官方模型是 `*_wheel_joint`（sim2sim 要映射）。
13. mid360.stl 是毫米单位+角点原点（scale 0.001 + 平移居中）；d435.dae 是米单位
    （姿态沿用 realsense2_description 官方 xacro）。
14. GitHub 上不少 mesh 走 Git-LFS，raw 链接只给指针 → 用 media.githubusercontent.com。
15. 一次只跑一个 sim 实例（GPU/内存都吃紧）；重启场景前 `pkill -9 -f "kit/pytho[n]"`。
16. **GUI 模式下 Kit 时间线 = 物理时间 × (rendering_dt/物理dt)**：每个物理步都触发 app
    更新且时间线前进 rendering_dt。必须 `render_interval=1` 且 dt 一致，否则 RTX 雷达
    时间戳跑倍速，SLAM 直接发散（实测 2 倍速 → z 漂到 -7.5m）。
17. RTX 雷达 helper 按扫描**完成时刻**打帧戳：转 CustomMsg 时必须回溯一个扫描周期
    （否则每帧都"来自未来"，SLAM 等 IMU 覆盖等到 buffer 爆）。实测旋转周期 0.2 sim-s
    （配置写 10Hz 也一样，内部系数），转换器按 0.2s 铺 offset_time。
18. isaaclab 的 `Imu.gravity_bias` 是本体系加常量，只对水平安装正确——斜装传感器必须
    自己按姿态投影重力（quat_apply_inverse），否则 SLAM 拿到错误重力方向。
19. 物理 100Hz 时腿部 PD 60/2 站不稳会摔（截图实锤）；100/5 稳。
20. DDS 必须隔离域（本仓库约定 ROS_DOMAIN_ID=42）：域 0 会和主机上其他 ROS 项目串台。
21. 手动 cmake install 到 /usr/local 后要 `ldconfig`，否则节点起不来（libgtsam 找不到）。
22. 编排脚本别用 `set -u`（ROS setup.bash 有 unbound 变量，直接静默死）。
23. **宿主机上其他项目的定时清理会跨命名空间杀容器内同 uid 进程**（本机实证：
    vector_os_nano 循环每轮末的 `rosm nuke` 定点清除我们 navstack 里的 ROS 进程，
    时间戳 ±1s 吻合）。防御：关键容器用 root 运行（`--user 0`，非 root 清理器 EPERM）+
    PID-1 supervisor 自愈（run_all_forever.sh）+ 配对重启脚本（restart_all.sh）。
24. fastdds 大消息（900KB 点云）+ 频繁进程死亡环境：禁 SHM 强制 UDP
    （FASTDDS_BUILTIN_TRANSPORTS=UDPv4），/dev/shm 僵尸段会让 SHM 静默瘫痪。
25. **自检必须与生产数据通路隔离**：navstack 的 pathFollower 无目标时持续发
    cmd_vel=0，会把自检的注入指令每帧覆盖（第 7 轮自检取证）。
26. 手搓四轮差速在 Go2W 上物理不可行（轴距>轮距的定轴四轮滑移转向：3s 只转 2-4°）。
    正解 = robot_lab 训练速度跟踪 RL 策略（真机 wheeled_sport 的仿真等价物）。
27. isaaclab `Imu` 的 OffsetCfg.rot 不作用于测量值——斜装传感器的水平化在发布器里
    用常量旋转自己做。

## 导航栈集成（M3，已验收）

Isaac 当机器人、CMU 导航栈（refs 见 docs/sim-plan.md）当大脑，全链路已跑通：
waypoint → FAR/local planner → cmd_vel → 差速轮速 → Go2W 在 full_warehouse 自主行驶
至目标（SLAM 与地面真值交叉验证一致，静止漂移 5cm/20s）。

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
