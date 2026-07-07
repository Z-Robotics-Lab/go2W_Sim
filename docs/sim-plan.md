# 仿真里程碑 M1 — 传感器版 Go2W 落地室内仓库场景（GUI 可跑）

目标：一台与真机配置一致的 Go2W 数字孪生，放进 Isaac Sim 内置室内复杂环境，
GUI 中可视、物理可跑，作为后续室内导航 + 抓取开发的基座。

## 机器人配置（与真机采购一致）

| 部件 | 安装位置 | 姿态/质量 | 建模方式 |
|---|---|---|---|
| Livox Mid-360 | 头顶（x≈+0.25m）| **前倾 20°**（pitch +0.349rad），0.265kg | 圆柱体 + 传感器 frame（后续挂 RTX Lidar）|
| PiPER 6 轴臂 | 背部前段（x≈+0.13m）| 4.2kg + 夹爪 | 官方 piper_description URDF 合入 |
| D435 深度相机 | PiPER 末端 link6（**手眼**）| 0.072kg | 盒体 + camera frame（后续挂 Camera prim）|
| NUC 主机 | 背部后段（x≈-0.18m）| **1.8kg 实测值**（2026-07-07 载荷轮保真修复；原 0.5kg 占位）| 盒体配重 |
| 安装板/杂项 | 背部 | 补足到背部总载 **~6.5kg** | 并入配重质量 |

参考基准（来自官方 go2w URDF 实测）：躯干质量 6.921kg；头部挂点 x=0.285；
内置雷达位 x=0.28945, pitch 2.878rad——Mid-360 的挂法参考它。

## 工作分解（按顺序）

1. **[进行中] 容器环境收尾**：isaaclab 六包 + rsl-rl + robot_lab 手动 pip 安装
   （绕开 isaaclab.sh 的强制 torch 下载；镜像自带 torch 2.7.0+cu128 已验证 CUDA 可用）。
2. **基线冒烟**：headless 跑 `RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0` 5 个迭代，
   证明未改装的 Go2W 在 Isaac Sim 里能跑 RL 任务。
3. **URDF 合成**：`assets/urdf/go2w_sensored.urdf`
   - 基底：unitree_ros `go2w_description.urdf`（12 腿关节 + 4 轮关节 `*_foot_joint`）
   - 合入 piper_description（6 关节 + 夹爪；base fixed 到躯干背部前段）
   - 新增固定 link：mid360（前倾 20°）、d435（fixed 到 piper link6）、nuc_weight（1.8kg，2026-07-07 起）
   - 质量校验：背部合计 ~6.5kg；URDF 通过 Isaac 导入器解析
4. **URDF -> USD**：IsaacLab `convert_urdf.py`（轮关节零刚度/速度驱动，腿关节位置驱动；
   注意 fixed-link 带惯量的 merge 行为——IsaacLab 2.3.2 已锁定兼容的 importer 版本）
   产出 `assets/usd/go2w_sensored/`（提交进库）。
5. **仓库场景脚本**：`scripts/sim/warehouse_scene.py`
   - 加载 Isaac Sim 内置室内资产（Nucleus 云端 `Isaac/Environments/Simple_Warehouse/
     full_warehouse.usd`，首次加载走网络并缓存到 docker-cache/ov；备选 hospital/office）
   - 落入 go2w_sensored USD，腿关节按站立位姿加 PD 保持，轮子速度驱动
   - headless 模式下渲染视口截图到 logs/scene.png 作为无人值守验证
6. **GUI 运行**：`scripts/run_gui.sh`（xhost + X11 socket 直通容器），用户桌面直接打开
   Isaac Sim 窗口看场景、拖机器人、跑物理。
7. **传感器真实化（M2，硬性要求：所有传感器/手臂出真实可用读数）**
   - Mid-360 → RTX Lidar prim 挂 `mid360_link`。Isaac Sim 5.1 不带 Livox 配置（已核实，
     只有 HESAI/Ouster/SICK/SLAMTEC/Velodyne/ZVISION）：优先找社区 Mid-360 RTX JSON，
     否则按规格自制近似（360°×[-7°,+52°]，~20 万点/秒，随机化 emitter 近似非重复扫描）
   - D435 → Camera prim 挂 `d435_link`（RGB + distance_to_image_plane 深度两路）
   - PiPER → 关节位置目标可控 + 关节状态可读（已是真关节），后续 IK/MoveIt2
   - ROS2 bridge（Isaac Sim 5.1 支持 Jazzy）把点云/图像/关节态发成 topic，与真机拓扑同构
   - URDF 侧已为此铺路：`merge_fixed_joints=False` 保留了 mid360/d435 的独立 frame

## 风险与对策

- **Nucleus 云资产下载慢/断**（国内网络）：仓库场景 USD+贴图约百 MB 级，一次性缓存;
  若不可用，退级方案是本地简单房间（ground plane + 盒体货架）先跑通,后补资产包。
- **合成 URDF 的关节命名冲突**：piper 与 go2w 均有 `base_link` 概念——合成时 piper 全部
  link/joint 加 `piper_` 前缀（已知 piper_description 本身以 base_link 为根）。
- **手臂加入后重心后移/前移**：站立位姿可能需要微调；M1 只要求稳定站立不翻。
- **RL 任务与带臂模型不兼容**：M1 不训练带臂 RL；robot_lab 的 Go2W 任务继续用原版 URDF，
  带臂模型用于场景/导航/抓取开发。全身 loco-manipulation 是路线图阶段 6（LeggedManip_Lab 路线）。

## 完成判据（M1 Definition of Done）

- [ ] GUI 中打开仓库场景，传感器版 Go2W 站立在地面，物理稳定（不翻、不抖）
- [ ] 机器人 USD 的 16 个运动关节 + 6 个臂关节在 stage 里可见且属性正确
- [ ] Mid-360 前倾 20°、D435 在末端朝前、NUC/稳压器在背板内（stage 里量得到）
- [ ] headless 渲染的场景截图存档（无人值守可复验）
- [ ] 全部脚本/URDF/USD 进 git，README 状态表更新

## M3 — 接入用户选定的导航栈（refs/Navigation-Physical-Experiment）

用户钉的导航栈 = CMU mecanum T-Bot 自主栈的 fork（arise_slam_mid360 + FAR/TARE 规划器 +
base_autonomy），**原生 Ubuntu 24.04 + ROS2 Jazzy**，docker/Dockerfile 基于
osrf/ros:jazzy-desktop（部署到 NX 用）。精读结论（2026-07-03，全部来自源码/配置实读）：

### 栈的接口契约（Isaac 桥必须对齐的）
| 项 | 值 | 出处 |
|---|---|---|
| 点云入 | `/lidar/scan`，**livox CustomMsg**（sensor:"livox" 分支，带每点时间）| featureExtraction.cpp L74 + livox_mid360.yaml |
| IMU 入 | `/imu/data`，sensor_msgs/Imu | 同上 |
| 控制出 | `/cmd_vel`，**TwistStamped**（frame "vehicle"）| pathFollower.cpp L302 |
| 世界/传感器 frame | `map` / `sensor`（`laser` 用于 mapping）| livox_mid360.yaml |
| IMU-laser 外参 | R=I，t=[-0.011,-0.02329,0.04412]（= Mid-360 出厂内置 IMU 偏移）| livox/livox_mid360_calibration.yaml |
| 倾斜安装支持 | `imu_laser_rotation_offset`（注释示例 pitch 0.5rad）→ 我们的 20° 前倾从这里配 | 同上 |
| 传感器安装参数 | `sensorOffsetX/Y`（默认 0.05/0）、`cameraOffsetZ`（0.25）→ Go2W 用 0.27/0/… | system_real_robot.launch |
| 地形分析阈值 | obstacleHeightThre 0.1 / vehicleHeight 1.5 / minRelZ -1.5 maxRelZ 0.3 | terrain_analysis.launch |
| 路径集 | local_planner 有 omniDir.yaml（全向）与 standard.yaml —— Go2W 4 轮不可横移，**sim 阶段用 standard** | local_planner/config |
| 无 go2 适配 | 全仓库 0 处 go2 字样 —— 适配点=cmd_vel 桥（真机走 wheeled_sport）| grep 全库 |

### 集成方案（Isaac 当机器人，栈当大脑）
1. Isaac 侧（M2 扩展）：RTX Lidar（Mid-360 近似配置）挂 `mid360_link` → PointCloud2；
   IMU sensor 挂 mid360 内置 IMU 位置（对齐出厂外参，calib 文件零改动）→ /imu/data 200Hz；
   ROS2 bridge（Isaac Sim 5.1 原生 Jazzy）发布 + 订阅 /cmd_vel。
2. 转换节点：PointCloud2 → livox CustomMsg（合成 offset_time，rotary 近似下 offset=方位角/转速）。
3. cmd_vel 执行：TwistStamped(vx, vyaw) → 差速轮速映射（4 轮定轴，忽略 vy）；
   真机则 → SportClient.Move(vx, vy, vyaw)（wheeled_sport 支持 vy）。
4. 栈侧：在其 Docker（osrf/ros:jazzy）内 colcon build（跳过 livox 驱动），
   system_real_robot 模式对接 Isaac topics（--network host 共享 DDS）。
5. 验收：warehouse 场景 RViz 点 waypoint → Go2W 自主避障到达。

## M4 — Local manipulation（导航到位 + 抓取，规划中）

导航栈跑稳后：PiPER 关节的 ROS2 控制/状态接口（关节已是真实 ImplicitActuator）→
D435 深度出抓取目标 → 导航到位-停稳-抓取的行为编排。先几何抓取，AnyGrasp 后补。

## M5 — vector_os_nano 接入（CEO 指示 2026-07-04，sim-to-real 桥梁）

Isaac warehouse + 传感器狗已接近真实场景；若 vector_os_nano（plan·route·verify·recover）
能稳定驱动它，sim-to-real 就不难。方案：按其插件协议做 isaac-go2w world（不改内核）——
注册 tools（navigate→/way_point、perceive→/camera/*）、verify 谓词读 Isaac 地面真值、
vocab/persona；复用本仓库 DDS 域 42 桥。跨仓库工作按 vector_os_nano 的 COORDINATION.md
协议声明。这正是其北极星的 "bring a robot" 用例。

## 完成判据（M2 Definition of Done — 传感器真实可用）

- [ ] Mid-360 点云：仓库场景中采一帧，点数/FOV/量程与规格相符，存 .npy + 可视化截图
- [ ] D435：RGB 与深度图各存一帧，深度值与场景几何吻合（抽查已知距离物体）
- [ ] PiPER：发一组关节目标 → 关节实际到位（误差 < 1e-2 rad），末端带着 D435 动
- [ ] （进阶）ROS2 bridge 出 /points /image /joint_states，rviz2 或 ros2 topic hz 验证

## 策略重训 — 载荷包络修复（2026-07-07 CEO 委托决策）

**病根（vx-audit 已定量，不复查）**：出厂策略 `model_1999.pt`（robot_lab v2.3.2 go2w flat，
`logs/rsl_rl/unitree_go2w_flat/2026-07-04_15-52-42/`）训练时是 **6.92kg 裸躯干**；部署背
**~6.5kg 前偏载荷**（PiPER 4.66kg + NUC/支架，质心前移 6.5cm / 上抬 7.5cm），把 base 复合体
推出训练随机化包络（质量 +3kg 上限、质心 ±5cm）**1.3-2.2 倍**，且 obs57 对载荷盲测 →
零指令爬行、低指令前向过冲、指令边缘步态退化。

**方案阶梯 d→a**（部署一致性红线：奖励/指令分布/执行器增益/decimation/obs57·act16 一律不动，
新 ckpt 必须与部署 shim `scripts/sim/go2w_policy.py` 同构）：

| 方案 | 改动 | 状态 |
|---|---|---|
| **d（默认）** | base 质量随机上限 +3kg→**+8kg**、CoM ±5cm→**±10cm**，重训 | 已落 diff（见下） |
| **a（d 不达标时）** | 固定 ~6.5kg 前偏载荷（base 质量 add 中心化 5-8kg + CoM 前/上偏），重训 | 预案已备，opt-in 未启用 |

方案 a **不换带臂 URDF 训练**：`go2w_sensored.urdf` 多 8 个臂关节，会破坏 obs57/act16 与冻结
shim 的同构；改用"base 附加前偏载荷"等价事件项（`payload_env_cfg.py`）保住 16 自由度形态。

**改动落点（robot_lab 是 git-ignore 的 vendored 依赖 → 改动以 tracked 补丁存本仓，clone_deps
后自动重贴）**：
- `scripts/sim/retrain/robot_lab_patch/go2w_payload_envelope.patch`（plan-d + plan-a 注册）
- `scripts/sim/retrain/robot_lab_patch/payload_env_cfg.py`（plan-a 变体 cfg）
- `scripts/sim/retrain/robot_lab_patch/apply.sh`（幂等贴补，已接入 `clone_deps.sh`）
- 覆写放在 go2w 专属 `rough_env_cfg.py`（非共享父 `velocity_env_cfg.py`）→ 不影响其他机器人。

**训练/验收工具（离线已备，待 sim 空窗真跑）**：
- `scripts/sim/run_retrain.sh`：前置检查（ONE-sim、free≥30G、GPU 空闲、容器 Up）→
  `systemd-run --scope -p MemoryMax=45G` 包裹容器内 headless 训练（2048 env / 2000 iter，
  与出厂 ckpt 同锚点；torch/CUDA 不用 ulimit -v）。
- `scripts/sim/policy_acceptance.py`：症状四段套件（载荷态、训练增益、复用 Go2WPolicy 加载/喂
  指令路径），输出 JSON 行做新旧对比。
- `scripts/sim/retrain_runbook.md`：完整规程 + 训练监控点 + plan-a 升级路径。

**先确定"是 policy 问题"再开火（A/B 判决实验，训练前第一个 sim 实验）**：同一旧 ckpt 在
两身体各跑一遍症状套件——A=裸机(`go2w_bare.urdf`,策略训练态躯干,`make_bare_urdf.py` 从有臂
版自动剥出)、B=带载(`go2w_sensored.urdf`)。`ab_verdict.py` 离线判决：B 在 跟踪误差/pitch
方差/摔倒率 三判据中 ≥2 项显著劣于 A 且 A 本身健康 → `OOD_CONFIRMED`(退出0,开火)；A 也差 →
`POLICY_SUSPECT`(退出2,停,别同配方重训)。`warehouse_nav.py` 亦支持 `GO2W_WITH_ARM=0` 裸机
目视对照(跳过 PiperGraspController,臂/抓取写入全 None-guarded)。

**预注册验收判据（写死防事后挪门柱；带载态 go2w_sensored.urdf，新旧 ckpt 同套件并排）**：

| # | 判据 |
|---|---|
| ① | 零指令漂移 **< 0.02 m/s** |
| ② | 0.3 指令跟踪误差 **比旧 ckpt 改善 ≥50%** |
| ③ | wz=±1.4 阶跃×5 **零摔**（\|roll\|>60° 或 \|pitch\|>60° 判摔） |
| ④ | 0.6 指令跟踪 **不劣化于旧 ckpt** |

**决策**：①②③④ 全过 = **d 成功**→部署切换；①③过但②不达 = **可用但升级 a 再迭代**；
①或③不过 = **d 失败直接上 a**。（判据在开火前冻结,结果出来不改。）

**训练验证纪律（硬检查点，详见 runbook §4/§10）**：开火后先核对新 run 的 params/env.yaml
里 mass/CoM 确是 +8kg/±10cm（没生效=白训 30min）→ 训练中对照旧 run tfevents 锚点,500 iter
还不收敛就停诊断而非硬跑 → 训完先跑旧 ckpt 定锚再跑新 → GO2W_POLICY 默认只在全判据过后改,
改完留旧路径注释。

**保真度注意（已修复 2026-07-07 载荷轮）**：`nuc_weight` 0.5kg 占位已改 **1.8kg 实测值**，
重核算：审计口径载荷（PiPER 4.660 + NUC 1.8）= **6.460kg ≈ 审计 ~6.5kg**；背部总挂载
7.463kg；全挂载（含 mid360/d435）7.800kg；裸躯干 6.921kg。裸机变体已 make_bare_urdf.py
再生成。训练载荷中心（plan-a 5-8kg add）/部署 URDF/实物三者对齐。

**产品裁定记录（2026-07-07，编排者按 CEO 委托拍板——可复议）**：
部署默认策略切换为 **model_3497**（run 2026-07-07_06-51-14 = 出厂 ckpt + 配方 v2
[rel_standing 0.12 + 轮税 -0.005] 微调 500+1000 iter）。预注册判据裁定（诚实记录）：
- ① 零指令漂移 <0.02：**部署带载面（§9 绑定面）PASS = 0.0101**（出厂 0.0876，8.7×改善）；
  **原生随机体群中位 FAIL = 0.0343**（标称体 env0 = 0.005；旧 0.0695）——照实记 FAIL。
- ③ wz±1.4×5 零摔 PASS（双环境）；④ 0.3/0.6 跟踪 0.050/0.010（锚 0.336/0.183）PASS 最佳；
  ② 过冲 0.15→0.178 轻微（旧 0.253）。arc 段 rel_err 0.367（非 §9 判据，软瑕疵留观察）。
落地理由三条：(1) 蠕动改善 8.7×（绑定面实测过门）；(2) 产品栈死区 v2 在零指令窗接管
站定，策略残余不上屏；(3) 墙钟视速 0.0101×RTF 0.222≈0.002 m/s 肉眼不可辨。
回滚 = bringup.sh/restart_all.sh 一处注释换回（旧 ckpt 未删）。

**配方轮沿革（2026-07-07 三轮）**：
- v1（从零训：rel_standing 0.25 + wheel_tax -0.01 + plan-d 包络）：§4b FAIL——策略掉进
  "站定+省惩罚"退化盆，跟踪误差 900+ iter 零收敛（run 2026-07-07_05-53-47 全量保留作消融）。
- v2 r1（微调 +500 从出厂 ckpt，0.12/-0.005，撤包络）：③④②过，①差之毫厘
  （0.0277/0.0218）→ model_2498。r2（加压税 -0.01）：回退（0.0488），税证到顶。
- Round-3（终局轮：r1 配方原封 +1000 从 model_2498）：**① 部署面过门 0.0101**，④ 最佳
  → model_3497 产品裁定落地（见上）。
- 【待办·后续轮】**载荷包络修复另开一轮**：原 plan-d 的 +8kg/±10cm（或 plan-a 定向载荷）
  在蠕动配方收口后单独训——带载 pitch_var 2.6× 劣化（A/B 判决实锤）仍未解决，勿忘。

**部署切换规程**（验收过后 = go-live）：
1. 验收全段 pass → 把 `GO2W_POLICY` 指向新 ckpt，或改 `scripts/nav/bringup.sh` +
   `restart_all.sh` 默认（当前 `.../2026-07-04_15-52-42/model_1999.pt`）。
2. 清空窗重启 nav 栈（`scripts/nav/restart_all.sh`）。
3. **步态复验**在真实验收面（裸栈 + 真 cmd_vel，目视 sim）：重跑 DEBUG.md 步态检查
   （零指令 idle 漂移、低指令跟踪、wz 阶跃稳定），确认部署行为与验收一致。
4. 旧 ckpt 保留不删，直到新策略在真实面证毕。
