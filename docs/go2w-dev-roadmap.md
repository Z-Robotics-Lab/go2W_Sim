# Unitree Go2W + PiPER 完整开发路线图

面向：机器人新手 · Ubuntu 24.04 + ROS2 Jazzy · Go2W（轮足）+ Jetson Orin Nano 扩展坞 + RealSense D435 + Livox Mid-360 + NUC（i7/16G）+ 松灵 PiPER 机械臂 + 工作站（RTX 5080）
目标：导航（LiDAR SLAM + Nav2）→ VLN → 抓取 / loco-manipulation
调研日期：2026-07-02。所有关键结论都经过实际抓取仓库/文档核实；标注 [推断] 的是未直接验证的推理；"待确认"一节列出所有没查到权威答案的问题。

---

## 0. 一页结论（先读这个）

1. **接入层弃用 grasp-lyrl/unitree_go2w_agent_sdk**（已核实：ROS1 Noetic + ROS2 Foxy + ros1_bridge 混合，正是你不喜欢的架构）。替代路线：**官方 unitree_sdk2 / unitree_sdk2_python（不依赖 ROS）+ 官方 unitree_ros2（纯 ROS2，走 CycloneDDS 直连）**。unitree_ros2 官方只测过 Foxy/Humble，但已有人在 Jazzy 上编译跑通真机 Go2 + Nav2（[Sayantani-Bhattacharya/unitree_go2_nav](https://github.com/Sayantani-Bhattacharya/unitree_go2_nav)），可行性有先例。
2. **Go2W 不是 Go2，照抄 Go2 教程会踩坑**：Go2W 的运动服务叫 `wheeled_sport`（模式名 `ai-w`），Go2 的叫 `mcf`/`sport_mode`——按名字关服务的教程代码在 Go2W 上会关错；但高层 `SportClient.Move(vx,vy,vyaw)` 的 API 完全一致（轮子由机器人自己处理），低层也是同一套 `rt/lowcmd`（20 个电机槽位里 12 腿 + 4 轮共 16 个在用）。
3. **Jazzy 兼容性矩阵参差**（详见 §3）：livox_ros_driver2、Nav2、STVL、realsense-ros 原生支持 Jazzy；FAST-LIO2 的 ROS2 移植和 piper_ros 只有 Humble 声明，需要自己编译/小改（都有成功先例）；完全没有"开箱即用的 Go2W + Mid-360 + Nav2 + Jazzy"现成仓库——集成这一层就是你要做的工程。
4. **VLN 的大模型必须离板跑**：NaVILA 8B 权重 FP16 就有 16.1GB、官方评测要求 ≥24GB 显存；VLFM 作者实机用 16GB 显存的 4090。你的 RTX 5080 是 16GB：VLFM 可行，NaVILA 全精度装不下（量化可能可行但无官方路径，[推断]）。Orin Nano 8G 和无独显 NUC 都不可能承载 VLN 大模型——它们只跑 SLAM/Nav2/检测/驱动。
5. **供电有两个硬性不兼容**：Go2W 背部 XT30 输出的是电池直出电压 24–33.6V（标称 28.8V），而 **PiPER 上限 26V、Mid-360 上限 27V——都不能直接接**，各需一个 DC-DC 降压模块（PiPER 建议 24V/≥240W，Mid-360 用 12V）。只有 Jetson 扩展坞（16–60V 输入）能吃电池直出。XT30 口的功率上限没有任何公开文档，装 NUC + 机械臂前**必须先问 Unitree 售后确认**。
6. **仿真侧好消息**：[fan-ziqi/robot_lab](https://github.com/fan-ziqi/robot_lab) 自带 Go2W 的 Isaac Lab 训练环境（Flat/Rough velocity 任务，支持 Isaac Sim 5.1）；官方 Go2W USD 在 HuggingFace [unitreerobotics/unitree_model](https://huggingface.co/datasets/unitreerobotics/unitree_model)；官方 [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) 也有 go2w 场景。你的 Isaac Sim 5.1 配 Isaac Lab 2.3.x 即可。
7. **机械臂**：piper_ros 官方无 Jazzy 分支（humble 分支 + 小配置修复可在 Jazzy 跑，issue #17 有先例）；更稳的起步是**不走 ROS 的 piper_sdk（纯 Python + SocketCAN）**。抓取推荐 AnyGrasp（唯一活跃维护，2026-06 还在更新，有 Jazzy ROS2 wrapper）但需要申请 license；免费替代是 graspnet-baseline（冻结的研究代码）。
8. **LeggedManip_Lab 评估**：它不是点足——是 Unitree 底座 + 机械臂的全身 RL 框架，**直接内置 GO2-PIPER 的 WBC 任务**（Isaac Sim 5.1），reward/curriculum/MuJoCo 资产都可以为你的阶段 6 复用；但没有 Go2W 轮足模型，sim-to-real 标注 "Coming Soon" 未发布。留作最后阶段的研究参考，不是近期依赖。

---

## 1. 硬件布局、算力分工、网络拓扑

### 1.1 算力分工（推荐）

| 设备 | 跑什么 | 理由 |
|---|---|---|
| 机器人本体 MCU/内置机 | 运动控制（wheeled_sport 服务）| 出厂即有，不要动它；你只发 `Move(vx,vy,vyaw)` 或低层命令 |
| **NUC i7/16G** | livox 驱动 + FAST-LIO2（纯 CPU）+ Nav2 + MoveIt2 + piper 驱动（USB-CAN 插 NUC）| SLAM/导航是 CPU+内存大户，16G > Orin 的 8G；CAN 和 Mid-360 网口都接它，数据不过无线 |
| **Orin Nano 8G**（扩展坞）| D435 驱动 + TensorRT 检测（YOLO11n ~37FPS 已核实）+ unitree 接口桥 | 它的价值是 GPU；8G 统一内存跑不动 SLAM+Nav2+检测三合一（[推断]，无公开基准） |
| **工作站 RTX 5080（16G）** | VLN/VLM 推理、AnyGrasp 抓取推理、Isaac Sim/Isaac Lab 训练 | 所有大模型都在这；经 Wi-Fi/以太网桥入机器人内网 |

### 1.2 网络拓扑（官方权威约束）

```
                    [工作站 RTX 5080]
                          | Wi-Fi 或长网线
[Go2W 本体 192.168.123.161 (固定, 严禁占用)]
   |  机身 RJ45
[小型千兆交换机(背部)]--- [NUC 192.168.123.99]---USB-CAN---[PiPER]
   |                        |  第二网口/USB网卡 192.168.1.50
[Orin 扩展坞 192.168.123.x] [Livox Mid-360 192.168.1.1XX (默认网段!)]
   (坞上有 2x RJ45)            XX = 序列号后两位
```

- 本体内置机固定 `192.168.123.161`，官方 Quick Start 明文禁止外接设备占用该地址；外接设备静态配到 `192.168.123.x`（官方示例 .222 / .99）。[已核实: [Unitree Quick Start](https://support.unitree.com/home/en/developer/Quick_start)]
- DDS 用 CycloneDDS 0.10.2，`CYCLONEDDS_URI` 里必须写真实网卡名（如 `enp3s0`）。[已核实: [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)]
- **Mid-360 默认在 192.168.1.x 网段**（不是 123 网段），最省事的接法是接 NUC 的第二个网口（或 USB 网卡）设 192.168.1.50，跟机器人内网物理隔开。[已核实: Livox Mid-360 用户手册]
- `.161` 的文档原文写的是 Go2，推广到 Go2W 是共享平台推断（合理，但无 Go2W 专门声明）。

### 1.3 供电（硬约束，装机前必读）

| 负载 | 输入要求 | 接法 | 依据 |
|---|---|---|---|
| Jetson 扩展坞 | 16–60V DC | XT30 直接供，官方设计 | [已核实: docs.quadruped.de 规格页] |
| PiPER | **DC 24V（上限 26V）**，峰值 ≤120W，建议电源 ≥10A | **必须加 24V DC-DC 降压（建议 ≥240W）**，禁止直连 | [已核实: [AgileX 官方手册 PDF](https://static.generation-robots.com/media/agilex-piper-user-manual.pdf)] |
| Mid-360 | 9–27V（推荐 12V），常态 6.5W，冷启动峰值 18W | **必须加 12V DC-DC**，禁止直连 | [已核实: Livox 官方手册] |
| NUC | 视型号（多为 19V）| 19V DC-DC 或 NUC 自带适配范围 | 查你的 NUC 铭牌 |
| D435 | USB 供电（~3.5W）| 插 Orin 或 NUC 的 USB3 | — |

- Go2W 背部电源口：XT30 连接器、电池直出 28.8V 标称（24–33.6V 随电量浮动）。[已核实: docs.quadruped.de + k-robotic；XT30 具体针脚来自搜索快照，中等置信]
- **XT30 口的电流/功率上限：任何公开文档都没有**。你的满配峰值估算 ≈ Orin 25W + NUC 40–90W + PiPER 120W + Mid-360 18W ≈ **200–250W 峰值**——先发邮件问 Unitree 售后这个口能不能带，不行就给机械臂单独配电池。这是全案最大的未知数。
- 载重核算：PiPER 4.2kg + 夹爪 0.5 + NUC ~0.8 + Mid-360 0.27 + D435 0.07 + 支架/降压板 ~1.2 ≈ **7 kg**，贴着 Go2W 的 ≈8kg 持续载重上限（极限 12kg）。[已核实: unitree.com/go2-w 规格] 尽量轻量化安装板，别再加东西。

---

## 2. 软件栈选型（每层的推荐 + 落选理由）

| 层 | 推荐 | 状态 | 落选/备选 |
|---|---|---|---|
| 本体 SDK | [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)（C++，自带 `example/go2w/`）+ [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)（有 go2w 高低层示例）| 官方、活跃 | — |
| ROS2 桥 | [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)（纯 ROS2，DDS 直连）| 官方测到 Humble；**Jazzy 需自编译**，有真机先例 | [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk)：WebRTC 为主、关节态仅 1Hz、无 Jazzy/Go2W——只作参考 |
| LiDAR 驱动 | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) `./build.sh jazzy` | **原生 Jazzy**（2026-04 落地）| — |
| LIO SLAM | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)（自带 `config/mid360.yaml`）| 声明到 Humble，**Jazzy 需自编译**（PCL/Eigen 可能要调）| Point-LIO/GLIM（架构参考：[mil-as/quadruped_navigation_ros2](https://github.com/mil-as/quadruped_navigation_ros2) 用 GLIM+Nav2，但平台是 Spot+Humble）|
| 3D 点云入 costmap | [STVL](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/jazzy) `apt install ros-jazzy-spatio-temporal-voxel-layer` | **Jazzy 二进制已发布**，PointCloud2 直入，免转 laserscan | pointcloud_to_laserscan（更简单的兜底）|
| 导航 | Nav2（Jazzy apt 二进制）| 原生 | — |
| 相机 | [realsense-ros](https://github.com/IntelRealSense/realsense-ros) | README 列 Jazzy；**24.04 内核 6.8 的 DKMS 有坑**（见 §6）| — |
| 机械臂 | 先 [piper_sdk](https://github.com/agilexrobotics/piper_sdk)（纯 Python，无 ROS）；后 [piper_ros humble 分支](https://github.com/agilexrobotics/piper_ros/tree/humble)（含 MoveIt2 配置）移植到 Jazzy | 官方无 Jazzy 分支（issue #46 无回复）；issue #17 证明 humble 分支在 Jazzy 上小改 `joint_limits.yaml` 即可跑 | — |
| 抓取 | [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk)（需申请 license，机器绑定）+ 社区 Jazzy wrapper [anygrasp_ros](https://github.com/CollaborativeRoboticsLab/anygrasp_ros) | 唯一活跃维护（2026-06 更新，支持 CUDA 12.8/Python 3.13）| [graspnet-baseline](https://github.com/graspnet/graspnet-baseline)（免费但冻结，PyTorch 1.6 时代）；contact_graspnet（TF2.2，弃） |
| VLN | 起步：VLM API 规划器 + Nav2（自建，见阶段4）；进阶：[VLFM](https://github.com/bdaiinstitute/vlfm)（16G 显存可行）；研究级：[NaVILA](https://github.com/AnjieCheng/NaVILA) | VLFM/NaVILA 都**没有任何 ROS2 代码**，需自建桥接 | — |
| 仿真 | Isaac Sim 5.1 + Isaac Lab 2.3.x + [robot_lab](https://github.com/fan-ziqi/robot_lab)（Go2W 任务）；轻量备选 [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)（go2w 场景）| robot_lab 明确支持 Isaac Sim 5.1 | menagerie 无 go2w；Isaac Lab 官方无 Go2W 资产（已核实 0 命中）|

### Go2W 专属 API 事实（全部经源码核实）

- 运动服务：`wheeled_sport`（模式 `ai-w`）。进低层前用**名字无关**的 `MotionSwitcherClient.ReleaseMode()` 停掉运动服务，不要照抄 Go2 教程的 `ServiceSwitch("sport_mode",0)`。
- 高层：`SportClient.Move(vx, vy, vyaw)` / `StandUp` / `BalanceStand` 等与 Go2 完全同 API（`example/go2w/go2w_sport_client.cpp`），轮子对你透明——**这意味着 Nav2 输出 cmd_vel 后只需要一个 ~100 行的桥接节点**。
- 低层：`rt/lowcmd` / `rt/lowstate`，unitree_go IDL，`motor_cmd[20]` 中 12 腿 + 4 轮（轮子索引官方示例未演示；社区仓库 [jj7258/unitree_go2w_ros2](https://github.com/jj7258/unitree_go2w_ros2) 演示了 16 电机控制，Humble，10 天一次性代码，仅参考）。
- URDF（[unitree_ros go2w_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2w_description)）：12 个 revolute 腿关节 + 4 个 continuous 轮关节，轮关节名是 `FL/FR/RL/RR_foot_joint`（不叫 wheel！）；**MuJoCo 版却叫 `*_wheel_joint`**——做 sim2sim 必须做关节名映射。

---

## 3. 分阶段路线图

### 阶段 0 — 环境搭建（工作站，1 周）
1. 工作站装 ROS2 Jazzy desktop（apt 官方源），过一遍 [官方入门教程](https://docs.ros.org/en/jazzy/Tutorials.html) 的 CLI + topic/service + colcon 部分——这是后面一切的语言。
2. 建工作区：`mkdir -p ~/go2w_ws/src && cd ~/go2w_ws && colcon build`。以后所有源码包都 clone 进 `src/`。
3. Isaac Sim 5.1（你已有 Docker）+ Isaac Lab **2.3.x**（与 5.1 官配；3.0 还是 beta）。
4. 建一个 GitHub 私有仓库存你的所有配置/launch/桥接代码——部署到 NUC/Orin 全靠 git（见 §4）。

**出口标准**：工作站上 `ros2 topic pub/echo` 玩转；Isaac Lab 跑通自带 Go2 例子。

### 阶段 1 — 仿真先行（1–2 周）
1. **MuJoCo 5 分钟版**：clone unitree_mujoco，跑 `unitree_robots/go2w/scene.xml`——最快看到 Go2W 动起来的方式，且它直接对接 unitree_sdk2 的接口（低层 sim2real 同构）。
2. **Isaac Lab 版**：装 robot_lab，跑 `RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0`（rsl_rl PPO，5080 上训练没问题）。这一步让你理解轮足 locomotion 是怎么被速度指令驱动的——即便你实机用官方 wheeled_sport 不用自训策略。
3. 官方 Go2W USD 在 HF `unitreerobotics/unitree_model` 的 `Go2W/usd/go2w.usd`（注意 GitHub 镜像没有 Go2W，要用 HF）。
4. 学 Nav2 本身建议用官方 TurtleBot 仿真教程（比在四足上直接学快得多）——Nav2 的概念（costmap/planner/controller/BT）与载具无关。

**出口标准**：Isaac Lab 里 Go2W 平地走起来；理解 cmd_vel → 轮足运动的链路。
**明确不做**：不要在 Isaac Sim 里搭完整 nav 仿真链（现有 go2 nav 仿真仓库都锁 Humble，投入产出比低；导航直接上实机更快）。

### 阶段 2 — 实机接入（1–2 周，先不装任何背部设备）
1. 网线连 Go2W，本机静态 IP 192.168.123.222，`ping 192.168.123.161`。
2. 装 unitree_sdk2_python，跑 `example/go2w/high_level/go2w_sport_client.py`——手柄旁站好，让它 StandUp / Move / StopMove。**这是你的第一个真机 demo。**
3. 在 Jazzy 上源码编译 unitree_ros2（先例证明可行；若编不过，fallback 是在 NUC 上开一个 Humble Docker 容器专门跑接入层，DDS 消息跨容器互通）。
4. 写你的第一个自有节点：**cmd_vel 桥**（订阅 `/cmd_vel` → 限幅/限加速度 → `SportClient.Move()`；带 watchdog：0.5s 没新消息就发 StopMove）。这 ~100 行是全项目的地基。
5. 把 Orin 坞和 NUC 装上、配好静态 IP，确认三台机器互相 `ros2 topic list` 可见（同一 `ROS_DOMAIN_ID`，CycloneDDS 指定网卡）。

**出口标准**：NUC 上 `teleop_twist_keyboard` 能开着 Go2W 走。

### 阶段 3 — SLAM + Nav2（3–6 周，核心工程阶段）
1. Mid-360 上机（接 NUC 第二网口），livox_ros_driver2 jazzy 出点云。
2. FAST_LIO_ROS2 在 Jazzy 上编译（记得 launch 传 `config_file:=mid360.yaml`，默认是 avia！）。得到 `odom` 和去畸变点云。
3. TF 树：`map → odom → base_link`（FAST-LIO 出 odom；先用它的位姿直接当定位，后期再考虑重定位）。量好 Mid-360 相对 base_link 的安装外参写进静态 TF。
4. Nav2：STVL 层吃点云进 costmap；controller 用 DWB 或 MPPI；机器人当作全向底盘调参（Go2W 的 Move 支持 vx/vy/vyaw）。cmd_vel 桥已在阶段 2 就绪。
5. 参考架构（不可直接复用但布线图值得抄）：[mil-as/quadruped_navigation_ros2](https://github.com/mil-as/quadruped_navigation_ros2)（Spot + Mid-360 + GLIM + Nav2）、[jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)（Go2 + Point-LIO + CMU 自主栈，非 Nav2 但质量最高）、[Sayantani-Bhattacharya/unitree_go2_nav](https://github.com/Sayantani-Bhattacharya/unitree_go2_nav)（唯一 Jazzy+Nav2 真机先例）。

**出口标准**：RViz 里点一个目标，Go2W 自主绕障到达。

### 阶段 4 — VLN（4–8 周，分三档，从可用到研究级）
- **档 A（先做，2 周内见效）**：自建"VLM 规划器"——D435/全景图 + YOLO 检测发到工作站/云端 VLM（如 API 调用），VLM 输出语义目标 → 你的节点转成 Nav2 goal。不是学术意义的 VLN，但产品意义上"说人话让狗去哪"立刻可用。[这是建议，非调研结论]
- **档 B（VLFM 复现）**：frontier + VLM 零样本目标导航，ICRA'24 已在 Spot 真机验证（同为四足，架构可迁移）。代码零 ROS 依赖（Habitat + Spot 专有接口），需要你把它的感知-决策环嫁接到你的 Nav2 栈上；模型组（GroundingDINO+BLIP-2+MobileSAM+YOLOv7）作者实机用 16G 显存——你的 5080 刚好够。项目 2023 年后不维护，"as is"。
- **档 C（NaVILA，研究级）**：8B VLA 输出语言化动作 + 底层运动策略。三个现实障碍（全部已核实）：无任何 ROS 部署代码；评测基准 NaVILA-Bench 锁死 Isaac Lab 1.1.0 + Isaac Sim 4.1.0（与你的 5.1 不兼容，得单独维护旧环境）且只有 Go2/H1 无 Go2W；FP16 权重 16.1GB + 官方要求 ≥24G 显存，5080 的 16G 装不下全精度（INT4 量化理论可行但官方未提供，[推断]）。留作长线。

**出口标准（档 A/B）**："go to the nearest chair" 类指令端到端成功。

### 阶段 5 — 机械臂 + 抓取（4–6 周）
1. PiPER 先单独在桌上调：24V 电源（≥10A）、随臂 USB-CAN 插 NUC、`can_activate.sh can0 1000000`（内核驱动 gs_usb，24.04 开箱支持）、piper_sdk 跑关节/末端位姿控制 demo。**注意：只支持随臂附带的 CAN 模块，别买第三方。**
2. piper_ros humble 分支在 Jazzy 编译（照 issue #17 修 `joint_limits.yaml` 的 float 问题），起 MoveIt2 demo（`piper_moveit` 在仓库内）。
3. D435 手眼标定（eye-in-hand 或固定于背部均可，推荐先固定视角）。
4. 抓取推理管线（跑在 5080 工作站）：D435 点云（裁剪 ROI 后传输）→ AnyGrasp → 抓取位姿回传 → MoveIt2 规划执行。AnyGrasp license 要提前申请（Google 表单，~5 工作日，机器绑定；2026-06 起 license 工具在换代，申请前看最新 README）。
5. 参考小项目（都是学生/实验室级，无主导栈）：LangGrasp-Piper（AnyGrasp+YOLO-World+语音）、sam3_grasp（PiPER+D435i+SAM3）。LeRobot 生态有 PiPER 第三方插件（AgRoboticsResearch/lerobot_robot_piper）——模仿学习路线备用。

**出口标准**：桌面上指定物体，臂+夹爪自主抓起。

### 阶段 6 — loco-manipulation（研究阶段，时间开放）
- **工程路线（先做）**：Nav2 导航到物体附近 + 停稳 + 阶段 5 抓取管线 = "local manipulation"。这不需要全身控制，是纯编排问题（行为树/状态机），你阶段 3+5 的成果直接组合。
- **RL 全身控制路线（研究）**：[LeggedManip_Lab](https://github.com/zzzJie-Robot/LeggedManip_Lab) 内置 **GO2-PIPER WBC 任务**（Isaac Sim 5.1 + Isaac Lab main + rsl-rl ≥5.0.1，2026-06 新发布），末端位姿跟踪的 reward/curriculum/MuJoCo 资产全可复用；但没有 Go2W（轮关节需自己加，robot_lab 的 Go2W 配置可以嫁接）且 sim-to-real 未发布。把它当作你阶段 6 的研究起点，而不是可部署依赖。

---

## 4. 完全新手的操作手册

### 4.1 怎么连上机器人
```bash
# 1) 网线插 Go2W 机身 RJ45，你的电脑网口设静态 IP：
#    Settings → Network → IPv4 Manual: 192.168.123.222 / 255.255.255.0（别用 .161！）
ping 192.168.123.161        # 通 = 物理链路 OK

# 2) SSH 进扩展坞的 Orin（坞的 IP 见坞的说明书；不知道就扫内网）：
sudo apt install nmap && nmap -sn 192.168.123.0/24   # 列出内网所有设备
ssh unitree@192.168.123.<orin-ip>                     # 用户名/密码见随机文档
```

### 4.2 怎么部署代码（git 工作流，三台机器同一套代码）
```bash
# 工作站上开发 → push；NUC/Orin 上 pull → 编译。永远不要在机器人上直接改代码。
# 每台机器一次性配置：
git clone git@github.com:<你>/go2w_stack.git ~/go2w_ws/src/go2w_stack
cd ~/go2w_ws
rosdep install --from-paths src --ignore-src -r -y   # 自动装依赖
colcon build --symlink-install
echo "source ~/go2w_ws/install/setup.bash" >> ~/.bashrc

# 日常迭代：
git pull && colcon build --packages-select <改过的包> && ros2 launch ...
```

### 4.3 第一个 demo 序列（每步 <1 小时）
1. `ros2 topic echo /lowstate`（unitree_ros2 起来后）——看到电机/IMU 数据流 = 接入成功。
2. `python3 go2w_sport_client.py`（unitree_sdk2_python）——站起、原地小步移动。
3. `ros2 launch livox_ros_driver2 msg_MID360_launch.py` + RViz 看点云。
4. FAST-LIO 建图：抱着狗（或遥控慢走）绕房间一圈，RViz 看地图长出来。
5. Nav2 点目标 → 狗自己走过去。

### 4.4 多机 DDS 配置（三台机器互通的关键）
```bash
# 每台机器的 ~/.bashrc：
export ROS_DOMAIN_ID=1                    # 三台一致
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml   # xml 里 NetworkInterface 写各自网卡名
# 验证：A 机 ros2 topic pub /test std_msgs/String "data: hi"，B 机 echo 收到即通。
```

---

## 5. 已知坑清单（每条都有出处）

| # | 坑 | 后果 | 规避 |
|---|---|---|---|
| 1 | Go2 教程按名字关 `sport_mode` | Go2W 上关错服务，低层接管失败 | 用 `MotionSwitcherClient.ReleaseMode()` |
| 2 | URDF 轮关节叫 `*_foot_joint`，MuJoCo 叫 `*_wheel_joint` | sim2sim 关节映射静默错乱 | 显式关节名映射表 |
| 3 | unitree_ros2 / FAST_LIO_ROS2 / piper_ros 无 Jazzy 声明 | 编译报错劝退 | 都有移植先例；兜底 = NUC 上 Humble Docker 跑接入层 |
| 4 | FAST_LIO_ROS2 默认 config 是 avia.yaml | Mid-360 数据进来全是垃圾 | launch 传 `config_file:=mid360.yaml` |
| 5 | Mid-360 默认 192.168.1.x 网段 | 插机器人 123 交换机收不到数据 | 接 NUC 独立网口，主机设 192.168.1.50 |
| 6 | XT30 是电池直出 24–33.6V | **直连 PiPER（26V max）/Mid-360（27V max）烧设备** | 各配 DC-DC（24V/12V） |
| 7 | XT30 功率上限无文档 | 满载掉电/触发保护 | 先问 Unitree 售后；不行臂单独供电 |
| 8 | 24.04 内核 6.8 的 librealsense DKMS 不在官方支持列表 | D435 驱动装不上 | 用 RSUSB 后端（免内核补丁，损失少量元数据）或补丁脚本 |
| 9 | NaVILA-Bench 锁 Isaac Lab 1.1.0 + Isaac Sim 4.1.0 | 你的 5.1 环境直接跑不起 | 需要时单独建旧版环境；近期走 VLFM/档 A |
| 10 | AnyGrasp license 机器绑定（feature id 依赖网络身份）| 换机器/换 MAC license 失效 | 用固定 MAC 的 Docker（社区 wrapper 就是这么做的）|
| 11 | go2_ros2_sdk 的 WebRTC 路径关节态仅 1Hz、随固件升级破裂 | TF 卡顿、随时失联 | 走官方以太网 DDS，不走 WebRTC |
| 12 | Orin Nano 8G 统一内存 | SLAM+Nav2+检测三合一 OOM | 按 §1.1 分工，SLAM/Nav2 放 NUC |
| 13 | piper_ros 只认随臂 CAN 模块 | 第三方 USB-CAN 不工作 | 用原装模块（gs_usb 驱动，24.04 原生支持）|
| 14 | 载重贴 8kg 持续上限 | 续航/散热/运动性能下降 | 轻量化支架；能不装的不装 |

## 6. 待确认清单（调研没有找到权威答案的）
1. **XT30 扩展口的电流/功率预算**——问 Unitree 售后（全案最高优先）。
2. unitree_ros2 + CycloneDDS 0.10.x 在 24.04/Jazzy 的编译是否零障碍（有真机先例但非官方声明）——你的阶段 2 第 3 步会实测。
3. Go2W 4 个轮电机在 `motor_cmd[20]` 中的确切索引/语义（官方示例只演示了 12 腿）——高层 Move 用不到；做低层/RL 部署时再啃 jj7258 仓库 + 实测。
4. Orin Nano 能否稳定跑 "D435 + TensorRT 检测 + unitree 桥" 组合（无公开基准，[推断]可行）。
5. NaVILA 8B INT4 量化后在 16G 显存上的实际可用性（官方未提供路径）。

## 7. 主要来源
Unitree 官方：[opensource 页](https://www.unitree.com/cn/opensource) · [Quick Start](https://support.unitree.com/home/en/developer/Quick_start) · [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) · [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) · [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) · [unitree_ros/go2w_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2w_description) · [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) · [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) · [unitree_model (HF)](https://huggingface.co/datasets/unitreerobotics/unitree_model)
传感/导航：[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) · [Livox Mid-360 手册](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Livox_Mid-360_User_Manual_EN.pdf) · [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) · [STVL jazzy](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/jazzy) · [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
案例：[Sayantani-Bhattacharya/unitree_go2_nav](https://github.com/Sayantani-Bhattacharya/unitree_go2_nav) · [jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2) · [mil-as/quadruped_navigation_ros2](https://github.com/mil-as/quadruped_navigation_ros2) · [jj7258/unitree_go2w_ros2](https://github.com/jj7258/unitree_go2w_ros2)
机械臂/抓取：[piper_ros](https://github.com/agilexrobotics/piper_ros) · [piper_sdk](https://github.com/agilexrobotics/piper_sdk) · [AgileX PiPER 手册](https://static.generation-robots.com/media/agilex-piper-user-manual.pdf) · [anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk) · [anygrasp_ros (Jazzy)](https://github.com/CollaborativeRoboticsLab/anygrasp_ros) · [graspnet-baseline](https://github.com/graspnet/graspnet-baseline)
VLN：[NaVILA](https://github.com/AnjieCheng/NaVILA) · [NaVILA-Bench](https://github.com/yang-zj1026/NaVILA-Bench) · [VLFM](https://github.com/bdaiinstitute/vlfm)
仿真/RL：[robot_lab](https://github.com/fan-ziqi/robot_lab) · [LeggedManip_Lab](https://github.com/zzzJie-Robot/LeggedManip_Lab) · [Isaac Lab 2.3.0](https://github.com/isaac-sim/IsaacLab/releases/tag/v2.3.0)
评估过并不推荐：[grasp-lyrl/unitree_go2w_agent_sdk](https://github.com/grasp-lyrl/unitree_go2w_agent_sdk)（ROS1+ROS2 混合）· [abizovnuralem/go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk)（WebRTC/无 Jazzy/无 Go2W）
