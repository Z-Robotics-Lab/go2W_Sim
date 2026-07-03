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

## 状态

- [x] 容器 + 全链安装（镜像已固化 go2w-isaac:ready）
- [x] 基线冒烟：Go2W 平地 RL 任务 5 迭代跑通（rsl_rl PPO，reward 正常）
- [x] 传感器版 URDF：真实网格、FK 校准 D435 位姿、FRAME 数值验证（Mid-360 19.6°）
- [x] GUI 平地场景人工目检通过
- [ ] 仓库（warehouse）场景导入（M1 收尾，进行中）
- [ ] M2：Mid-360 RTX 点云 / D435 深度图 / PiPER 关节控制真实读数
