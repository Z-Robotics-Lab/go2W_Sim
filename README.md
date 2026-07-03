# go2W_Sim — Unitree Go2W 仿真与开发环境

Unitree Go2W（轮足四足 + 后续 PiPER 机械臂）的 sim-first 开发环境：
Isaac Sim 5.1（Docker）+ Isaac Lab v2.3.2 + robot_lab v2.3.2（自带 Go2W 训练任务）+ 官方 go2w URDF。

完整技术路线（选型依据、真机接入、SLAM/Nav2、VLN、抓取路线图）见 **[docs/go2w-dev-roadmap.md](docs/go2w-dev-roadmap.md)**。

## 版本锁定（经兼容表验证的组合，勿单独升级）

| 组件 | 版本 | 来源 |
|---|---|---|
| Isaac Sim | 5.1.0 | Docker 镜像 `nvcr.io/nvidia/isaac-sim:5.1.0` |
| Isaac Lab | v2.3.2 | github.com/isaac-sim/IsaacLab（clone_deps.sh 锁定）|
| robot_lab | v2.3.2 | github.com/fan-ziqi/robot_lab（Go2W Flat/Rough 任务）|
| torch / torchvision | 2.7.0+cu128 / 0.22.0+cu128 | 离线 wheel（fetch_torch_wheels.sh）|
| go2w URDF | unitree_ros master `robots/go2w_description` | 12 腿关节 + 4 轮关节（`*_foot_joint`）|

## 前置条件

- Ubuntu 24.04，NVIDIA 显卡（RTX 系列，驱动 >= 570），docker + nvidia-container-toolkit
- 本地已有镜像：`docker pull nvcr.io/nvidia/isaac-sim:5.1.0`（约 23GB）

## 快速开始

```bash
git clone https://github.com/Z-Robotics-Lab/go2W_Sim.git && cd go2W_Sim
bash scripts/clone_deps.sh          # 拉取 IsaacLab / robot_lab / go2w URDF 并锁版本
bash scripts/fetch_torch_wheels.sh  # 网络差时先离线下好 torch（断点续传，可跳过）
bash scripts/setup_container.sh     # 建 go2w-isaac 容器并完成全部安装
```

冒烟测试（headless 训练 Go2W 平地行走任务几个迭代）：

```bash
docker exec go2w-isaac bash -c "cd /workspace/go2w/robot_lab && TERM=xterm \
  /isaac-sim/python.sh scripts/reinforcement_learning/rsl_rl/train.py \
  --task RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0 --headless --num_envs 64 --max_iterations 5"
```

## 目录结构

```
scripts/            clone_deps / fetch_torch_wheels / setup_container
docs/               go2w-dev-roadmap.md（深度调研报告 + 分阶段路线图）
IsaacLab/           (gitignored) Isaac Lab v2.3.2
robot_lab/          (gitignored) robot_lab v2.3.2，Go2W 任务在此
assets/unitree_ros/ (gitignored) 官方 go2w_description URDF
assets/usd/         URDF 转换出的 USD（后续提交）
wheels/             (gitignored) 离线 torch wheel
docker-cache/       (gitignored) shader/扩展缓存，二次启动提速
```

## 已踩过的坑（scripts 已内置规避）

1. **pip 直装 isaacsim 不可行**：`pypi.nvidia.com` 的多 GB wheel 在不稳定网络下反复断流且 pip 不支持续传 —— 用 Docker 镜像替代。
2. **kit python 缺 setuptools**：容器内 Python 3.11 没有 `pkg_resources`，源码构建的依赖会挂。
3. **isaaclab.sh 的 pip 自升级自毁**：运行中升级 pip 会留下新旧混合的损坏 pip（`pip._vendor.packaging._structures` 缺失）——需删干净后用 get-pip 重装。
4. **TERM=dumb 会让 isaaclab.sh 立即退出**：容器内跑安装脚本要 `TERM=xterm`。
5. **1GB torch 用 wget -c 断点续传**：pip/aria2（阿里云对多连接回 403）都不行，单连接 wget + 重试循环最稳。
6. **轮关节命名**：URDF/USD 里是 `FL/FR/RL/RR_foot_joint`（continuous），MuJoCo 官方模型里却叫 `*_wheel_joint` —— sim2sim 需要显式映射。

## 状态

- [x] 容器 go2w-isaac（GPU 直通 / host 网络 / 40G 内存上限）
- [x] Isaac Lab v2.3.2 依赖链修复 + torch 2.7.0+cu128 离线就位
- [ ] Isaac Lab + robot_lab 安装收尾（进行中）
- [ ] go2w URDF -> USD 转换
- [ ] Go2W 平地任务 headless 冒烟测试
- [ ] GUI 运行脚本（X11）
