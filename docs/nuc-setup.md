# NUC 配置指南（真机大脑）

> 状态：依据调研 + 仿真侧已验证架构写成，硬件到位后按"验收自检"逐条打钩并修订本页。
> 背景与选型依据见 [go2w-dev-roadmap.md](go2w-dev-roadmap.md)；桥接口合同见
> [agent-bridge-api.md](agent-bridge-api.md)（P5.2 落地后生效）。

## 0. NUC 在系统里的角色

NUC（i7 / 16G / Ubuntu 24.04）是真机部署的**大脑**，跑且只跑：

```
[你] ──说话──> z-agent CLI ──HTTP──> agent_bridge(:8042) ──DDS──> CMU 导航栈(docker)
                                                                    │  /cmd_vel
                                                                    v
                                              宇树桥(~150行, 待写) ──SDK──> Go2W wheeled_sport
传感器： Mid-360 ──网线──> NUC(第二网口) ──> 导航栈 SLAM
         D435   ──USB3──> NUC（抓取/VLN 阶段用）
         PiPER  ──USB-CAN──> NUC（抓取阶段用）
```

Isaac Sim **不在** NUC 上——它留在开发机（RTX 5080）当数字孪生/回归测试台。
仿真里验证的 z-agent、桥、导航栈三样，真机上是**同一份代码**。

## 1. 硬件准备清单

- [ ] NUC 本体 + 电源；机器人背部供电走**稳压器**（背板上已建模的那只，Go2W 背部供电口 → DC 稳压 → NUC 供电头；上机前先在台面用电源适配器完成全部软件配置）
- [ ] 网口规划（关键）：
  - **网口1（板载）** → Go2W 扩展坞以太网（宇树内网 `192.168.123.x`）
  - **网口2（USB3 千兆网卡）** → Livox Mid-360（雷达网段 `192.168.1.x`）
  - **WiFi** → 家里/实验室路由（开发 ssh + 拉代码；跑任务时可断）
- [ ] USB3 口预留：D435 一个、PiPER 的 USB-CAN 一个
- [ ] 宇树遥控器充好电——它是独立于软件栈的硬件急停，agent 跑动时**必须在手**

## 2. 系统安装（Ubuntu 24.04 LTS）

1. 官方 iso 装 Ubuntu 24.04 Desktop（要 RViz 本地显示；纯 ssh 运维也可 Server）
2. 用户名建议 `go2w`；勾选 OpenSSH
3. 装完立刻：
   ```bash
   sudo apt update && sudo apt install -y git curl openssh-server chrony net-tools
   ```
4. 关闭自动休眠/合盖挂起（跑任务时断电即失控）：
   ```bash
   sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
   ```

## 3. 网络配置（netplan 双静态网段）

`/etc/netplan/99-go2w.yaml`（网卡名以 `ip link` 实际输出为准）：

```yaml
network:
  version: 2
  ethernets:
    enp1s0:            # 网口1 → 宇树内网
      addresses: [192.168.123.99/24]
      dhcp4: false
    enx00e04c680001:   # 网口2(USB网卡) → Mid-360
      addresses: [192.168.1.5/24]
      dhcp4: false
```

```bash
sudo netplan apply
ping -c2 192.168.123.161   # Go2W 主控（宇树内网可达）
ping -c2 192.168.1.1XX     # Mid-360（XX=雷达 SN 后两位，见雷达标签）
```

时间同步（SLAM 对时间戳敏感）：chrony 用默认公网 NTP 即可；机器人内网若需要
给 Orin 授时，参照导航栈 fork 的 `chrony_conf/`。

## 4. Docker + 导航栈容器

```bash
# docker（无 GPU 依赖，导航栈是纯 CPU：gtsam/ceres SLAM + 地形分析 + TARE）
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER && newgrp docker

# 拉本仓库 + 导航栈源码 + 打补丁 + 构建镜像（与开发机完全同一套脚本）
git clone https://github.com/Z-Robotics-Lab/go2W_Sim.git ~/go2W_Sim
cd ~/go2W_Sim
git clone <Navigation-Physical-Experiment fork> refs/Navigation-Physical-Experiment
bash scripts/nav/patch_navstack.sh refs/Navigation-Physical-Experiment
# 按 refs/.../docker/README.md 构建 jazzy 镜像并 colcon build，完成后固化：
docker commit <容器> navstack:ready
```

真机与仿真的差异只在容器**启动参数/launch 选择**（真机：`system_real_robot*` 系 +
真 livox 驱动 + `use_sim_time=false`；不需要 `pc2_to_livox.py` 转换器）。桥
`agent_bridge.py` 原样复用。真机 bringup 清单落在 P5.4（见 §8 待定项）。

## 5. z-agent（产品运行时）

```bash
git clone https://github.com/Z-Robotics-Lab/z-agent.git ~/z-agent
cd ~/z-agent
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv .venv --python 3.12 && source .venv/bin/activate
uv pip install -e .          # 产品 tier：无 mujoco/无 torch，16G 内存无压力
cp .env.example .env         # 填一个 provider 的 API key
```

（F1 打包分层落地后，本节命令若有变化以 z-agent README 为准。）

## 6. 宇树侧（P5.4，依赖待 CEO 批准）

- `unitree_sdk2_python`（**新外部依赖，待批**）：宇树桥用它把导航栈的 `/cmd_vel`
  转成 `SportClient.Move(vx, vy, vyaw)`，交给 Go2W 出厂 `wheeled_sport` 步态。
- 宇树侧 DDS 用 CycloneDDS 且要指定网口（`CYCLONEDDS_URI` 写网口1的名字）；
  导航栈容器内部是 fastrtps+UDPv4——**两者隔离在各自进程里，不要试图统一**。
- 安全硬性三件套（桥内实现，上真机前必须齐）：
  1. 0.4s 无新 `/cmd_vel` → 自动 `StopMove()`（watchdog）
  2. 桥内二次限幅（vx≤0.6 m/s、vyaw 上限）+ NaN/inf 拒收
  3. 遥控器硬件接管优先级全程可用（不经软件栈）

## 7. 内存预算（16G）

| 进程 | 预算 |
|---|---|
| 导航栈容器（SLAM+规划+TARE+桥） | ≤ 8G（docker `--memory 8g`）|
| z-agent + CLI | ≤ 1G |
| RViz（可选，看图时开） | ≤ 1.5G |
| 系统 + 余量 | ~5G |

规则照旧：容器必须带 `--memory` 上限；任何新常驻进程先估内存再上。

## 8. 待定项（等三个 CEO 决策落地后修订本页）

1. **真机 verify 语义**：真机无 `/ground_truth/pose`，`go2w_at`/验收谓词读什么
   （推荐 SLAM 自洽 + 显式标注 + AprilTag 抽检）→ 定了改桥与谓词。
2. **算力拓扑**：本页按"方案 A：NUC 跑全栈"写成；若后续迁 Orin，本页拆出
   Orin 版。
3. **unitree_sdk2_python 依赖批准** → 定了补 §6 的具体安装与桥部署命令。

## 9. 验收自检（硬件到位后逐条打钩）

```bash
ip a | grep -E '192.168.(123.99|1.5)'      # [ ] 双网段静态 IP 就位
ping -c2 192.168.123.161                    # [ ] 宇树内网通
ping -c2 192.168.1.1XX                      # [ ] Mid-360 通
docker run --rm navstack:ready ros2 --help  # [ ] 容器就绪
cd ~/z-agent && .venv/bin/vector-cli --help # [ ] agent CLI 就绪
chronyc tracking                            # [ ] 时间同步正常
```

全部打钩后进入 P5.4 真机 bringup（桥 + 导航栈 + 首次站立联调，按当时的
bringup 手册执行）。
