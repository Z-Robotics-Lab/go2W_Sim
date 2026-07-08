# Go2W 使用手册（从零开始：只有这个仓库和硬件）

> 假设：你有一台电脑、这个仓库的地址、（真机路线）一只 Go2W 和一台 NUC——
> 其它什么都没装、什么都不懂。照本页从上往下做即可。
> 标注【P5.x/F2】的环节正在落地，当前替代做法都已写明。
> 两条路线：**仿真**（先学会和狗说话，零风险）→ **真机**（同一套话术上真狗）。

---

# 第 0 章 · 你需要什么

## 仿真路线（建议人人先走一遍）

| 项 | 要求 | 为什么 |
|---|---|---|
| 电脑 | Ubuntu 24.04 + NVIDIA RTX 显卡（本项目在 RTX 5080 上验证，建议显存 ≥12G）+ 内存 ≥32G + 磁盘空余 ≥120G | Isaac Sim 是重型物理仿真 |
| 网络 | 稳定外网，首次安装要下载 **几十 GB**（Isaac 镜像 + 仓库资产 + 模型） | 一次性成本 |
| LLM API 密钥 | 任选一家：DashScope(通义)/OpenRouter/Anthropic/OpenAI/DeepSeek | agent 的"大脑"按对话计费，个位数人民币/天的量级 |
| GitHub 账号 | 能 `git clone` 本组织仓库 | 拉代码 |

## 真机路线（在仿真路线全部跑通之后）

| 项 | 说明 |
|---|---|
| Unitree Go2W | 按宇树说明书完成首次开箱：充电、遥控器配对、App 激活、手动遥控走通 |
| NUC（i7/16G） | 按 [nuc-setup.md](nuc-setup.md) 从零配置（那页就是"NUC 的第 1 章"） |
| 配件 | USB 千兆网卡 ×1（接雷达）、网线 ×2、稳压器（背板供电） |

---

# 第 1 章 · 第一次安装（仿真机，一次性，约半天 + 下载时间）

以下全部在 Ubuntu 24.04 的终端里执行（`Ctrl+Alt+T` 打开终端；`$` 开头的行是命令，
复制粘贴回车即可；`# ...` 是注释不用输）。

## 1.1 基础工具

```bash
sudo apt update && sudo apt install -y git curl wget
```

## 1.2 NVIDIA 驱动（已装可跳过）

```bash
nvidia-smi          # 能打印显卡表格 = 已装好，跳过本节
sudo ubuntu-drivers install   # 自动装推荐驱动
sudo reboot         # 重启后再 nvidia-smi 验证
```

## 1.3 Docker + NVIDIA 容器支持

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER && newgrp docker
docker run --rm hello-world        # 验证：打印 "Hello from Docker!"

# NVIDIA Container Toolkit（让容器用得到显卡）
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
docker run --rm --gpus all ubuntu nvidia-smi   # 验证：容器里也能看到显卡
```

## 1.4 本仓库 + 资产 + Isaac 容器

```bash
cd ~ && git clone https://github.com/Z-Robotics-Lab/go2W_Sim.git go2w && cd go2w
bash scripts/clone_deps.sh          # IsaacLab/robot_lab/宇树与PiPER官方描述文件（含 robot_lab 重训补丁自动 apply）
bash scripts/fetch_wheels.sh        # 大体积 python wheel 断点续传预下载
bash scripts/fetch_sensor_meshes.sh # 真实传感器网格
python3 scripts/tools/compose_sensored_urdf.py   # 生成传感器版机器人 URDF

# —— Isaac Sim 基础镜像（NGC，需登录；~23GB，一次性）——
docker login nvcr.io                 # 用户名填 $oauthtoken，密码填 NGC API Key（下方说明）
docker pull nvcr.io/nvidia/isaac-sim:5.1.0   # setup_container.sh 的前置基础镜像

bash scripts/setup_container.sh     # 建 Isaac 容器 + 装全套（最耗时的一步，内置全部坑修复）
docker commit go2w-isaac go2w-isaac:ready        # 固化镜像，以后秒级重建
```

> **NGC API Key（拉 isaac-sim 基础镜像必需）**：去 ngc.nvidia.com 免费注册 → 生成 API
> Key → `docker login nvcr.io`（用户名固定填 `$oauthtoken`，密码填 Key）。这是 NVIDIA
> 官方镜像仓库，不登录 `docker pull nvcr.io/...` 会失败——这是新机搭建**唯一**需要外部
> 账号的一步。（本机无法替你验证 NGC 拉取，请自行完成。）
>
> **策略权重已入库**：RL locomotion ckpt 在 `assets/policies/`（git 追踪，clone 即有），
> 容器 bind-mount 仓库故容器内 `/workspace/go2w/assets/policies/...` 直接可见——`bringup.sh`
> 默认就指它，**无需训练即可拉起**。细节见 `assets/policies/README.md`。

验证这一步成功：

```bash
bash scripts/run_gui.sh --env flat   # 屏幕出现 Isaac 窗口 + 站立的机器狗 = 成功
```

> **哪些资产 clone 就有、哪些要拉取**（新机排障用；`.gitignore` 是权威）：
> - **git 追踪（clone 即有）**：`assets/urdf/*.urdf`（合成机器人本体）、
>   `assets/lidar_configs/*`（Mid-360 近似 usd/json）、`assets/policies/*`（RL 策略权重）。
> - **脚本拉取/生成（不入库）**：`assets/unitree_ros`+`assets/piper_ros`（`clone_deps.sh`）、
>   `assets/sensor_meshes/*`（`fetch_sensor_meshes.sh`——`go2w_sensored.urdf` 引用它俩，缺则网格不显）、
>   `IsaacLab`/`robot_lab`（`clone_deps.sh` 的 vendored 检出）、`refs/`（§1.5 手动 clone）、
>   `wheels`/`docker-cache`/`logs`/`var`（运行时产物，自动生成）。
> 上面 §1.4 的命令顺序已把「先 fetch 网格 → 再 compose URDF」排对，照抄即可。

## 1.5 导航栈容器

CMU 差速/麦轮自主导航栈的我们 fork（SLAM + 地形分析 + FAR/local planner + TARE 探索）。
`refs/` 是 gitignored 的（不随本仓库分发），故需手动 clone 到 `refs/` 后打补丁、建镜像、编译。
基础镜像 `osrf/ros:jazzy-desktop` 是**公开**的（无需 NGC 登录，与 Isaac 侧不同）。

```bash
cd ~/go2w
# 1) clone 导航栈源码到 refs/（公开仓库，jazzy 分支——上游 jizhang-cmu 麦轮栈的我们 fork）
git clone -b jazzy https://github.com/Yuxin916/Navigation-Physical-Experiment.git \
    refs/Navigation-Physical-Experiment

# 2) 打 Isaac 适配补丁（use_sim_time 全链切真、生成 system_isaac_sim*.launch.py、
#    同步 pc2_to_livox / agent_bridge / run_navstack 等真相源文件）
bash scripts/nav/patch_navstack.sh refs/Navigation-Physical-Experiment

# 3) 构建 jazzy-dev 基础镜像（一次性；详见 refs/.../docker/README.md）
cd refs/Navigation-Physical-Experiment/docker
docker build --no-cache \
    --build-arg UID=$(id -u) --build-arg GID=$(id -g) --build-arg USERNAME=$USER \
    -t jazzy-dev:latest .
cd ~/go2w

# 4) 在容器里 colcon 编译工作区（arise_slam / livox_driver 三包跳过——见 --packages-skip）
docker run --rm -it --name navstack_build --net=host --gpus all \
    -v "$PWD/refs/Navigation-Physical-Experiment":/ws -w /ws \
    jazzy-dev:latest bash -lc '
      source /opt/ros/jazzy/setup.bash &&
      rosdep update && rosdep install --from-paths src --ignore-src -r -y &&
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-skip arise_slam_mid360 arise_slam_mid360_msgs livox_ros_driver2'
# 编译产物落在 refs/.../install/（gitignored；本机才有），run_all_forever.sh 会 source 它。

# 5) 固化成 navstack:ready（bringup.sh/restart_all.sh 用这个 tag 起容器）
docker run -d --name navstack_seed -v "$PWD/refs/Navigation-Physical-Experiment":/ws \
    -w /ws jazzy-dev:latest sleep infinity
docker commit navstack_seed navstack:ready
docker rm -f navstack_seed

# 完成标志：
docker images | grep navstack       # 看到 navstack:ready
```

> **坑 28（NAV_MODE=explore 才需要）**：上面的 colcon 默认不含 `tare_planner`（探索规划器）。
> 若要跑自主探索，先补编一次（`bringup.sh` 检出缺失时也会打印同款命令）：
> ```bash
> docker run --rm --memory 8g -v "$PWD/refs/Navigation-Physical-Experiment":/ws -w /ws \
>     navstack:ready bash -c \
>     'source /opt/ros/jazzy/setup.bash && colcon build --packages-select tare_planner'
> ```
> waypoint 巡航（默认模式）不需要它。
>
> `navstack:ready` 是「jazzy-dev + 已编译工作区」的 `docker commit` 快照——目前无一键脚本
> （承诺中的 `scripts/nav/setup_navstack.sh` 尚未落地）；以上 5 步就是完整可复现序列。

## 1.6 agent（说话的入口）

```bash
cd ~ && git clone https://github.com/Z-Robotics-Lab/z-agent.git && cd z-agent
curl -LsSf https://astral.sh/uv/install.sh | sh && source ~/.bashrc
uv venv .venv --python 3.12 && source .venv/bin/activate
uv pip install -e .
cp .env.example .env
nano .env    # 只需填一家 LLM 提供商的 KEY（文件里每家的变量名都写着，填一块即可）
```

> 【当前过渡期】仿真侧 `scripts/vector_os/run_agent.sh` 默认找 `~/Desktop/vector_os_nano`；
> 若你只装了 z-agent：`export VECTOR_OS_NANO_DIR=~/z-agent`。F2 落地后本节直接变成
> `za --world go2w`，无需该变量。

**到此安装结束。以下是日常使用。**

---

# 第 2 章 · 仿真模式日常使用

## 2.1 启动全链（每次开机后一次）

```bash
cd ~/go2w
bash scripts/nav/restart_all.sh
# 等 2-6 分钟，最后一行必须是 "ALL-GREEN: 全链就绪"；GATE-FAILED 就再跑一次
```

【P5.1 落地后】换 `bash scripts/nav/bringup.sh`（幂等：环境已好会直接返回），
健康检查 `bash scripts/nav/status.sh`。

### 2.1.1 换场景（GO2W_SCENE）

```bash
GO2W_SCENE=office bash scripts/nav/bringup.sh teardown   # 换场景必须配对重启（坑42/43）
GO2W_SCENE=office bash scripts/nav/bringup.sh            # 默认 warehouse；office 首拉慢几分钟
```

不设 `GO2W_SCENE`（或 `=warehouse`）= 仓库（历史行为不变）；`=office` = 办公室。

## 2.2 和机器狗对话

```bash
cd ~/go2w
bash scripts/vector_os/run_agent.sh            # 交互模式，像聊天一样输入
# 或单发一句：
bash scripts/vector_os/run_agent.sh --no-permission -p "导航到 (2.0, 0.0)"
```

【F2 落地后】统一为 `za --world go2w`。

能说的话（示例）：
- `去 (2, 0)` / `导航到坐标 (1.5, -1)` —— 开到地图坐标
- `现在在哪` —— 报当前位姿
- `explore` / `探索这个仓库`【P5.2】—— 自主探索建图

**怎么判断真成了**：agent 最后输出 `VECTOR_VERDICT verified=true` 才算数——
这个判定读仿真的地面真值，agent 自己"说到了"不算。false 就是真没到。

## 2.3 看画面

- Isaac 窗口：狗和场景（restart_all 自动打开）。默认视角已对准出生的狗（每场景配了
  相机）。**视角乱了/看到屋顶或外景**：Stage 树里选中机器人 prim（/World/Robot）再按
  `F` 键 framing（把镜头框到机器人）；office 天花板高，滚轮缩放步长随视距变粗属正常。
- RViz【P5.1】：SLAM 点云/地形/路径/探索区域
- `logs/shots/`：每 30 秒自动截图

## 2.4 结束

不用管（常驻无害）；要清场：【P5.1】`bash scripts/nav/bringup.sh teardown`。
手动等价：`docker rm -f navstack`，然后
`docker exec go2w-isaac pkill -9 -f "kit/pytho[n]"`。
**绝不要**裸 `pkill python`——会误杀无关进程。

---

# 第 3 章 · 真机模式（Go2W + NUC）

前置：仿真模式你已经玩熟；NUC 按 [nuc-setup.md](nuc-setup.md) 配置完且
"验收自检"全部打钩；Go2W 按宇树说明书完成开箱激活、遥控器能手动遥控。

## 3.1 上电顺序（每次固定这么做）

1. **遥控器**满电、开机、确认在手——它是硬件急停，全程不离手
2. 狗趴平地，周围 3m 无人无障碍
3. Go2W 开机，等自检完成、站立
4. **先用遥控器手动走两步**——底盘正常，才交给 agent
5. NUC 上电（背部稳压器供电），等 ~1 分钟

## 3.2 你的电脑连 NUC（PC 只是遥控终端）

ssh = 在你电脑的终端里远程操作 NUC。推荐同一 WiFi：

```bash
ssh go2w@192.168.31.50     # IP 以路由器给 NUC 绑定的为准；密码=装 NUC 时设的
```

- 场地没 WiFi：手机开热点让两台都连上；或网线直连 PC↔NUC（别拔错口：
  网口1=狗、网口2=雷达）
- 想要图形界面/传文件方便：装 VS Code + Remote-SSH 插件，等于直接在 NUC 上开编辑器

## 3.3 启动机器人软件栈（在 ssh 会话里）

```bash
cd ~/go2W_Sim
bash scripts/nav/bringup.sh        # 【P5.4 真机版】导航栈+桥+宇树桥
bash scripts/nav/status.sh         # 输出 green 才继续
```

之后狗进入"听 agent 的"状态；遥控器任何时刻拨回手动即接管（硬件优先级最高）。

## 3.4 对话（和仿真一模一样）

```bash
cd ~/z-agent && source .venv/bin/activate
vector-cli --world go2w            # 【F2 后：za --world go2w】
```

你在仿真里学会的每一句话，这里原样能用——这就是 sim-to-real 的设计目标。

## 3.5 看画面（三选一）

1. PC 上装 RViz2（需 ROS2 Jazzy）+ 三个环境变量（域 42/fastrtps/UDPv4，见
   [nuc-setup.md](nuc-setup.md)）——经 WiFi 看轨迹地图够用，点云略卡
2. 远程桌面（NoMachine）连 NUC 桌面开 RViz——流畅，演示推荐
3. 不看图：只信 agent 的 verified 输出与位姿播报

## 3.6 停止与关机（倒序）

- **随时急停**：遥控器接管；或 agent 里 `停` / Ctrl+C
- 正常结束：agent 退出 → `bringup.sh teardown` → 遥控器让狗趴下 →
  NUC `sudo poweroff` → Go2W 关机

## 3.7 安全铁律（违反任何一条就不要开跑）

- 遥控器全程在手、有电、已验证能接管
- 首跑/改码后首跑：空旷场地、速度上限 0.6 m/s、不载重
- explore 时 3m 内不站人；楼梯/坡沿必须有物理隔挡
- 桥的 watchdog（0.4s 断令即停）与限幅是底线，不许绕过
- 电量 <20% 不开新任务

---

# 第 4 章 · 快速排障（先看这里再喊人）

| 症状 | 动作 |
|---|---|
| 安装卡在下载 | 全部脚本支持断点续传，重跑同一条命令即可 |
| `桥不在线` | `status.sh` 看哪层红：L0 容器没起→bringup；L3 红→等 30s；L4 红→SLAM 没收敛，查雷达口/网段 |
| agent 说 verified=false | 它真没做成——不是 bug 是诚实。看位姿，重新下指令 |
| SLAM 漂移/画面乱 | 配对重启 `bringup.sh`（单侧重启没用，必须两侧一起） |
| 狗不动但一切绿 | 真机：宇树桥进程在吗？遥控器是不是手动模式？仿真：查 RL 策略加载日志 |
| explore 和 goto 打架 | 【P5.2】桥会拒绝并提示；先说 `停止探索` 再下导航 |
| 其它诡异问题 | 翻 [pitfalls.md](pitfalls.md)——27+ 条前人踩过的坑 |
