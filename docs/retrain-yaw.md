# Go2W yaw 重训专项（CEO B 案，2026-07-12）—— 预注册卷宗

现役 `model_5495` 对纯 wz 指令近零输出（wz=1.4→0.48°/s，原地 yaw 有效率 ~2%）。本轮 fine-tune
补 yaw 命令跟踪。单一真源=`scripts/sim/retrain_runbook.md`；本文件只加 yaw 轮的**根因、配方 delta、
预注册 gates、迭代预算、失败判据、回滚锚**。启训/续训/评估全命令照 runbook §3/§6。

## 根因（双机制，已从代码定罪，勿再litigate）
1. **命令分布**：5495 配方 `heading_command=True` + `rel_heading_envs=1.0`（env.yaml:1685）。
   `velocity_command.py:150-160` 对 100% 非站立 env **覆写**直采的 ang_vel_z（:134），换成
   `heading_control_stiffness(0.5) × heading_error` 裁到 ±ang_vel_z。⇒ 策略从未见持续纯 wz，wz 永远
   与非零 vx 耦合（弧线），从未见 vx≈0 & |wz|大（原地转）。部署 `/manip/cmd_vel` 直接置 ang_vel_z
   （无 heading 环）⇒ 纯 wz 双重 OOD。
2. **摩擦域**：训练地面 static=1.0/dynamic=1.0 combine=**multiply**（velocity_env_cfg.py:53-58），
   机体材质随机化封顶 static≤1.0/dynamic≤0.8（:267-268）⇒ 有效轮地摩擦封顶 ~1.0。部署=office 大理石
   μ_s=1.8/μ_d=1.6 combine=**max**（warehouse_nav.py:193-451）。四定轴轮原地转唯靠**蹭轮**（横向刮地）
   ⇒ 需高牵引，训练里没有。

## 配方 delta（`robot_lab_patch/go2w_yaw_command_friction.patch`，层叠在 envelope patch 之上）
起点 ckpt = `assets/policies/go2w_flat_payload_5495/model_5495.pt`（容器 run
`2026-07-07_07-53-57/model_5495.pt`）。其余（载荷包络、wheel_vel_penalty=-0.005、
rel_standing_envs=0.12、obs57/act16、执行器增益、decimation）**原样保留**——部署一致性红线不动。

| 项 | 5495 | yaw 轮 | 依据 |
|---|---|---|---|
| a. rel_heading_envs | 1.0 | **0.5** | 半数 env 保留直采 ang_vel_z（含 vx≈0&|wz|大角），半数留 heading 模式（边走边转） |
| a. ranges.ang_vel_z | ±1.0 | **±1.5** | 括住部署阶梯（wz 达 1.4） |
| b. track_ang_vel_z_exp.weight | 1.5 | **2.0**（2:3 配 lin 3.0） | 原地转 lin_vel≈0 时 lin 项近饱和梯度极小，yaw 项须够强驱动轮差；不超配（超配→零指令 yaw 漂移，伤①门）。std=0.5 不动 |
| c. randomize_rigid_body_material static/dynamic | (0.3,1.0)/(0.3,0.8) | **(1.4,2.0)/(1.2,1.8)** | 以部署 μ1.8/1.6 为中心的随机域 |
| c. terrain.physics_material.combine | multiply | **max** | 镜像 warehouse_nav.py:451，有效 μ=max 而非被压到操作数以下 |

幂等重放：`clone_deps.sh` → `robot_lab_patch/apply.sh`（envelope→yaw 两层，sentinel 检测，验过 3×幂等）。

## 迭代预算（fine-tune）
- resume from 5495（run `2026-07-07_07-53-57/model_5495.pt`；+**2000** iters，2048 env，seed 42，
  ~30min/5080——匹配 runbook §3 anchor）。§4a 开火即验 env.yaml：`rel_heading_envs: 0.5`、
  `ang_vel_z:[-1.5,1.5]`、`friction_combine_mode: max`、机体材质 (1.4,2.0)/(1.2,1.8)、
  `track_ang_vel_z_exp.weight: 2.0`；仍旧值→patch 没生效，停，重跑 apply.sh。§4b（~500it）
  `track_ang_vel_z_exp` 应爬升、episode 长度不塌；明显发散则停。

## 预注册 gates（**先于训练 commit，跑分后不许回调**）
**⚠ 前置工具改动**：现 `policy_acceptance.py` 段3只测**摔倒**，从不测达成 yaw 率——正是 yaw 问题此前
隐形的原因。评估会话须先给 `run_segment` 加 `yaw_rate_rad_s`（`robot.data.root_ang_vel_w[0,2]` 段内均值，
稳态窗），新增段 `3b_wz_track`（纯 wz=1.0/1.4，vx=0，测 achieved/cmd）。此工具改动是 E-T 门的前提。

- **G1 E-T 原生 yaw 跟踪**（3b 段，loaded body）：纯 wz=1.4、vx=0 稳态 achieved yaw 率
  **≥0.98 rad/s**（rel_err ≤0.30；推导：track_ang_vel_z_exp std=0.5，30% 内=exp(-(.42/.5)²)≈0.49
  的"跟上"带）。新 vs 旧同一 battery 对跑，旧 5495 此段必显著劣（现 ~2%）以证是重训之功。
- **G2 部署面**（office 活链，`/manip/cmd_vel` 纯 wz，眼看 sim + GT yaw 率实测）：
  wz=0.5→实测 ≥**0.35** rad/s（≥70%）；wz=1.0→实测 ≥**0.60** rad/s（≥60%）。
- **G3 回归不退化**（loaded body，新 vs 旧同 battery）：
  ① 零指令漂移仍 <0.02 m/s（段1，5495 的 creep 修复不破）；
  ② vx 跟踪（段2 vx=0.3/0.6 rel_err）与 crawl duty 劣化 **≤10%** vs 5495；
  ③ 段3 wz=±1.4 ×5 **0 摔**（fall_rate==0）。
- **G4 M1 复演**（bare-cli + 自然语言，眼看 sim）：`find(soup_can)` → 底盘对准（含原地 yaw）→
  `HOLD` 达成。这是 CEO 验收面。

## 失败判据 & 回滚锚
- **失败**：G1 未达（yaw 率 <0.98）**或** G3 任一退化（漂移 >0.02 / vx 劣化 >10% / 有摔）。
- **回滚锚**：`model_5495` 保持 `bringup.sh`/`restart_all.sh` 的 `GO2W_POLICY` **默认直至全 gates 过**
  （runbook §4d/§8）。切换=全门过后拷 ckpt+params 进 `assets/policies/go2w_flat_payload_<yaw>/`、
  改默认行、留旧路径注释一键回滚、旧 ckpt 不删。
- **升级路径**（均下一轮，不本轮回调门）：G1 达 G2 差→机体材质域上界再抬/combine 硬 μ=1.8 单点复训；
  命令分布不够→ rel_heading_envs 再降 0.3。
