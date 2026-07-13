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

---

## 第 2 轮（2026-07-13）—— 修 G3① 零指令漂移，保 G1 yaw 增益

**⚠ gates 阈值不回调。** §预注册 gates 的 G1/G2/G3/G4 数字**原样冻结**——本轮只换配方 delta，判据面
不动（这是铁律，非本轮可议）。第 1 轮 fine-tune（`model_7494`，run `2026-07-13_02-39-58`）**诚实 FAIL**：
G1 过（纯 wz=1.4 达 0.9843 rad/s vs 旧 0.8633，rel_err 0.2969≤门 0.30），**G3① 零指令漂移 FAIL**
（0.0337 m/s > 门 0.020，旧 5495=0.0112；伴 tilt 5.46° vs 1.93、mean_speed 0.102 vs 0.046）。
根因=`track_ang_vel_z_exp` 权重 1.5→2.0 **超配**换来的零指令 yaw 漂移，正是本卷宗 line 27 预注册的风险。
G2 未测（部署面，留全门过后）；G3②（vx 无退化）③（0 摔）均过。5495 保部署默认。

### 两变体（各自独立 patch，`apply.sh` 经 `GO2W_YAW_VARIANT` 选一；互斥、幂等、sentinel 检测）
两变体均**层叠在第 1 轮 yaw patch 之上**，均 fine-tune from `model_5495`（非 from 7494——回到干净起点，
每变体独立 run 目录），其余第 1 轮配方（命令分布 rel_heading 0.5 / ang_vel_z ±1.5、摩擦域 max 1.4-2.0/1.2-1.8、
载荷包络、obs57/act16、执行器增益、decimation）**原样保留**——部署一致性红线不动。

| 变体 | patch 文件 | delta（相对第 1 轮配方）| 依据 |
|---|---|---|---|
| **A 保守** | `go2w_yaw_variant_a.patch` | `track_ang_vel_z_exp.weight` **2.0→1.75**（其余第 1 轮不动）| 1.5(旧,无漂)与 2.0(第1轮,过G1但漂)的中点：留住多数 G1 yaw 梯度，退掉超配的站定破坏。std=0.5 不动 |
| **B 对症** | `go2w_yaw_variant_b.patch` | weight **保 2.0**；`stand_still.weight` **-2.0→-3.0**（1.5×）；`wheel_vel_penalty.weight` **-0.005→-0.0075**（1.5×，**严守 <-0.01 上限**）；`rel_standing_envs` **0.12→0.20** | 保住 G1 的 yaw 梯度，从**漂移根源=站定项**下手：第 1 轮漂是全身 creep（腿偏→tilt、轮滚→mean_speed），三杠杆分别治腿（stand_still 罚 ||cmd||<0.1 时腿 joint_deviation）、治轮（wheel_vel_penalty 唯一直接轮滚税）、加站定练习占比。三处均避开崩溃前例：-0.01 轮税已证 fine-tune 会过冲（漂 0.0277→0.0488），rel_standing 0.25 已证懒基坍缩（post-mortem H1），故 -0.0075<-0.01、0.20<0.25 |

B 的 5495 现值取证（`rough_env_cfg.py`）：`stand_still.weight=-2.0`(:214)、`wheel_vel_penalty.weight=-0.005`(:239)、
`rel_standing_envs=0.12`(:320)、`joint_pos_penalty.weight=-1.0`+`stand_still_scale=5.0`（后者本轮**不动**——两条腿站定杠杆
已足，加第三条冒 H1 懒基坍缩险）。

### 判据（先 A 后 B 串行，同 gates 面）
1. **先跑 A**（保守优先——单杠杆最小改动，回归风险最低）。A 若**全门过**（G1 达 且 G3①②③ 不退化）→ **选 A 收工**，
   进部署切换（runbook §8）+ G4 M1 复演（CEO 验收面）。
2. A 未全门过（尤其 G3① 仍 >0.02，说明只退 yaw 权重不足以消漂）→ **再跑 B**。
3. **两者都过** → 选 **G1 边际更厚者**（3b 段纯 wz=1.4 的 achieved yaw 率更高 / rel_err 更低者；第 1 轮 7494 边际薄=0.2969 逼近门 0.30，边际厚度是本轮首要甄别量）。
4. **都不过**（A、B 均 G1 未达或 G3 任一退化）→ **报 CEO**，不擅自第 3 轮/不回调门。

### 启训命令（先 A；A 不过再 B。每变体独立 run 目录，唯经 `run_retrain.sh`（capped），ONE-sim 串行）
```bash
cd ~/Desktop/go2w
# ---- 起前清场（ONE-sim 铁律；非空且非本训练→WAIT，禁 kill 兄弟）----
pgrep -fa "warehouse_nav.py|warehouse_scene.py|isaac-sim/kit|/isaac-sim/python.sh"   # 必空
# 遗留 Rl 僵尸 sim → runbook 认可的 scoped 清场（容器内 kit/pytho[n] 字符类，绝不宿主 pkill）：
#   bash scripts/nav/bringup.sh teardown

# ---- 变体 A（先跑）----
git -C robot_lab checkout -- \
  source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_go2w/{rough_env_cfg.py,__init__.py}
GO2W_YAW_VARIANT=a bash scripts/sim/retrain/robot_lab_patch/apply.sh     # 层叠 envelope→yaw→variant-a
GO2W_RETRAIN_RESUME_RUN=2026-07-07_07-53-57 GO2W_RETRAIN_RESUME_CKPT=model_5495.pt \
  scripts/sim/run_retrain.sh                                            # fine-tune from 5495, +2000 iters
# §4a 开火即验新 run env.yaml：track_ang_vel_z_exp.weight: 1.75、rel_standing_envs: 0.12、
#   stand_still.weight: -2.0、friction_combine_mode: max、rel_heading_envs: 0.5；仍旧值→patch 没进，停、重跑 apply.sh。
# 完成后评估：runbook §5.1（先旧 5495 锚后新，加了 3b_wz_track/yaw_rate 的 policy_acceptance.py），diff G1/G3。

# ---- 变体 B（仅当 A 未全门过）----
git -C robot_lab checkout -- \
  source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_go2w/{rough_env_cfg.py,__init__.py}
GO2W_YAW_VARIANT=b bash scripts/sim/retrain/robot_lab_patch/apply.sh     # 层叠 envelope→yaw→variant-b（与 A 互斥）
GO2W_RETRAIN_RESUME_RUN=2026-07-07_07-53-57 GO2W_RETRAIN_RESUME_CKPT=model_5495.pt \
  scripts/sim/run_retrain.sh
# §4a 验：track_ang_vel_z_exp.weight: 2.0、stand_still.weight: -3.0、wheel_vel_penalty.weight: -0.0075、
#   rel_standing_envs: 0.20；仍旧值→停、重跑 apply.sh。
```
注：`GO2W_YAW_VARIANT` 缺省=`none`=第 1 轮配方原样（不静默上变体）；`a`/`b` 互斥，checkout 里已有另一变体时 apply.sh
报错退出（须先 `git checkout` 复原 robot_lab）。回滚锚不变：`model_5495` 保 `bringup.sh`/`restart_all.sh` 默认直至新模全门过。
