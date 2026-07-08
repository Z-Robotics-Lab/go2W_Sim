# DEBUG — Findings A (点云地图倾斜/雷达外参) + B (轮滑移/摩擦)

> 独立会话（非 fix-a）。fix-a 会话持有 DEBUG.md/STATUS.md 单写锁 + sim 槽（warehouse 活跑）。
> 本文件是本任务的隔离 DEBUG，避免与 fix-a 的 DEBUG.md 写冲突。sim 用完由 fix-a 归还后，
> 本会话配对重启到 **office** 做受控验证。所有静态取证已完成（下），动态受控验证待槽位。

=====================================================================
# FINDING A — 地图整体倾斜（雷达外参）
=====================================================================

## OBSERVE（原始证据，不解释）
- CEO 实测：office 走廊 SLAM 地图整体仰起 ~20°（新截图），与真机 Mid-360 20° 安装仰角一致。
- 真机事实（CEO/硬件）：Mid-360 20° 仰角安装。
- 活跑现场为 fix-a 的 **warehouse** 驱动run（非 office），实测数字仅供机理定位，非 office 门验收。
- 静态取证（本会话，read-only）：
  - URDF `assets/urdf/go2w_sensored.urdf` `mid360_joint` origin: `rpy="0 0.3490658503988659 0"`
    = **+20.0° pitch（绕 Y）**。Ry(+20°) 使雷达 +X 轴指向 **下方 20°（nose-down）**（数值验证）。
  - `imu` link（:750）rpy=0 0 0（平），但 **sim IMU 传感器挂在 `mid360_link`（斜 20°）上**，
    非这个平 imu link。IMU_OFFSET_IN_LIDAR=(0.011,0.02329,-0.04412) 仅平移。
  - warehouse_nav.py 发布 IMU 时对比力施加**恒定 Ry(+20°)**（CY/SY=cos/sin20°, :748-750）把斜帧
    旋到水平帧。数值验证：机体水平时 Ry(+20°)@f_lidar=(0,0,9.81)，**正确回水平**。
  - SLAM 生效配置（容器 /ws 直挂 refs/，实测 live 一致）：
    `refs/.../arise_slam_mid360/config/livox/livox_mid360_calibration.yaml`：
    `extrinsicRotation_imu_laser=I`；`imu_laser_rotation_offset=[0,20,0]`（度）。
  - SLAM C++（parameter.cpp:220-234）：`T_i_l.rot = Ry(+20°)·I = Ry(+20°)`。
  - **LIVE 运行日志 /ws/system.log（决定性）**：feature_extraction + imu_preintegration 两节点均打印
    `updated pitch: 20.000000` → **SLAM 确实在应用 +20° 外参，非单位阵、非缺失**。
  - LIVE SLAM init 重力：`Gravity: 0.735, 0.198, -9.780` → `pitch offset gravity: -4.30°,
    roll offset gravity: +1.16°`（两次采样 21min 隔仍一致=**稳定的 init 重力偏差**）。
  - LIVE 发布 /imu/data 稳态均值（10s 驱动中）：acc=(-0.191,-0.323,9.962)|g|=9.969 → 仅
    **1.1° pitch / 1.9° roll** 倾斜 = 稳态 IMU 基本水平（sim 的 Ry(+20°) 稳态正确）。
  - LIVE /state_estimation 姿态（60 样本/15s，warehouse 驱动中）：roll mean=-8.34°(std1.13)、
    pitch mean=2.59°(std1.60) → 存在**持续的数度姿态偏置**（warehouse 现场，非 office 门数）。

## HYPOTHESIZE（每条带证据）
| # | 假设 | 类别 | 证据 | 状态 |
|---|------|------|------|------|
| A1 | SLAM 外参=单位阵/缺失 20° 补偿（CEO 假设） | 外参 | — | **REFUTED**：live log `updated pitch:20.0`；config 非 I |
| A2 | sim 装斜 20° + SLAM 认平 → 地图斜 20° | 判决矩阵 | URDF=+20°，但 SLAM 也=+20° | **REFUTED**：两边都 20°，数值验证 MATCH=level |
| A3 | sim Ry(+20°) 与 URDF/SLAM 符号相反 → 叠加成 40° | 符号 | 数值 consistency_check：p_imu==p_base | **REFUTED**：三方符号自洽，得水平图 |
| A4 | init 重力偏差（spawn 沉降瞬态被 use_imu_roll_pitch:false 冻死）→ 固定地图倾斜 | 重力init | live init Gravity x=+0.735=4.3°pitch；稳态 IMU 却仅 1-2° | **CONFIRMED（部分）**：4.3° 量级偏差真实、被冻结 |
| A5 | ~20° 观测的主量来自 office 专属（地毯/spawn 姿态/相机视角错觉） | 场景 | CEO 在 office 见 20°，warehouse live 仅见 4-8° | **待 office 受控验证** |
| A6 | imu_acc_x/y_limit=0.3/0.2 剪裁畸变重力估计 | 限幅 | 运行 config imu_preint 段 x_limit=0.3 | 待查（仅影响 acc_diff 预积分，非 init 重力矩阵） |

## DECISIVE LIVE 测量（2026-07-07，warehouse 活跑，read-only）
- **raw /lidar/points（sensor 帧）地面法向 vs sensor-Z = 19.70°（pitch-comp -19.69°）**
  → URDF 的 20° 挂载**确实保留在 sim**（sim 雷达真斜 20°，非转换丢失、非装平）。
- 同时 system.log **SLAM 应用 updated pitch:20.0** → 两个 20° 都在，数值验证相互抵消得水平图。
- **联合裁定：A1/A2/A3 全 REFUTED。sim 装斜 20°（对）+ SLAM 认斜 20°（对）→ 静态几何=水平。**
  CEO 的"SLAM 缺补偿"假设=运行时实证否定。**绝不能再加第二个 20°**（会把水平变成真 40° 倾斜）。

## 关键结论（静态取证 + live 机理，office 受控验证前的裁定）
1. **CEO 的"SLAM 缺 20° 补偿"假设被运行时证据 REFUTED**：SLAM 已应用 +20°（system.log 实锤）。
   sim/URDF/SLAM 三方 20° 符号自洽，静态几何得**水平图**（数值验证 MATCH=True）。
   → **不能盲改 SLAM 外参加第二个 20°**——那会把自洽的 0° 变成真正的 20° 倾斜（反向坑）。
2. **真实缺陷 = init 重力偏差被 `use_imu_roll_pitch:false` 冻结**：SLAM 只在启动 ~1s 窗口测一次
   重力定水平（Roll_Pitch_Gravity_Matrix，imu_data.h:122），此后不再用 IMU roll/pitch 纠正。
   若 spawn 沉降瞬态里机体在 pitch（live 实测 init 重力 x=0.735=4.3°），该错误**永久烙进整张图**。
   这正是 **z-bobbing / 静止漂移 / 地图倾斜的统一机理候选**：init 定向错 → 地面在图里是斜面 →
   quantileZ 地面估计逐帧移（DEBUG.md:1246 已记）→ z 起伏 0.277-0.401。
3. office 20° vs warehouse 4-8° 的量级差 → **必须 office 受控（静止 spawn）复测**才能定 office 真值。

## 修复方向（office 受控验证后定稿；本会话预备，未擅改 SLAM 外参）
- **不动** `imu_laser_rotation_offset`（已正确=20°）。
- 候选修法（待 office 数据定选）：
  (a) sim 侧：spawn 后延迟启动 SLAM init，或 spawn 时锁平机体，保证 init 重力窗机体真水平；
  (b) SLAM 侧：`use_imu_roll_pitch: true`（让 SLAM 持续用 IMU 重力纠倾，不靠一次性 init）——
      **触 SLAM 语义=CEO gate**，需 [RULING]；
  (c) 若 office 实测确为纯 spawn 瞬态：sim 侧 settle N 步再 unpause SLAM（config/时序，非红线）。
- **纳入单一真源**：`livox_mid360_calibration.yaml` 当前不在 sync_navstack_files.sh 清单（且 refs/
  被 gitignore）——把 SLAM 生效 config 纳入 sync（见修复实施）。

## 启动时序取证（Explore 子代理，代码只读，2026-07-07）+ 本会话红队
- **spawn 折叠姿态**：warehouse_nav.py:212-218 init_state thigh=0.8/calf=-1.5=折叠（非站立），z=0.42。
- **IMU 从 sim_t≈0.01s 即发布，无 settle 前置**（:346 reset → :541 loop → :770 publish）。
- SLAM 冻结重力=[first_imu, +1.0s] 平均（imuPreintegration.cpp:886），use_imu_roll_pitch:false 永不纠。
- **GO2W_STANDSTILL（:513-524）是 idle 死区控制，非启动 settle**——0.5s 后才介入，太晚，救不了 init 窗。
- 子代理裁定：机体折叠→站立沉降（~0.5s）与 SLAM [0.01,1.0]s init 窗**重叠** → 折叠瞬态被烙进重力帧。
- **本会话红队（诚实校准，防子代理过度断言 20°）**：
  - sim 发的是**软件水平化 IMU**（固定 Ry+20°），非原始斜帧——折叠瞬态的体 pitch 会经该固定旋转
    传到发布值，但**量级**由折叠幅度定，非必然 20°。
  - LIVE 实测 init `Gravity x=0.735 → 仅 4.3° pitch`（warehouse），稳态 IMU 仅 1-2° → **确证有几度
    固定倾斜偏差，但 warehouse 未见 20°**。office 折叠沉降是否更烈需受控实测。
  - 结论校准：**init-settle 缺陷=真实、已确证产生数度持久倾斜；office 20° 全量待受控复测**。
    修法（settle 后再启 SLAM init）无论量级都正确、无害。

## 验证脚本（本会话新增，read-only）
- `scripts/nav/ground_normal_probe.py`：订阅 registered_scan/map，RANSAC 拟合地面法向，报与竖直夹角。
  门：<2° = 水平。live warehouse 驱动中拟合噪声大（易锁墙面），office 静止采样才干净。

=====================================================================
# FINDING B — 轮转身体不前进（摩擦）
=====================================================================

## OBSERVE
- CEO 观察：office 大理石地面，轮子转但身体几乎不前进（滑移嫌疑）。
- CEO 硬指令（升级）：现实轮子摩擦很大不会漂移 → 轮摩擦设高值（1.5-2.0 起）+ 摩擦合成模式=max。
- 静态取证：warehouse_nav.py:325-331 已有 `wheel_mat = RigidBodyMaterialCfg(
  static_friction=1.6, dynamic_friction=1.4, restitution=0.0)` 绑到 FL/FR/RL/RR `_foot`。
  → 轮材质**已设**，但 (1) 合成模式未设（默认 average/multiply，被 office 低 μ 大理石吃掉）；
     (2) 绑定路径 `{foot}_foot` 需核对是否命中实际轮 collision prim。

## HYPOTHESIZE
| # | 假设 | 证据 |
|---|------|------|
| B1 | 摩擦合成模式默认 average/min → office 大理石低 μ 拉低有效摩擦 → 滑移 | CEO 指令；Isaac 默认合成非 max |
| B2 | wheel_mat 绑到 `_foot` 未命中真实轮 collision（URDF 轮 link 名不同） | 需查 URDF 轮 link/collision 名 |
| B3 | office.usd 地面 physics material μ 显著低于 warehouse | 待查 office.usd 地面 prim |

## 修复（CEO 硬指令，office 验证）
- 轮 collision 物理材质 static/dynamic friction 设高（1.5-2.0，实测定），restitution 0；
- **friction_combine_mode = "max"**（关键，否则场景低 μ 经 min/multiply 吃掉高摩擦；设 max 后换景免调）；
- warehouse_nav.py 场景配置内做（per-scene 允许）；
- 训练包络偏离照实记录（rationale：CEO 硬件事实；摩擦向上=安全方向，滑移减少更接近训练理想意图）。
- 验证：滑移率 1-v/(ω·r) → ≈0（r=WHEEL_RADIUS=0.086）；office GT 实速 0.40 → 0.55+；**转向回归**
  （wz 阶跃+弧线：零摔、轮 effort 不顶 23.5、无关节力矩异常、直立全程）。

## 训练包络偏离（诚实记录，CEO 硬件事实指令批准）
- 训练随机化（robot_lab velocity_env_cfg.py:267-268，go2w_flat 策略所训 locomotion 任务）：
  `static_friction_range=(0.3,1.0)`、`dynamic_friction_range=(0.3,0.8)`。
- 部署 play_cs.py:136-137 固定 (1.0,1.0)（已在训练随机化上界）。
- 本修 static=1.8 / dynamic=1.6（GO2W_WHEEL_MU_S/D 可调）→ **高出训练上界**（static 1.0→1.8,
  dynamic 0.8→1.6）。
- rationale：CEO 硬件事实（真机轮摩擦大不漂移）；向上偏离=安全方向（滑移↓→轮式运动学更贴训练
  理想意图，非引入策略没见过的低抓地失稳）；friction_combine_mode="max" 保证场景无关（PhysX
  优先级 max>multiply>min>average，轮设 max 后 office 大理石低 μ 不再吃掉高摩擦）。
- **转向回归是本偏离的红线验证**：高抓地改变原地转/弧线动力学负载（轮差速+腿协调），
  必须实测零摔 + 轮 effort 不饱和(23.5) + 直立全程，否则回退档位。

## 实施记录
- warehouse_nav.py:326-337 wheel_mat 加 friction_combine_mode="max"（关键）+ 高摩擦常量
  WHEEL_FRICTION_STATIC/DYNAMIC（:166 附近，env 可调 GO2W_WHEEL_MU_S/D）。
- 轮 link 名核对：URDF 轮=`{FL,FR,RL,RR}_foot`（continuous joint，left/right_wheel.dae，
  collision cylinder r=0.086 l=0.052）→ 现有 bind_physics_material 绑定路径**命中正确**（B2 REFUTED）。
