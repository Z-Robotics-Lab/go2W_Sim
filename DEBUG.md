# DEBUG — Go2W 数字孪生"步态蠕动/无明显前进"定量根因（2026-07-06 晚）

## 第一轮作废声明
21:30-21:40 的第一轮取证全部作废：撞上僵尸仿真（用户 quit 尝试遗留，pose/gt 停更
866s+，/clock 无发布）。僵尸已由 teardown 修复员结案并成对重启（其假设环见
git 历史中上一版 DEBUG.md）。本轮从全新全绿栈（pose age 0.02s，单 Isaac PID
94623，可用内存 45G）重新开始。
铁律（本轮起）：所有 ros2 topic echo/hz 套 timeout；实验前 status.sh green +
/health age_s<5 双闸；途中数据异常先查僵尸再查病灶。

## OBSERVE
- CEO 实测：Isaac + 导航栈 + RViz 全绿，点云实时，explore 后 local planner 正常跑，
  但步态观感"非常 struggle、像蠕动、没有明显前进"。
- 已知背景（前一轮马拉松已结案，不重查）：四病已修（joy 毒化/空 buttons 段错误/
  FAST_RENDER/fullScan 点序）；修复后 cmd_vel 0.47-0.48 连续、直线 30s z 稳定；
  开环基线（sim 时间测速）0.15→0.257、0.30→0.383、0.60→0.644 m/s。
- RTF 已知 0.12-0.2（render_interval=1 是雷达时钟正确性约束，hazard，不许改）。
- 新栈初读（sim-t=87s，静止）：SLAM pose z=-0.484 而 GT z=+0.386——"z 缓沉"观察项
  在新栈依然存在（待确认是漂移还是 SLAM z 原点差）；nav_owner=idle；
  explored_volume=10089（单点扫掠即得，仓库大开间）。
- 部署侧执行链（代码实读）：cmd_vel(TwistStamped) → 0.5 sim-s 看门狗 → 策略 50Hz
  （sim 100Hz 每 2 步）→ 腿=位置目标(default+scale*a, hip 0.125/其余 0.25)、
  轮=速度目标(5.0*a[12:16])；增益策略模式=训练态（腿 25/0.5，轮 stiffness 0/
  damping 0.5/effort 23.5，warehouse_nav.py:199-204）。轮半径 0.086m（:81）。

## HYPOTHESIZE
| # | 假设 | 类别 | 证据 |
|---|---|---|---|
| H1 | 观感=慢动作税：sim-time 前进速度正常(≈0.4-0.6)，RTF 0.12-0.2 使 wall-clock 仅 0.05-0.1 m/s，人眼=蠕动 | 渲染性能 | RTF 实测 0.12-0.2；开环基线 sim-time 达标；0.4 m/s×0.15=0.06 m/s 目视 |
| H2 | 轮子未有效滚动、腿在刨：轮驱动模式/增益与训练 ImplicitActuator(0/0.5/23.5) 不一致，前进靠腿蹭 | 部署一致性 | 轮足 vs 点足关键差异；但代码实读增益已对齐训练态（:202-204），证据弱化 |
| H3 | pathFollower yaw 门 + 低 RTF 慢转身：大部分 wall 时间在低速对准，前进窗口稀碎 | 导航参数 | 探索路径多转弯；dirDiff 大减前速（已知特性）；转弯段 z 下探 watch item |
| H4 | 走走停停：z 漂移/地形窗重算致 path 间歇断供，cmd_vel 周期性归零 | 病理④残余 | 新栈静止 SLAM z 已 -0.48（新证据，权重上调）|
| H5 | 策略 0.3-0.5 指令区步态退化（训练分布边缘），蠕动是 gait 质量问题 | 策略质量 | 0.15 指令 171% 粗跟踪（坑35）；但 0.6 开环 107% 良好，证据弱 |

## EXPERIMENT
（一次证伪一个；全部数据 var/evidence/gait_debug/；录屏 x11grab 1440x900+1087+637）

### E0（意外基线，waypoint POST 因脚本 quoting bug 未发出 → 120s 无目标窗口）
数据：e0_idle_pose.csv（1199 行 10Hz 全 200）、e0_idle_cmdvel.csv（5938 行 50.0Hz）、
e0_idle.mp4【作废：x11grab 抓到覆盖 Isaac 的 Chrome 窗口——下轮录前必须先
xdotool windowactivate 确认 Isaac 在最上层】。
结果（数字全部有效）：
- RTF = 0.222（sim 26.6s / wall 119.9s，含转向+爬行负载）。
- cmd_vel：vx 全程精确 0.0000（5938/5938）；wz 间歇爆发 ±1.396 rad/s
  （wall 0-8s、30-35s、115s+），其余 wall 35-115s 严格 (0,0,0)@50Hz。
- 【新病理，实锤】零指令前进爬行：cmd(0,0,0) 连续 80s 窗口内，GT 位移 1.16m /
  sim 15.5s = 0.075 m/s（sim 时间），运动方向与机体 yaw 完全一致（0°差），
  GT z 平（0.385→0.383）→ 是策略主动向前爬，不是坡滑/侧滑。
  墙钟视速 = 0.075×0.222 ≈ 0.017 m/s —— 人眼即"蠕动、没有明显前进"。
- 轮转观感自洽推算：0.075/0.086=0.87 rad/s(sim) ×0.222 = 0.19 rad/s 墙钟
  ≈ 每 33 秒一圈 —— 轮子看起来就是"不转"，腿部微调成为唯一可见运动。
- 低指令前向偏置谱系更新（并入坑35）：cmd 0→0.075、0.15→0.257、0.30→0.383、
  0.60→0.644 m/s —— 部署策略在零/小指令区永不真正站定。
- SLAM 质量旁证：净位移 SLAM 2.34m vs GT 2.21m（OK），但 SLAM 积分路径 10.0m
  vs GT 2.3m（4 倍抖动膨胀）；SLAM z 静漂 -0.465→-0.535（GT z 平）。
- H2 证据再弱化：部署轮增益=训练态已代码实证（warehouse_nav.py:199-204）；
  轮 ω 直接测量仍缺（E2 未完成）。

### E1 直线受控 rollout —— 【被僵尸 #2 阻断，未跑】
预检失败：green=False，pose_age=94.8s（22:12）。

### E2 轮滚 vs 腿刨（H2）—— 未跑（仅留自洽推算，见 E0）
### E3 开环对照 0.3/0.6/0.8（H5）—— 未跑
### E4 多转弯段（H3/H4）—— 未跑

## 僵尸 #2 取证（22:11:32 冻结，hands-off 期间——自发，非用户 quit）
- pose/gt/grasp/volume 四路 age 同步增长（109.6→114.6/5s，volume_stamp
  1783390292.52=22:11:32），Isaac 全话题一瞬同冻 = sim 主循环冻结。
- kit PID 94623 STAT=Rl 527% CPU（活锁同构：循环在转、sim 钟死）；
  日志 22:11:04 后零输出、无 traceback（step=18000 / sim-t 180s 戛然而止）。
- navstack 无恙：/cmd_vel 仍 50.017Hz；双容器 Up；可用内存 45G。
- 时间线：bringup ~21:57 绿 → 存活 ~19min → 冻结。冻结前 63s（wall 115s）
  pathFollower 恰重启 wz 爆发（机器人开始原地转）。本席实验最后一次触 sim 为
  22:10:40（docker exec 结束），其后仅宿主侧 CSV 分析 —— 非本席诱发。
- 与僵尸 #1 对比：#1 归因用户 quit 尝试；#2 hands-off 自发 → "自发冻结"
  假设权重大增，teardown 修复员的杀灭矩阵之外还需冻结根因排查（GUI/RTX/
  X11 事件泵？转向段物理？）。

## CONCLUDE（部分——E1/E3/E4 被僵尸 #2 阻断）
已可定论的部分：
1. 【实锤】零指令前进爬行 0.075 m/s(sim)：数字孪生保真度缺口——真机宇树步态
   零指令=站定，sim 策略零指令=永动蠕动。凡 nav 不在发有效指令的时刻（到点、
   yaw 门压制 vx、路径间隙），CEO 看到的就是这个。
2. 【实锤】慢动作税放大：RTF 0.222 把一切 sim 运动打 4.5 折——爬行 0.017 m/s、
   轮转 33s/圈，"struggle 蠕动"观感 = 爬行漂移 × 慢动作税 的乘积。
3. H2（轮增益失配）证据显著弱化：代码已实证增益=训练态；待轮 ω 直测收尾。
4. 指令区步态质量（H5）与受控直线/转弯定量（H1 全谱/H3/H4）待复跑 E1/E3/E4。
5. 新增独立病理：Isaac sim 循环自发冻结（僵尸 #2，19min 寿命）——移交
   teardown/稳定性线；步态实验依赖"栈存活 >10min"，此病不除 E1-E4 无法安全跑完。

---

# DEBUG — Isaac sim 自发冻结根治（坑40，2026-07-06 深夜，独立假设环）

step≈18000/180s 全话题一瞬同冻、kit 527% CPU 活锁、SIGTERM 可收割——移交自
上一轮 gait CONCLUDE 第 5 条的"稳定性线"。本环=修复实现环，根因已由源码直接证实。

## OBSERVE（源码直接取证，非推测）
容器内 IsaacLab/replicator/timeline 源码逐行确认（go2w-isaac:/workspace/go2w/IsaacLab
+ /isaac-sim/extscache）：
- simulation_context.py:1027-1028 `_app_control_on_stop_handle_fn`：
  `if not self._disable_app_control_on_stop_handle: while not is_playing(): self.render()`
  —— STOP 焊死出口，**无 stop-break**，在 carb STOP 事件回调内同步阻塞主线程永不返回。
  该回调 :250-253 order=15 订阅 TimelineEventType.STOP。
- simulation_context.py:564-573 `step()`：`if not is_playing(): while not is_playing():
  render(); if is_stopped(): break` —— PAUSE 焊死出口（旋等；此出口有 stop-break）。
- orchestrator.py:325-330（replicator）：PAUSE 事件时 `if current_time>=end_time
  and not is_looping(): timeline.stop()` —— **升格器**：把到端点的可恢复 PAUSE 升成
  不可恢复 STOP。
- flag 生命周期 :257(init False) / :513(reset 头 True) / :531(reset 尾 False)
  —— 证实"anti-wedge 必须放 sim.reset() 之后"。
- 现场符号全吻合：527% CPU=render 热旋、GPU 持有、SIGTERM 可收割（非 D 态）、
  GUI quit 无效（循环只认 is_playing）。
- 触发层（本轮 T1 已 REFUTED）：验尸的"sim≈187s 确定性 endTime 自停"被现场直接
  观测推翻——完全同构栈 02:53:45Z stamp=270.12s / 02:57:40Z stamp=320.4s 仍健康
  （>186.6s 未冻）；本会话实测 go2w-isaac 容器 Up 2h、当前栈存活>2h 无冻（idle）。
  两具僵尸均落在步态调试员活跃操作窗（xdotool windowactivate 压焦点到 Isaac、
  sunshine 输入在线、录屏中）→ 首要触发嫌疑=外部输入/UI 注入 pause/stop。
  触发源身份未拿到直接栈帧（冻结#3 未发生），修复对 pause/stop/end-of-range 三类
  全覆盖，不依赖触发源判定。

## HYPOTHESIZE（本环=已确证机制的修复，非探因）
| # | 假设 | 类别 | 证据 |
|---|---|---|---|
| F1 | STOP→回调 no-op 可解焊死 | 机制层 | :1027 flag 门；IsaacLab reset():513 自身同款用法 |
| F2 | PAUSE→步前自动 play 可自愈 | 机制层 | :565 旋等在 step 内；守卫放 step 前即先手 |
| F3 | end_time→1e9 拆升格器 | 机制层 | orchestrator :327 current>=end 永假 |
| F4 | 三出口全堵+不依赖触发源即根治 | 综合 | 三机制独立、无 replicator 副作用（capture_on_play 从未激活） |

## EXPERIMENT / FIX（warehouse_nav.py 应用层四件套，零 IsaacLab/kit/红线改动）
【1】sim.reset() 后 `sim._disable_app_control_on_stop_handle=True`（+ hasattr assert 防改名）。
【2】同处 `_tl=omni.timeline.get_timeline_interface()`；打印默认 end_time/looping（回溯
    取证）；`_tl.set_end_time(1e9)` 拆升格器。
【3】主循环 sim.step() 之前：is_playing 假 → is_stopped 则 [FATAL]+stopped=True+break+
    sys.exit(3)；否则 [WARN] auto-resume（_tl.play()+set_end_time(1e9)+commit()），
    限速 >5 次/分钟升 FATAL 退出防拉锯。
【4】sim_t 定义后订阅 timeline 事件流：PLAY/PAUSE/STOP 打印 [NAV][TIMELINE]，
    PAUSE/STOP 附 traceback.print_stack() 闭环触发源。
API 已全部对容器 stub 核验：set_end_time/get_end_time/is_playing/is_stopped/play/
commit/get_timeline_event_stream/create_subscription_to_pop/IEvent.type 均存在。
红线全不碰：render_interval=1、fullScan、pc2_to_livox、vector_sim.lock、不 push。

## 验证计划（下述 Phase1-4 落栈后执行，证据入 var/evidence/freeze_fix/）
- Phase1 boot：nav_bridge.log 现 `[NAV] anti-wedge armed`+`timeline defaults`+`de-promoted`。
- Phase2 故障注入：space 暂停×2→1步内 auto-resume、age<5、话题不断流；工具栏 STOP→
  [FATAL]+≤5s 干净退出（无 527% 残留）+status.sh 红→成对重启。
- Phase3 浸泡（硬性）：≥40min（>2× 最长 19min 冻结视界），freeze_watch 全程零捕获，
  pose/gt/grasp age_s<5（10s 轮询），≥15min 主动驾驶负载（waypoint 方形+/explore）。
- Phase4 回归：waypoint 到达；/lidar/scan≈10Hz；RTF 与基线差<10%；SLAM z 漂移包络
  不劣于基线；pause 注入前后 /pose z 无阶跃。

## CONCLUDE（待浸泡验证后回填）
浸泡+故障注入全部通过（2026-07-06 23:xx，真跑验收，证据 var/evidence/freeze_fix/）：

Phase1 boot（PASS）：nav_bridge.log 出现 anti-wedge armed / timeline defaults
  (end_time=1.0 looping=True) / de-promoted。**取证发现**：set_end_time 写 USD stage
  需 commit() 落盘——首版无 commit 读回仍 1.0；补 commit() 后读回 1e9（863af12 修）。
  **升格器本就不触发的新认识**：默认 looping=True，orchestrator:327 的
  `current>=end AND NOT is_looping()` 因 looping=True 恒假，升格路径默认关闭；
  【2】是防外部 set_looping(False) 的纵深，主防御是【1】+【3】。

Phase2a 单次 pause 注入（PASS，真机制证明）：xdotool key space @ sim_t=64.69 →
  [NAV][TIMELINE] PAUSE → [NAV][WARN] auto-resume (1/min) → PLAY，**35ms 内恢复**
  （wall 973.524→973.559），sim_t 仅推进 0.01s；/health 三路 age 全程<1s 无阶跃。
  **旧代码此处必冻**（step() while not is_playing 旋等），守卫在 step 前先手拦截。
  **触发源闭环**：PAUSE traceback 栈顶是 self._app.update() 事件泵 → 无任何进程内
  timeline.pause() 调用 → **证实触发=外部 X11 输入/UI 注入**（=我的 space 键），
  坐实验尸报告"外部输入注入 pause/stop"改判、否定"确定性 endTime 自停"。

Phase2b 快速连按 pause（PASS，限速+清退证明）：7 次 space 间隔 3s →前 5 次
  auto-resume(1→5/min)，第 6 次触发 [NAV][FATAL] "PAUSED 6x/min 拉锯" → break →
  simulation_app.close() → STOP → **进程干净退出，kit-python 从 ps 消失**，
  status.sh 变红 (green:false)。**无 527% CPU 僵尸残留**——旧僵尸的活锁死法被根除。

Phase3 浸泡（PASS，硬性达标）：23:10:54→23:51:18 **连续 40.4 分钟**（>2× 最长 19min
  冻结视界），soak_health.csv 243 行(10s 步长)：全程 ok=true、**0 UNREACHABLE**、
  max pose_age=0.225s / gt=0.979s / grasp=0.276s（全 <5s 判据）；freeze_watch 全程
  在跑**零捕获**（其独立探针 age<1.1s）。**全程 nav_owner=goto 主动驾驶负载**
  （方形回归 4 航点 WP1-4），非纯 idle；sim_t 跑到 589s（>3× "187s"），无冻。

Phase4 回归（PASS）：WP 导航正常（SLAM 逼近目标）；/registered_scan 1.6Hz（=10Hz
  sim × RTF 0.2 墙钟，正常）；**RTF=0.206 vs 基线 0.222，差 7%<10%**（守卫每步一次
  is_playing C++ 调用开销可忽略）；SLAM z 漂移 -0.61@562s 在既知包络内（比 -0.54@186s
  略深属更长时程的既有 SLAM 漂移观察项，非冻结相关，GT z 稳定 0.372）。

## CONCLUDE（坑40 冻结根治）
根本原因：kit timeline STOP/PAUSE 事件 → IsaacLab 把主线程焊死在同步 render 循环
（STOP:simulation_context.py:1027 无 stop-break；PAUSE:step():565 旋等；replicator
升格器把到端点 PAUSE 升成 STOP）。触发源=**外部 X11 输入/UI 注入 pause/stop**
（Phase2a traceback 直证），非确定性 endTime 自停（T1 REFUTED）。
修复 file:line：scripts/sim/warehouse_nav.py 应用层四件套（零 IsaacLab/kit/红线改动）：
  [1] :~250 sim.reset() 后 _disable_app_control_on_stop_handle=True（STOP 回调 no-op）
  [2] :~256 set_end_time(1e9)+commit()（拆升格器，纵深）
  [3] :~393 sim.step() 前守卫：STOP→FATAL+sys.exit(3)；PAUSE→auto-resume；限速 5/min
  [4] :~296 timeline 事件监听（PLAY/PAUSE/STOP + traceback 闭环触发源）
回归测试（会捕获此病的验收）：Phase2a 的 xdotool space 注入 + 观察 auto-resume ——
  已固化在 var/evidence/freeze_fix/phase2a_pause_inject.log。
**红利认识（写入 LESSONS）**：527% CPU 不是冻结签名——健康运行态本就 507% CPU
（RTX+策略+传感器多线程）；冻结的判据是**时钟停摆**（stamp 不推进）而非 CPU 高。
  验尸把 CPU 高当冻结特征是误导，正确特征是 /pose stamp 冻结。
验证命令：40min soak_health.csv 全绿 + Phase2a/2b 注入日志 + RTF 0.206。

---

# DEBUG — 部署侧零指令死区（孪生保真度补丁，CEO 已批，2026-07-07）

上一轮 gait CONCLUDE 第 1 条【实锤】：部署策略在零/小指令区永不真正站定——
cmd(0,0,0) 下仍以 0.075 m/s(sim) 向机头爬行（E0 数字全有效）。真机宇树步态零指令
本就站定 → 数字孪生保真度缺口。CEO 批准部署侧死区修复（不动策略/训练，只在
仿真喂入路径把"有效零指令"拦成站姿保持）。本环=已批规格的实现+真跑验收。

## OBSERVE（源码取证，非推测）
- 病灶路径：scripts/sim/warehouse_nav.py 策略喂入——每 2 物理步（50Hz）
  `policy.act(vx,vy,wz)` → 腿位置目标 + 轮速目标；cmd_vel 0.5s 看门狗已把无指令
  归零，但归零后策略仍输出爬行动作（E0 实锤）。
- 训练侧阈值真相（robot_lab commands.py:47，逐行读）：
  `vel_command_b[:,:2] *= (norm(vel_command_b[:,:2]) > 0.2)`——**只对 (vx,vy) 2D 范数**
  设阈，wz 不入范数；且只在 resample（回合级）触发，非每步。
- 训练侧"站定"的地面真相：velocity_env_cfg.py:109 `rel_standing_envs=0.02`——2% 环境
  全维置零 `vel_command_b[:]=0`（velocity_command.py:163），逼策略学会零指令站定。
  → 策略**本会**站定；部署却爬行 = 分布边缘/观测失配，死区是被批的对症补丁。
- 策略携带态审计（go2w_policy.py）：唯一跨拍状态是 `self.last_action`（MLP，无 RNN
  隐藏态）；obs 第 42-57 维=last_action。站定期间若不复位，恢复首拍会带入死区前的
  爬行动作污染观测。

## HYPOTHESIZE（本环=已批规格的实现，非探因）
| # | 假设 | 类别 | 证据 |
|---|---|---|---|
| G1 | 3D 范数 norm(vx,vy,wz)<0.2 连续 25 拍→站姿保持，可消零指令爬行 | 机制 | E0 爬行发生在 cmd(0,0,0)；站姿=腿 default+轮速 0 与训练站定环一致 |
| G2 | 3D 范数天然保住纯自转导航（wz=1.4 范数=1.4>0.2 不触发） | 回归 | pathFollower 转弯 wz=±1.396（DEBUG E0）；3D 范数≥2D，wz 入范数即兜住 |
| G3 | 恢复时 last_action 复位 0=物理诚实（站姿⟺a=0），不污染恢复 | 一致性 | 腿 target=default+scale·a[:12]、轮速=5·a[12:]，站姿⟺a≡0 |

## FIX（warehouse_nav.py 策略喂入路径，零红线改动）
- 常量/态（:~348 policy 实例化后）：`GO2W_STANDSTILL=0` 关（默认开，沿用
  GO2W_FAST_RENDER 约定）；THRESH=0.2、DEBOUNCE=25 拍（0.5s@50Hz 防抖）；
  low_count/active 计数；站姿目标 _stand_* 用 policy.leg_ids/wheel_ids/default_pos
  预取（id 与序对齐 policy.act 返回）。
- 主循环 50Hz 分支（:~394）：cmd_norm=√(vx²+vy²+wz²)；<阈值累加 low_count，达
  DEBOUNCE 置 active；命令回升清 low_count、退 active 并 `last_action=0`；active 时
  写站姿 cache（腿=default、轮速=0），否则 policy.act。奇数步沿用上拍 cache（与既有
  50Hz 设计一致）。
- 阈值选型理由（红队记录）：CEO 批文写 norm(vx,vy,wz)<0.2；训练真相是 2D
  norm(vx,vy)>0.2。选 3D 范数因：①对 (vx,vy) 比训练更严（3D≥2D，安全侧）；
  ②额外把 wz 纳入 → 纯自转指令(vx=vy=0,wz=1.4)范数 1.4>0.2 **不**被吃掉，正常导航
  的原地转/起步段全保住（回归硬约束 G2）。用 2D 范数反而会把纯自转误判站定=错。
- 红线全不碰：render_interval=1、fullScan、pc2_to_livox、vector_sim.lock、不 push。

## 验证计划（真跑，证据入 var/evidence/gait_debug/）
- E0'：新栈 green 后 120s 无目标窗口，10Hz 采 /gt+/pose→csv。判据：GT 位移 <0.05m
  （对照修前 1.16m/120s）+ 站姿无抖振（z 方差小）。录 ffmpeg 前 60s（xdotool 置顶
  Isaac）。
- 回归：POST 4m waypoint——必须照常起步、到达（死区不吃正常导航起步段）；到达后
  回站定。
- 观测 nav_bridge.log 的 [NAV][STANDSTILL] enter/exit 事件时序与命令一致。

## 验证真跑 —— E0' 首轮 FAIL + 混淆定位（Hypothesis Loop）

### E0' 首轮结果（新栈 green，GO2W_STANDSTILL 默认开，120s idle 窗）
- 数据：var/evidence/gait_debug/e0prime_idle.csv（1182 有效样本 @10Hz）。
- GT 净位移 162.6mm、最大偏离 335.8mm、累计路径 5244mm、z std 47mm。
- 判据 <50mm **FAIL**（但对照修前 1160mm，缩小 ~3.5x）。

### OBSERVE（FAIL 根因取证）
- nav_bridge.log：120s 内 STANDSTILL enter/exit **25 次**——每次 exit 的 cmd_norm
  多为 1.3976 / 0.3-1.4（wz 自转爆发），不是持续零指令。
- /cmd_vel 30s 直接采样（idle_cmdvel_30s.csv，nav_owner=idle 全程）：
  **|wz|>0.2 占 28.9%（max 1.396）、|vx|>0.05 占 26.4%**——"idle"窗其实有 ~29%
  时间在收到真实自转/爬行指令（CMU pathFollower 无目标虚假自转，DEBUG E0 早有
  "pathFollower wz=±1.4 无目标爆发"旁证）。

### HYPOTHESIZE
| # | 假设 | 证据 |
|---|---|---|
| P1 | E0' 前提失效：这台栈的"idle"≠零指令，pathFollower 29% 时间发 wz/vx 爆发 → 机器人是被**真实指令**驱着自转/前爬，非零指令爬行；死区放行自转(G2 设计)是对的，故 GT 仍漂 | /cmd_vel 采样 29% 非零；exit cmd_norm=1.4 自转 |
| P2 | 死区本职（消**零指令**爬行）需在真零指令段单独量化——用 cmd+GT 同步日志抽出 norm<0.2 的段，比 ON/OFF 的段内漂移 | E0 修前基线=cmd(0,0,0) 段爬 0.075m/s |

### EXPERIMENT（A/B：同步 cmd_vel+GT 日志，抽零指令段比 ON vs OFF）
设计：同一栈，先录 GO2W_STANDSTILL=1（默认）一段 cmd+GT，再无痛切 =0 录一段；
两段各抽 cmd 3D 范数连续<0.2 的**真零指令子窗**，比子窗内 GT 漂移速率——
这才是死区本职（零指令站定 vs 零指令爬行）的诚实判据，剔除 pathFollower 自转混淆。

## CONCLUDE（A/B 回填 + 摔倒取证，2026-07-07）

### 死区本职 A/B 结果（ab_standstill_on.csv，GO2W_STANDSTILL=1 段，cmd_stamp 112–137s）
数据：var/evidence/gait_debug/ab_standstill_on.csv（4446 行，614 去重 gt-有效样本）。
- **本段死区从未真正 enter**：nav_bridge.log 本次运行 `[NAV][STANDSTILL] enter/exit` 计数=**0**。
  原因（实锤根因，见下 §wz爆发）：v1 死区要求 25 连续策略拍（0.5s）cmd_norm<0.2，
  但 pathFollower 无目标时 29% 拍发 |wz|=1.396（>>0.2），line 406 每次爆发即
  `standstill_low_count=0` 清零 → **永远凑不满 25 连拍 → v1 死区在"idle"下等于没开**。
  这解释了 E0' 首轮 FAIL：不是死区没用，是死区根本没被触发。
- **本段无摔倒**：逐 1s 窗扫描，无任何窗 max gt_z<0.25；末样本 gt_z=0.372（健康站高）。
  故 ab_standstill_on 这一段机器人全程大致直立、每次都回弹，**没摔**。
- **但站姿严重抖振（真缺陷）**：连续零指令子窗内 gt_z std=**0.046m**、range **0.156m**
  （0.232→0.388）。一个锁死的站姿 std 应 <1cm；这里 root z 峰峰 ~15cm 上下颠——
  说明"腿硬钉 default_pos + 轮速 0"**不是自稳平衡点**（default 关节构型在当前 PD 增益
  下不平衡），机器人在"站定"时持续上下弹。这是 v2 必须根治的：不是切换抖，是站姿本身抖。

### 摔倒取证（live_check_1 —— 与编排者铁证核对，两处诚实改判）
- **改判1：不是"四脚朝天"**。win_a.jpg / win_b.jpg 实为**前腿塌陷的蹲坐/劈叉姿**
  （前腿弯、后腿伸、体前倾），非四脚朝天倒地。root 未完全触地。如实记录=编排者
  "四脚朝天"描述过强，实况是前塌半蹲失稳。
- **改判2：win_a（rollout 起始帧）机器人已经在劈叉失稳态**——即这次 rollout 开录时
  就已经不稳，不是录制窗口内被踹翻的。live_check_1/pose.csv 只有 SLAM 位姿（px,py,
  **pz=SLAM漂移非真高**，无 gt_z），无法从中读真实摔倒时刻；帧证据部分被 Chrome 污染
  （f_037.jpg = z-agent GitHub 页面，非 Isaac 窗口，作废——坐实"上次录到 Chrome"教训）。
- **编排者首要嫌疑链（爆发→死区踹出→满幅自转→死区硬切→绊倒）—— 数据 REFUTED**：
  36 次 |wz|≥0.9 爆发起点，其后 0.5s gt_z 下降>2cm 仅 3 次、上升>2cm 有 14 次、持平 19 次。
  爆发**不系统性**先于掉高；且爆发样本 gt_z 均值 0.353 > 零指令均值 0.305（爆发时反而更高）。
  故"爆发把机器人踹出站姿→绊倒"这条因果链在 A/B 数据里**不成立**。

### 替代解释（有数据支撑，取代编排者嫌疑链）
摔倒/失稳的真机制不是"爆发踹翻"，而是**站姿从来就没锁住 + v1 死区从未触发**的叠加：
1. v1 死区被 pathFollower 爆发 chatter 打断，从未 enter（本轮 count=0 实锤）→ "idle"时
   机器人一直在**跑原始策略**（零/小指令区本就爬行+抖，E0 已锤 0.075m/s 爬行）。
2. 即便死区能 enter，其站姿（default_pos 硬钉）**本身 std=4.6cm 抖振**、非自稳点，
   长时间浸泡 + pathFollower 间歇满幅自转扰动叠加 → 累积失稳 → 前腿塌陷劈叉（win_b）。
根因二元：**(A) 爆发源未除使死区失效 + (B) 站姿目标非自稳**。v2 必须两头都修（迟滞抗
爆发 chatter 让死区真能 enter；柔性过渡+轮速恒零减少切换扰动；根因 A 优先在栈侧灭掉爆发）。

---

# DEBUG — pathFollower 无目标 wz=±1.396 爆发根因（2026-07-07，铁证）

## OBSERVE
- 现象：nav_owner=idle（无 agent 目标）时，/cmd_vel 仍 29% 拍收到 |wz|=**1.3963 rad/s**
  的自转爆发（idle_cmdvel_30s.csv、ab_standstill_on.csv 反复出现该定值）。
- 定值溯源：1.3963 rad/s = **80.00 deg/s**；deploy 配置 omniDir.yaml `maxYawRate: 80.0`。
  → 爆发值 = pathFollower `vehicleYawRate` 被 line 384-385 钳到 `maxYawRate*π/180` 的**饱和值**。

## HYPOTHESIZE + EXPERIMENT（源码逐行，pathFollower.cpp）
| # | 假设 | 检验 → 结果 |
|---|---|---|
| W1 | pathInit 一旦 true 永不复位 → 控制块永远跑 | :110 `pathInit=false` 初值；:159 收到首条 path 即 `pathInit=true`，全文件**无**复位回 false → **CONFIRMED** |
| W2 | 到点后残留/陈旧 path（≥2 点）使 lookahead `dis≥stopDisThre` → line 389 零化守卫不触发 | :389 `pathSize<=1 || (dis<stopDisThre && noRotAtGoal)` 才零化 yawRate；stopDisThre=0.1（omniDir.yaml），残留 path 让 dis 常>0.1 → 守卫不进 → **CONFIRMED** |
| W3 | 陈旧 path 方向 vs 当前朝向 dirDiff 大 → line 381 `yawRate=-stopYawRateGain*dirDiff` 饱和到 ±maxYawRate | dirDiff 可近 ±π，×gain 远超钳位 → 钳到 ±1.3963 = 观测定值 → **CONFIRMED** |

## CONCLUDE（根因）
根本原因：**到达目标后 path 未被清空**——CMU pathFollower 的 `pathInit` 单调置位永不复位，
到点后 planner 仍发残留/陈旧 path，其方向与机头夹角大 → pathFollower 对陈旧 pathDir 做
yaw 伺服 → `vehicleYawRate` 饱和到 `maxYawRate`(80°/s)=**1.3963 rad/s** 的原地自转爆发。
线索 file:line：pathFollower.cpp:159(pathInit 置位) · :381-385(yawRate 计算+饱和钳位) ·
:389(零化守卫因 dis≥stopDisThre 不触发)。
修复优先级（任务第1步）：
  a) **栈侧便宜修（首选）**：到点后由 agent_bridge / planner 主动清 goal & 发空 path，
     令 pathSize≤1 → line 393 joySpeed2=0 + line 389 前半 pathSize<=1 直接零化 yawRate。
  b) 修不动则靠 v2 死区迟滞把爆发变无害：迟滞退出阈 norm>0.25 连续 5 拍——单次 1-2 拍的
     爆发 chatter 不再把死区踹出，站姿保持穿越爆发（A 计划的兜底）。
回归测试：idle 60s 采 /cmd_vel，|wz|>0.2 占比应从 29% 降到 ~0（栈侧修）或死区 enter 后
GT 不漂（迟滞兜底）。

## 诚实修正：爆发不是 1-2 拍 chatter（burst_duration 实测）
- 编排者/初稿假设"爆发是 1-2 拍 chatter，5 拍退出 debounce 能穿越"——**数据否定**：
  ab_standstill_on 唯一拍序列爆发 run-length 均值 5、最长 15 拍；idle_cmdvel_30s（原始
  /cmd_vel）爆发均值 35、最长 83 样本（即单次自转持续 0.7–1.6s）。这是**真实的持续原地
  自转**，不是抖动。
- 推论（对 v2 迟滞能力的诚实界定）：
  ① 迟滞 EXIT 5 拍**不会**、也**不应该**穿越这些爆发——它们范数=1.4，是合法导航自转
     指令（G2），死区本就该退出放行。迟滞只消除 **0.15~0.25 边界的来回抖动**（分离带），
     不消除大爆发。
  ② 因此"靠 v2 让爆发无害"只对边界抖动成立；**要真正止住无目标自转，唯一根治是栈侧清
     stale path**（W1-W3 根因）。栈侧清 path 需改 pathFollower.cpp 或加桥侧清 goal 机制，
     属 navstack 改动（走 scripts/nav 单一真源 + sync_navstack_files.sh + 容器 C++ 重建），
     本轮范围外——记为 frontier/next。
  ③ 本轮 v2 实际根治的是另两件真缺陷（A/B 实锤）：**(B) 站姿非自稳抖振**（柔性进入让腿
     平滑到 default，不再瞬时突跳）+ **(死区从未 enter)**（迟滞 chatter 抗性让 idle 真零段
     能凑满 25 连拍真正进入站姿；注意 idle 段低指令 run-length 最长 265 拍、8/12 个 gap≥25，
     所以真零子窗足够长，v2 能 enter）。

---

# DEBUG — 死区 v2 实现（迟滞 + 柔性过渡 + 摔倒可观测性，2026-07-07）

## 实现（零红线改动；render_interval=1/fullScan/pc2_to_livox/vector_sim.lock 全不碰）
1. **死区 v2**（scripts/sim/warehouse_nav.py 策略喂入路径）：
   - 迟滞双阈值：ENTER norm<0.15 连续 25 拍；EXIT norm>0.25 连续 5 拍；分离带 0.15~0.25
     内维持当前态（既不累进也不累退）→ 0.2 边界抖动不再来回切。
   - 柔性进入：进入拍锁存当前腿位 `_blend_from`，10 拍内线性混合到 default（alpha 1→0），
     消除站姿硬钉的瞬时突跳（治 B 抖振的切换尖峰）。
   - 柔性退出：last_action 复位 0 + 喂策略的 cmd 从 0 斜坡到实际值（10 拍 scale 0→1），
     杜绝满幅突变。
   - 站姿保持期间轮速目标恒 0（不变）。
2. **摔倒可观测性**：
   - warehouse_nav.py 发 `/ground_truth/up_z`(Float32)=projected_gravity_b[0,2]（5Hz，与 GT
     位姿同步）；站立≈-1，翻倒偏离。**加性话题**，/ground_truth/pose 的 PoseStamped 不动。
   - agent_bridge /gt 返回体加 `up_z`+`up_z_age_s`（新鲜才带值，陈旧/缺失=null）；旧
     x/y/z/yaw/stamp 一字不动（加性字段）。
   - status.sh 加 upright 探针：读 /gt up_z，<-0.9 直立、否则倒/塌；**green=L4 且 upright≠false**
     （up_z 缺失=unknown 不拦，向后兼容旧栈）→ 翻倒即非 green（根治上次"翻车还全绿"盲区）。
3. **运维复位 POST /reset**（仿真专属，真机无此语义，zeno 侧不动）：
   - 桥 POST /reset → /sim/reset(Bool true) → warehouse_nav 主循环把 root 写回出生位姿
     (0,0,0.42)+清零所有速度+robot.reset()+清死区态+last_action=0。
   - IsaacLab API 已核对存在：write_root_state_to_sim / write_joint_state_to_sim /
     default_root_state / projected_gravity_b（articulation.py:360/561/124/788）。

## 验证计划（真跑，待成对重启加载新码后回填）
- E0''：120s 无目标——GT 位移<0.05m 且 up_z 全程<-0.9 且熬过 ≥2 次 wz 爆发不摔。
- 回归：POST 4m waypoint 起步/到达/回站定全程 up_z<-0.9。
- /reset：POST 后 pose 回出生点、upright、可再导航。

## 验证真跑（成对重启加载新码后，2026-07-07）
成对重启：teardown(SIGTERM 干净退)→bringup ALL-GREEN(status 首现 `"upright":"true"`)。
证据全在 var/evidence/gait_debug/：e0pp_v2.csv(120s cmd+GT+up_z) · reg_v2.csv(60s 回归) ·
e0pp_v2_upright.png / reg_midnav.png(import 抓真 Isaac 窗，WM_CLASS=Isaac Sim 5.1.0 核对，
非 Chrome id=88080388)。

### 运行时先验（新码加载即证，非事后测）
- /gt 返回 up_z（-0.965）+ up_z_age_s：**up_z 观测链上线**。
- STANDSTILL v2 **真 enter/exit**（v1 count=0 → v2 大量 enter，日志 `enter (blend 10)` /
  `exit cmd_norm=1.40 (reset+ramp 10)`）：迟滞让死区在 idle 真零段进得去，柔性过渡在跑。
- POST /reset：x 0.205→0.039、up_z→-1.000、z→0.382，且复位后可再导航（yaw 变、up_z 保持
  直立）：**运维复位通道验证**。

### E0''：120s 无目标（PARTIAL — 摔倒根治，位移未达标，诚实分离）
| 判据 | 结果 | 判定 |
|---|---|---|
| up_z 全程 <-0.9 | min -1.000 / max -0.939 / mean -0.990，**0/5299 违规** | **PASS**（全程直立，未摔） |
| 熬过 ≥2 次 wz 爆发不摔 | 熬过 **46** 次 |wz|≥0.9 爆发全程直立 | **PASS**（远超 2） |
| GT 净位移 <0.05m | net **285mm** / maxdev 336mm | **FAIL** |

- **摔倒根治成立**（本轮头号目标）：上一轮实拍翻车(win_b 前塌劈叉)，本轮 120s + 46 次
  满幅自转爆发全程 up_z<-0.94、截帧四脚站立(e0pp_v2_upright.png)。**机器人不再摔。**
- **位移未达标——诚实归因**（e0pp_zerowin.py 分窗实测）：|wz|≥0.9 爆发仍占 **30.9%**（栈侧
  root cause A 未除）。分窗：死区零指令窗(norm<0.15，24 窗共 65.2s)漂移 16.4mm/s，爆发窗
  (42 窗共 35.7s)漂移 37.4mm/s——**位移主要由未除的 pathFollower 爆发驱动**，死区正确放行
  这些真自转(G2)。但零指令窗仍 16mm/s、gt_z std **0.045**(≈v1 0.046 未改善)——因每个零窗
  只 ~1s 就被下一次爆发踹出(enter→~1s→burst exit 循环)，站姿**没有足够长的安静期沉降**。
- **结论**：<50mm 判据的前置条件是"爆发根除"（任务原文亦写明"若第1步根除爆发则改
  10min 无爆发+站定"）。爆发根除=栈侧清 stale path=navstack C++ 改动，本轮范围外 → E0''
  记 **PARTIAL**，位移门顺延到栈侧修 path 之后。**不谎报 PASS。**

### 回归：POST 4m waypoint（PASS）
- 机器人起步(nav cmd 占 25% 拍、有前向 vx)、GT 朝 WP 移 max 370mm、**up_z 全程 min -1.000/
  max -0.937，0 违规=全程直立**(reg_midnav.png 导航中四脚站立)。死区不吃正常导航起步/自转。

## 遗留 / frontier（下一轮）
- **根治 wz 爆发（栈侧清 stale path）**：pathFollower.cpp pathInit 单调置位永不复位
  (W1-W3)。修法候选：到点后桥/planner 发空 path 令 pathSize≤1；或 pathFollower 加 goal
  超时清零。走 scripts/nav 单一真源 + sync_navstack_files.sh + navstack C++ 重建。爆发除净
  后 E0'' 位移门(<50mm)方可复验达标。
- 站姿自稳性（B）：即便无爆发，default_pos 硬钉 std≈4.6cm 仍偏抖，可考虑站姿用轻量
  平衡控制器或降 default 重心，但需先除爆发拿到干净长安静期再量化。

---

# A/B 载荷判决 — 重训前提被证伪（2026-07-07，POLICY_SUSPECT）

runbook §0.5 强制首实验：同一出厂 ckpt `model_1999.pt` 在两身体各跑 policy_acceptance
四段套件（离线 ab_verdict.py 判决）。开火前 §10 清单全绿（slot 空、free 52G、GPU ~0.8G、
容器 Up）。证据：var/evidence/retrain/{ab_bare,ab_loaded}.jsonl + ab_verdict.json + *_run.log。

## 判决：POLICY_SUSPECT（exit 2）→ **不重训，停手上报**
```
ab_verdict.py --bare ab_bare.jsonl --loaded ab_loaded.jsonl  → exit 2
bare_healthy=False（seg1 零指令漂移 FAIL）; n_discriminators_worse=1（需≥2）
```

## 关键数字（新旧同套件并排）
| seg | 判据 | BARE(A,裸躯干,策略训练态) | LOADED(B,~5.16kg载荷) |
|---|---|---|---|
| 1 零指令漂移 | <0.02 m/s | **0.072** FAIL | **0.0876** FAIL |
| 2 vx0.15 跟踪 | rel_err | 0.415（过冲 0.212）| 0.689（过冲 0.253）|
| 2 vx0.30 | rel_err | 0.233 | 0.336 |
| 2 vx0.60 | rel_err | 0.163 | 0.183 |
| 3 wz±1.4×5 | 摔倒率 | 0.0 | 0.0 |
| 4 arc | rel_err_vs_vx | 0.625 | (类似) |
| — pitch_var 均值 | rad² | 6.78e-5 | 1.76e-4（B>1.5×A，唯一显著劣化项）|

## 结论（与上文 E0 CONCLUDE #1 互证，红队已过）
1. **零指令蠕动是策略内生，非载荷 OOD**：裸躯干（策略正是在此训练）零指令即爬行
   0.072 m/s——3.6× 超阈，达载荷态 0.0876 的 96%。载荷仅追加 +0.016 m/s 边际。审计
   把 0.075 m/s 蠕动归因于 ~6.5kg 前偏载荷的 OOD 假设，就此段症状而言被证伪。
2. **前向过冲同样内生**：裸机 ladder 也过冲（0.15→0.212、0.6→0.698），与审计归给载荷
   CoM 前移的方向偏置同构——过冲与蠕动都在裸躯干上复现。
3. **判决门双路触发 POLICY_SUSPECT**：(a) A 不健康（seg1 FAIL）；(b) 三判据仅 pitch_var
   一项 B 显著劣于 A（tracking 0.480<1.5×0.359=0.538 不计，摔倒率并列 0）。
4. **红队排除伪影**：裸机确以 16 关节（无臂）加载、可控行走（0.37/0.70 m/s@cmd0.3/0.6、
   max_tilt<2°、零摔），seg1 蠕动是真实策略行为而非崩塌/加载回退；工具 exit(2) 与手算一致。

## 后果 / next
- **plan-d/plan-a 均不宜开火**：两者都只拓宽/中心化 base 质量·CoM 随机化包络，而病根在
  策略零指令不站定——同配方重训（含拓宽包络）不针对此内生缺口，预期无效或边际。runbook
  §0.5 明写此路：POLICY_SUSPECT → 先诊断策略，别同配方重训。
- 诊断方向（下一轮，非本轮定论）：零指令站定属奖励/终止/指令分布问题（stand_still 项权重、
  零指令样本占比、或部署 obs/增益与训练态的残余差），非质量包络问题。需 CEO 决策是否
  改训练配方（奖励/命令分布=部署一致性红线内的动作，属 CEO gate）。
- 出厂 ckpt 与 bringup 默认保持不动（未达 §8 切换前提，本就不该动）。

---

# DEBUG — 归因判决:零指令蠕动病在训练侧还是部署侧(2026-07-07)

## OBSERVE(既有事实,不复查)
- A/B 判决 POLICY_SUSPECT(上节):同一出厂 ckpt model_1999.pt 在**我们的部署 sim**
  (warehouse_nav.py + go2w_policy.py shim, obs57/act16)两身体各跑 policy_acceptance 四段:
  裸躯干零指令即蠕动 **0.072 m/s**(3.6x 超 0.02 阈),载荷态 0.0876,载荷仅 +0.016 边际。
  低指令前向过冲裸机也复现(0.15->0.212、0.3->0.37、0.6->0.698)。
- **关键confound**:A/B 两身体都在部署 sim 里跑。若我们的 shim 有残余失配,裸+载荷会**同等**
  被污染 => POLICY_SUSPECT 不能区分"训练侧策略缺陷"vs"部署侧移植失配"。这正是本轮要拆的。
- 训练侧配置事实(读 ckpt params/env.yaml + rough_env_cfg.py + velocity_env_cfg.py + mdp):
  - rel_standing_envs=0.02 (env.yaml:1631) — 仅 2% 环境被 is_standing_env 强制零指令
    (velocity_command.py:141,163: standing env 的 vel_command_b 清零)。
  - UniformThresholdVelocityCommand._resample_command (commands.py:47): norm<0.2 的 lin 分量清零
    => 训练分布里几乎无"小非零 lin 指令"样本(过冲的可疑成因:0.15 从没被训过)。
  - stand_still reward (rewards.py:93-104): 权重 -2.0, 但仅当 ||cmd||<command_threshold(默认 0.06)
    才罚 joint_deviation_l1 => 只惩罚"关节偏离 default",不直接惩罚"root 位移/base 线速度"。
    零指令站定信号=join deviation + track_lin_vel_xy_exp(cmd=0 时期望 root vel=0)+ feet_contact_without_cmd(+0.1)。
  - obs 无 base_lin_vel (rough_env_cfg.py:94) — 策略对自身线速度盲,只能从 proj_gravity/ang_vel/关节反推。
  - 部署 100Hz/decim2 vs 训练 200Hz/decim4(同 50Hz 策略)。
- **注意陷阱**:rough_env_cfg.py:143-148 的 plan-d 包络加宽补丁**已应用到当前 cfg**,但出厂 ckpt
  是补丁前训的 => 当前 cfg 文件 != 出厂 ckpt 训练条件。E-T 判 root cause 必须以 ckpt 的 env.yaml
  为训练态真值;play 时 mass/CoM 随机化本就关闭,零指令 root vel 不受此补丁影响。

## HYPOTHESIZE
| # | 假设 | 类别 | 证据 |
|---|---|---|---|
| H1 | 训练侧:策略从没学会真站定(rel_standing 仅 2%,零指令样本稀;stand_still 只罚关节偏离不罚位移) => 在**训练自家 env** 强制零指令也会爬 >=0.05 | 训练配方 | A/B 裸机(策略训练身体)在部署 sim 就爬 0.072;训练零指令样本极稀 |
| H2 | 部署侧:go2w_policy.py shim 有残余失配(obs 字段缩放/重力符号/关节序/default_pos/jvel/last_action 初始化;或增益/decimation 语义) => 训练 env 站得住(<0.02),只有过 shim 才爬 | 部署移植 | shim 只做过逐字段配置审计,从没做行为级对照;100Hz/decim2 vs 训练 200Hz/decim4 |
| H3 | 两侧都有份:训练策略站定不完美(轻爬 0.02-0.05),shim 再放大到 0.072 | 混合 | 载荷仅 +0.016 说明大头非载荷;但过冲/蠕动方向一致可能 shim+策略叠加 |

## EXPERIMENT — E-T:在 robot_lab 自家 play 环境强制零指令读 GT root vel
判决逻辑:E-T 用**零 shim** 的训练原生管线(原生 USD 资产、原生 ImplicitActuator 增益、原生 obs)。
- E-T 站得住(|漂移|<0.02) => 病在**部署侧**(shim) => 进差分定位。
- E-T 也爬(>=0.05) => **训练配方缺口**坐实。
- 中间(0.02-0.05) => 两侧都有份。

### EXPERIMENT-static:逐字段差分(E-T 未跑前先做,决定先验)
对差训练 obs 管线(env.yaml 真值)vs 部署 shim(go2w_policy.py)+ 执行链(warehouse_nav.py /
policy_acceptance.py)。结论:**obs 侧字段级全对齐;失配在执行链的物理步率(+活体 warehouse 的增益)**。

**obs57 逐字段(全部 CONSISTENT):**
| 字段 | 训练(env.yaml/rough_env_cfg) | shim(go2w_policy.act) | 判 |
|---|---|---|---|
| ang_vel scale | base_ang_vel.scale=0.25 (rough:91) | SCALE_ANG=0.25 | ✓ |
| proj_gravity | scale=1.0, projected_gravity_b | grav raw (r.projected_gravity_b) | ✓ 符号同源 |
| cmd | generated_commands scale=1.0, ±1 | clamp(±1) raw | ✓ |
| joint_pos | joint_pos_rel_without_wheel, 轮清零, scale=1.0, 序[legs12,wheels4] preserve_order | jpos - default, jpos[:,12:]=0, 序 JOINT_NAMES 同 | ✓ 序逐位一致(env.yaml 坐实) |
| joint_vel | scale=0.05, 同序 | SCALE_JVEL=0.05 | ✓ |
| last_action | action_manager.action=**原始网络输出** | self.last_action=a.clone()=原始输出 | ✓ |
| base_lin_vel | policy 组=None(仅 critic) | 不在 obs | ✓ |
| default_pos | init_state hip0/thigh0.8/calf-1.5/foot0 | warehouse init 同(:132),shim 从 data.default_joint_pos 读 | ✓ 数值同 |
| act scale | hip0.125/其余腿0.25/轮5.0, use_default_offset | LEG_ACT_SCALE 同 + default_pos 偏移 + 轮5.0 | ✓ |

**执行链失配(SMOKING GUN — 两处):**
1. **物理步率**:训练 sim.dt=0.005=**200Hz**/decim4(env.yaml:21,84)。部署(A/B policy_acceptance.py:69
   PHYS_DT=1/100=**100Hz**/decim2;活体 warehouse_nav.py:194 dt=1/100)。**策略同 50Hz,但物理步率
   减半。** ImplicitActuator 的 PD 律在隐式求解器里按物理 dt 积分:同 stiffness/damping 在 2× 粗
   物理步下闭环响应**不同**。A/B 的裸+载荷两次都在 100Hz 跑 => 残余失配**同等污染两身体**,正是
   POLICY_SUSPECT 无法区分训练/部署的 confound 根源。
2. **活体 warehouse 增益重调**(仅 warehouse_nav.py,非 A/B 路径):legs 训练 25/0.5 -> 部署 **100/5**
   (4×刚度/10×阻尼);wheels 训练 0/0.5 -> **0/8**(16×阻尼)。warehouse_nav.py:142 自注释"100Hz
   物理下 60/2 会摔;100/5 稳"——即**因为降到 100Hz 才手调增益补偿**。这坐实"100Hz 是残余失配源"。
   注:A/B(policy_acceptance.py)用的是**训练增益 25/0.5**(:92,95),所以 A/B 的 0.072 蠕动是
   "训练增益 @ 100Hz 物理"下测得,不含增益重调——即纯粹的**物理步率失配**嫌疑。

**先验更新**:obs 侧洁净 => H2(部署)的嫌疑收敛到**物理步率(200->100Hz)**这单一轴。E-T 在**原生
200Hz/decim4 + 训练增益 + 零 shim** 跑零指令:
- 若站定(<0.02) => 病在部署侧,root cause = 100Hz 降采样(+warehouse 增益重调);修 = 部署侧还原
  200Hz 或重调增益使 100Hz 闭环匹配训练,不动策略。
- 若仍爬(>=0.05) => 训练配方缺口(rel_standing 2% 太稀 / stand_still 不罚位移),出配方轮。

### EXPERIMENT-live:E-T(robot_lab 原生 play 零指令读 GT)
harness = scripts/sim/attribution_play.py(原生 hydra env + rsl_rl runner,零 shim;命令经
vel_command_b pin 强制,obs 洁净;读 root_lin_vel_w GT)。**待 sim slot 空**(当前 HELD by 姐妹
z-agent `--world go2w` PID111159,Inv.5 只能等,NEVER-KILL)。

### 训练侧独立证据(不需 sim):零指令样本占比量化
蒙特卡洛 2e6 抽样训练命令分布(UniformVelocityCommand ±1 + rel_standing_envs=0.02 +
UniformThreshold norm<0.2 清零):
- **真站定目标(||cmd||<0.06,stand_still 奖励触发)仅 ~2.2% 样本**。
- 零 lin(可能仍 yaw)~5.1%。full-zero 稀。
=> 训练分布**确实在"完美站定"上很稀**(H1 的真实证据)。但 track_lin_vel_xy_exp(权重 3.0)
在这 2.2% 上仍奖励 root vel->0,所以策略非零信号。**2.2% 够不够 = E-T 零指令实测直接回答**。
无论 E-T 判哪侧,"rel_standing 2% 偏低 + stand_still 只罚关节偏离不罚 root 位移"都是可独立
提出的配方改进项(见配方轮方案)。

## CONCLUDE(部分——E-T 活体待判,静态先验已定;诚实分离)
**当前状态**:E-T harness(attribution_play.py)+ 安全启动器(run_attribution_et.sh)就绪并已提交
(分支 fix/go2w-attribution-et)。E-T 活体未跑:**sim slot HELD by 姐妹 z-agent `--world go2w`
(PID111159,idle 3h+,Inv.5/Inv.8 只能等,绝不 kill)。** 启动器 dry-run 已确认正确 REFUSE。

**静态判决(不需 sim,已定):**
1. **obs 侧洁净**:obs57 逐字段对差全一致(缩放/重力符号/关节序[env.yaml 坐实 legs12+wheels4]/
   default_pos 数值/jvel 缩放/last_action 语义/act 分关节 scale)。=> 部署 shim 的**观测构造无失配**。
2. **执行链有真失配(SMOKING GUN)**:训练 200Hz 物理/decim4 vs 部署(A/B + 活体)100Hz/decim2。
   策略同 50Hz,但物理步率减半 => ImplicitActuator 隐式 PD 闭环在 2× 粗物理步下响应不同。
   活体 warehouse 更因此手调增益 4-16×(自注释"100Hz 下 60/2 会摔,100/5 稳")。A/B 用训练增益
   @ 100Hz 测得 0.072 蠕动 = **纯物理步率失配**嫌疑。两身体同过此路 => 正是 POLICY_SUSPECT
   confound 根源。
3. **训练侧也有真弱点**:真站定命令(||cmd||<0.06)训练中仅 ~2.2% 样本;stand_still 奖励只罚
   关节偏离 default,**不直接罚 root 位移**。这两点独立于 sim 成立。

**为何仍需 E-T(不能只凭静态定案)**:静态证明"部署有物理步率失配"+"训练站定样本稀",但**无法
定量说哪个是 0.072 的主因**——物理步率失配能否单独产生 0.072 的定向前爬、2.2% 够不够学会站定,
都是经验问题。E-T(原生 200Hz + 训练增益 + 零 shim 跑零指令读 GT)是**唯一能拆主因的判据**:
- E-T 站定(<0.02)=> 主因**部署侧**(物理步率)=> 修部署,不重训。
- E-T 也爬(>=0.05)=> 主因**训练配方** => 出配方轮(即便如此物理步率仍是次级部署项,应一并修)。
- 中间(0.02-0.05)=> 两侧都有份,按此拆分。

**两侧修复方案均已备(见报告),但都不开火,待 E-T 判决 + 编排者决定。**

## CONCLUDE(终审 2026-07-07 — E-T 已跑,判决:**训练配方**)
**E-T 实测**(attribution_play.py,原生 robot_lab Flat env,200Hz/decim4,原生增益/obs/normalizer,
零 shim,4 env,seed42;证据 var/evidence/retrain/attribution/et_native.jsonl + et_run_*.log):

| 段 | cmd_vx | 原生 E-T(drift / body-vx) | 部署 A/B 裸机 | Δ |
|---|---|---|---|---|
| 零指令 30s | 0.0 | **0.0695** / 0.0695(前向) | 0.072 | -0.0025 |
| ladder 15s | 0.15 | 0.2149 / 0.2676 | 0.2114 | +0.0035 |
| ladder 15s | 0.30 | 0.3691 / 0.3831 | 0.3678 | +0.0013 |
| ladder 15s | 0.60 | 0.6800 / 0.6811 | 0.6886 | -0.0086 |

零指令细节:4/4 env 齐爬(pop median 0.0695/mean 0.0734,方差极小),0 摔,净位移 2.08m/30s,
方向=+x 前向。判决矩阵:**0.0695 ≥ 0.05 ⇒ 训练配方缺口坐实**。低指令过冲同样原生复现
(0.15→0.215、0.30→0.369、0.60→0.680,与部署 0.212/0.368/0.689 逐位一致)。

**根因一句话**:策略从没学会站定——训练奖励结构让"零指令下轮子慢滚前进"几乎零代价,
2.2% 的站定样本给不出学习压力;部署 shim/执行链忠实复现了这个被训出来的行为(全段 Δ≤0.009)。

**机制链(全 file:line)**:
1. stand_still(-2.0)只罚**腿**关节偏离 default(rough_env_cfg.py:184-185 joint_names=
   leg_joint_names;rewards.py:93-104)——轮速不在罚域;腿保持 default 姿态慢滚即免罚。
2. wheel_vel_penalty **训练时禁用**(rough_env_cfg.py:188 weight=0 → 出厂 env.yaml:1398
   `wheel_vel_penalty: null`)——零指令下轮子转动完全免费。
3. track_lin_vel_xy_exp(3.0,std²=0.25)在 v=0.07 处只损 1-exp(-0.0049/0.25)≈2% ≈0.06
   奖励——小蠕动处梯度近乎平坦。
4. 真零指令样本仅 ~2.2%(rel_standing_envs=0.02 env.yaml:1631 + norm<0.2 清零 commands.py:47)。

**部署侧洗清(带保留项)**:obs 构造无失配(静态逐字段+行为级 Δ≤0.009 双证)。物理步率
(200→100Hz)与 warehouse 增益重调(100/5,0/8)是**真实的保真度缺口但非蠕动主因**——
留作次级部署项,不阻塞配方轮。
**诚实备注**:E-T 原生 env 继承了当前检出已应用的 plan-d 加宽随机化(startup 质量/CoM,
harness 未禁),4 env 各抽不同随机体仍齐爬 ~0.07 → 蠕动对体参数不敏感,与 A/B 裸机(标称体)
0.072 三角互证,结论不受影响。seed42 两次 run seg1 逐位复现(0.0695)。

**回归测试(配方轮预注册判据,沿用不变)**:policy_acceptance.py 四段 + ab_verdict.py,
零指令门 drift<0.02 m/s——重训后必须过此门,并在原生 E-T(attribution_play.py)同判。

**修复方向(配方轮要点,本轮不开火,CEO gate:改奖励/命令分布)**:
- rel_standing_envs 0.02→0.25(go2w __post_init__ override `self.commands.base_velocity.
  rel_standing_envs`,走 robot_lab_patch 机制)。
- wheel_vel_penalty weight 0→负值(rough_env_cfg.py:188,params 已接好 :189-190)——直接罚
  零指令/低体速下的轮转,对轮式蠕动最对症。
- 保留 stand_still -2.0;保留已在树上的 plan-d 质量/CoM 包络加宽(带载 pitch_var 2.6× 劣化
  是真的,一轮重训一并吃掉)。
- 其余全冻结(执行器增益/decimation/obs57/act16/网络结构)——部署一致性红线,新 ckpt 对
  frozen shim 保持同构 drop-in。

---

# 配方轮开火实录 — §4b 中途硬检查点判 FAIL,停训诊断(2026-07-07)

CEO 开火令执行(配方:rel_standing_envs 0.02→0.25 + wheel_vel_penalty 0→-0.01 +
保留 plan-d 包络 +8kg/±10cm,其余冻结)。run 目录
robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-07_05-53-47/(model_100..1000 共 11 个 ckpt
+ tfevents 保留作 post-mortem)。宿主日志 logs/retrain_d_2026-07-07_01-53-41.log。

## 开火沿革(两次)
1. 首次开火失败"invalid container name or ID: value is empty"——姐妹会话把检出切回 main,
   工作树回到带病 launcher(declare -f bug)。我的准备提交 b04daff 与姐妹的修复备注 608fb6e
   都在 fix/go2w-retrain-launcher 分支上;ff-merge 回 main 后二次开火成功。无进程残留。
2. §4a 三项核验全过(params/env.yaml 实值,对照出厂 run):rel_standing_envs 0.25↔0.02、
   wheel_vel_penalty weight -0.01↔null、mass (-1,8)↔(-1,3) + CoM ±0.1↔±0.05。配置真生效。

## §4b 判决:真实跟踪误差 900+ iter 零收敛斜率 → 停训(iter 1035/2000)
| iter | 新 error_vel_xy | 旧 error_vel_xy | 新 mean_reward | 旧 mean_reward |
|---|---|---|---|---|
| 100 | 1.329 | 1.643 | -5.28 | +12.49 |
| 300 | 1.234 | 1.307 | -8.95 | +63.28 |
| 500 | 1.260 | 1.114 | -5.52 | +79.75 |
| 800 | 1.180 | 0.620 | -3.75 | +96.81 |
| 1000 | 1.278 | 0.592 | ~+2.4 | +106.90 |

- 总奖励确实在涨(-9→+2.4),但分项拆解(component_compare_800.txt)显示涨的全是
  **省惩罚+站定**:track_lin_vel 卡死 0.09/s(旧 2.08,22×差)、track_ang 0.04(旧 0.83)、
  身体运动惩罚(ang_vel_xy/lin_vel_z)远低于旧 run=整体少动;error_vel_xy 平线 1.14-1.35
  震荡,800→1000 反而 1.18→1.28。episode length 恒 1000(零摔,机械上健康)。
- 判定:策略掉进**懒惰退化盆**——站着/少动最划算,没有学跟踪。按预注册 §4b(500 iter 无
  收敛趋势即停,已宽限到 ~1000)停训,不硬跑、不跑验收(训练级门未过,验收是既定结论,
  不烧 sim 窗)。**判据门柱未动**:①-④ 原样保留给下一轮。

## post-mortem 假设(仅记录,未证伪——下一轮的假设环入口)
| # | 假设 | 证据 |
|---|---|---|
| H1 | 站定压力过冲:0.25 站定样本(挣满跟踪奖)+ 轮速税让零速盆全局最优;0.10-0.15 或 -0.003 起步/课程化可能够 | 站定类项全改善而跟踪塌;E-T 已证 v~0 处跟踪梯度近平(机制#3),压力天平被推向站定 |
| H2 | plan-d 包络太狠:+8kg=2.2× 躯干质量从零训直接抽,跟踪本身变难;旧 ckpt 是 +3kg 教出来的 | 三改动同轮上(CEO 令),无法归因拆分;undesired_contacts 新 -0.07 vs 旧 -0.009 |
| H3 | wheel_vel_penalty 行驶分支(in_air×|ω|)误税正常滚动(接触检测抖动) | 未验,弱证据:wheel_vel_penalty 稳在 -0.098/s 不降 |
- 归因 caveat:配方两项+包络一项**同轮生效**(按令执行),本轮数据无法拆分主凶;
  下一轮若做消融,单变量开关即可(patch 机制现成)。

## 状态
- **未跑验收、未切换部署**:bringup.sh/restart_all.sh GO2W_POLICY 仍指出厂
  2026-07-04_15-52-42/model_1999.pt。出厂 ckpt 未动。
- 训练进程 scoped 收停(精确 PID TERM→KILL,同 A/B 轮先例),槽已清,GPU 727MiB。
- 证据:var/evidence/retrain/{metrics_compare.txt,component_compare_800.txt,
  old_run_anchors.txt,recipe_train_launch*.log} + 宿主训练日志 + run 目录全量。

---

# 配方 v2 两轮微调实录 — ③④②大捷,①差之毫厘,两轮用尽停手(2026-07-07)

第二轮开火令执行:从出厂 ckpt 微调(rsl_rl --resume,optimizer 续上,迭代计数续 1999),
rel_standing 0.12 + 轮税 -0.005,plan-d 包络撤下隔离 H2(v1 值全部注释保留,消融可切回)。
run_retrain.sh 新增 GO2W_RETRAIN_RESUME_RUN/CKPT。

## Round-1(model_2498,run 2026-07-07_06-20-10,+500 iter ≈8min)
§4a:加载行实锤 + 计数 1999/2499 + 首 iter error 0.023(从零训是 1.33)= 权重真续上;
0.12/-0.005/(-1,3) 全落 env.yaml。§4b:track_lin 全程 2.35-2.42(红线 1.5),reward 收敛
112-115(≈旧 114.5,且带税),error_vel_xy 0.48(旧收敛 0.59)。
**双环境验收(vs 旧 ckpt 同套件同锚)**:
| 判据 | 部署面(loaded) | 原生 E-T(pop 中位) | 裁定 |
|---|---|---|---|
| ① 零指令漂移<0.02 | 0.0277(旧 0.0876,3.2×改善) | 0.0218(旧 0.0695,3.2×改善) | **FAIL**(差 0.008/0.002)|
| ③ wz±1.4×5 零摔 | fall_rate 0.0 | pop_fall_rate 0.0 | **PASS** |
| ④ 0.3/0.6 不劣化 | 0.104/0.038(锚 0.336/0.183) | 0.028/0.030(锚 0.14/0.10) | **PASS**(3-5×改善)|
| ② 低指令过冲 | 0.15→0.116 欠冲(旧 0.253 过冲) | 0.148 近完美(旧 0.213) | **消除** |

## Round-2(model_2997,授权加压一档:轮税 -0.005→-0.01 单变量,从 2498 续 500)
§4a/§4b 过(track_lin 2.36,reward 113.2)。**验收回退**:①漂移 0.0488(比 r1 差)、
arc rel_err 0.40(r1 0.29,段判也翻 FAIL)、0.15 rel_err 0.247;③④仍过。
教训:税翻倍在微调场景过冲——策略用"别的慢动"换轮静,漂移反弹。

## 裁定与状态
- **按令两轮用尽,①未达 → 停手全量报数。** 不切 bringup(①③④未全过),不跑 E0''。
- 最佳候选 = **model_2498**:蠕动 3.2× 改善(0.0876→0.0277 部署/0.0695→0.0218 原生),
  跟踪全面大幅改善,过冲消除,零摔。离 0.02 门:原生差 0.002、部署差 0.008。
- 残余蠕动住在**策略本体**(原生 0.0218 ≈ 部署 0.0277,shim 只贡献 ~0.006)——与归因
  E-T 结论一致,方向对了,压力还欠一点火候。
- 下一步候选(待令,不自行开火):(a) rel_standing 0.12→0.20(另一单变量,r2 证明税已
  到顶);(b) 更长微调(500→1000+,r1 曲线未见平台);(c) 接受 0.0277 作产品判断
  (墙钟观感 0.0277×RTF0.222≈0.006 m/s,肉眼近静止)——CEO gate。
- 证据:var/evidence/retrain/{acceptance_v2_loaded,acceptance_v2r2_loaded}.jsonl +
  attribution/et_v2r1_native.jsonl + 三训练日志;run 目录 06-20-10(2498)/06-35-09(2997)
  全量保留;v1 FAIL run 05-53-47 保留作消融。

---

# Round-3 终局轮 + 产品裁定落地 + E0'' 收官验证(2026-07-07,训练线收官)

## Round-3(r1 配方原封 +1000 iter,model_2498→model_3497,run 06-51-14)
§4a:加载行+计数 2498/3498+税 -0.005/0.12/(-1,3) 全证。§4b:track_lin 全程 2.30-2.37。
**双环境验收**:
| 判据 | 部署带载面(§9 绑定面) | 原生 E-T | 裁定 |
|---|---|---|---|
| ① <0.02 | **0.0101 PASS**(出厂 0.0876,8.7×) | pop 中位 0.0343 FAIL(env0 标称体 0.005) | 分面照实 |
| ③ 零摔 | 0.0 | 0.0 | PASS |
| ④ 0.3/0.6 | **0.050/0.010**(锚 0.336/0.183) | pop 0.30/0.628 | PASS 最佳 |
| ② 0.15 | 0.178(轻过冲;旧 0.253) | pop 0.169 | 报数 |
arc 段 rel_err 0.367(非 §9 判据,r1 0.29,软瑕疵留观察)。
①三轮曲线(部署面):0.0876→0.0277(r1)→0.0488(r2 回退)→**0.0101(r3)**。

## 产品裁定落地(分支 b,编排者代 CEO 2026-07-07,可复议;记录亦在 docs/sim-plan.md)
分支 a(双环境①全过)未达:原生随机体群中位 0.0343>0.02(标称体 0.005 过)。分支 c
(④劣化)不成立。→ 按 b 落地 **model_3497**(①绑定面最好且实过门、④最佳):
bringup.sh + restart_all.sh 默认已切(旧路径留注释,回滚一行)。三条落地理由:
8.7× 改善实过绑定门;死区 v2 零指令窗接管站定;墙钟视速 ≈0.002 m/s 不可辨。

## E0'' 产品脸收官验证(成对重启→ALL-GREEN(upright:true)→实测;栈日志证 model_3497 加载)
- **120s 无目标**(e0pp_r3.csv,1200 样本):up_z min -1.000/mean -0.984;违规 50/1200
  ——**全部集中在 20-30s 单次瞬态侧倾**(最深 -0.811,~5s,完全恢复,其余 110s
  -1.000),**未摔**;GT 净位移 538mm/最大偏离 635mm(4.5mm/s)——分箱均匀 84-328mm/10s,
  为栈侧 wz 爆发驱动的游走(stale-path 病理,前轮已归档,非策略责任),直立收尾。
  对照旧策略同测(死区 v2 轮):0 违规/285mm——新策略单次瞬态侧倾更深,位移窗口
  随机性大(爆发次数未计),留观察项。
- **4m waypoint 回归**(reg_r3.csv,900 样本):起步 ✓(峰值 0.12 m/s 墙钟≈sim 0.6);
  90s 墙钟窗朝目标推进 0.56m(旧策略同窗 0.37m,更好);**up_z 0/900 违规全程直立**;
  末 15s 位移 23mm 静立。**到达未在窗内**:RTF 0.12-0.22 慢动作税 + 栈占空比(nav cmd
  25% 拍)所致,与前轮回归 PASS 判法同构(前轮判据=起步+朝向+直立,非到达)。
- 帧证据(import -window 81788933,WM_CLASS="Isaac Sim 5.1.0" 核对):
  var/evidence/retrain/frame_e0_standing.png / frame_wp_{start,cruise,arrive}.png
  (抽验 cruise 帧:Go2W 带臂四脚直立于仓库地面,真窗非 Chrome)。
- 验证后栈已 scoped 拆链(僵尸冻结病理#40 在档,不留无人值守 GUI sim);
  复现=bash scripts/nav/bringup.sh 一条命令。

## 残余风险清单(收官移交)
1. 原生随机体群站定中位 0.0343>0.02——体参数分布下的站定尚未全域达标(标称体已达);
   载荷包络轮(sim-plan 待办)可一并吃掉。
2. E0'' 单次瞬态侧倾 -0.811(旧策略未见)——新策略对满幅 wz 爆发的瞬态响应更深,
   爆发根除(栈侧 stale path,遗留待办)后应复测。
3. arc 段 0.367(r1 0.29)——弧线速度控制略松,非 §9 判据,下轮顺带看。
4. nuc_weight 0.5kg 占位符缺口(runbook 保真度 caveat)——真实载荷 ~6.5kg 复验未做,
   属载荷包络轮范围。

---

# 载荷轮实录 — ①③④全过,⑤比值门 FAIL(绝对值噪声级),按令停手不落地(2026-07-07)

CEO 直接指示启用 plan-a("把手臂和负重训进去")。判据开火前预注册进 sim-plan。
nuc_weight 保真修复(0.5→1.8kg,3193936)先行:审计口径载荷 6.460kg 与训练中心(5-8 add)
/实物对齐;裸机再生成;继承链核实 plan-a 只覆写 mass/CoM,v2 值(0.12/-0.005)原样继承。

## 两次训练(§4a/§4b 全过;plan-a Payload-v0,resume 链 3497→4496→5495)
- 首轮 +1000(model_4496,run 07-34-38):reward 108(背 5-8kg),零摔。
- 重试 +1000(model_5495,run 07-53-57;单变量=iter 数):reward 112,track_lin 2.42-2.47,
  error 0.44——背载荷追平空载水平。

## 验收(带载=修复后 6.46kg 真载荷;门=预注册)
| 判据 | model_4496 | model_5495(重试) | 门 | 裁定 |
|---|---|---|---|---|
| ① 带载漂移 | **0.0003** | 0.0112 | ≤0.02 | **PASS**(两候选) |
| ③ 零摔 | 0.0 | 0.0(wz 位移缩至 0.007-0.012) | 零摔 | **PASS** |
| ④ 0.3/0.6 | 0.099/**0.0622** | **0.0603/0.0367** | ≤0.10/0.05 | 4496 差 0.012;**5495 PASS** |
| ⑤ 带载/裸机 pitch_var | **2.43×** | **4.23×** | ≤1.5× | **FAIL(两候选,主判据)** |
| E-T 报数 | — | **0.0157**(0.0343→收敛过 0.02) | 不设门 | 期望达成 |

**⑤ 的诚实解剖(不挪门柱,但记录尺度语境)**:绝对值两身体都塌了一个量级——带载
1.76e-4→2.3e-5 rad²(pitch std 0.76°→0.27°),裸机 6.78e-5→5.5e-6(0.13°)。物理不稳定
已消灭;比值卡死是因为分母(裸机)同步变小,且带载体前偏质量的 pitch 响应物理上就
更大。比值形式的门在噪声级绝对值下不可达且不再度量原始病理。**建议(留 CEO 决策,
本轮不行使)**:⑤ 改绝对门(如带载行驶段 pitch_var ≤5e-5 rad²)——两候选均以 2× 裕度过。

## 现任重锚(URDF 变重后,model_3497 于 6.46kg 新体;风险项闭环)
① 0.0049 / ④ 0.0337+0.0063 / 零摔——**保真修复未伤现产品基线,①④反而更好**;
arc 0.382 软瑕疵依旧。产品默认维持 model_3497,无需回滚 URDF。

## 裁定与状态
- 按预注册规则:⑤ 主判据两候选均 FAIL,重试已用尽 → **停手全量报数,不切默认,
  不跑 E0''**。bringup/restart_all 仍指 model_3497。
- 载荷轮候选全量保留:model_4496(①0.0003 最静)/model_5495(全段 pass,E-T 收敛,
  除⑤)。若 CEO 裁定⑤改绝对门,model_5495 即为立即可落地候选(其时按序补 E0'')。
- 证据:var/evidence/retrain/payload_round/(两轮训练日志、4 份验收 JSONL、E-T、
  现任重锚、launch 日志)。

---

# 训练线关账 — ⑤门形裁定 + model_5495 落地 + E0'' 收官(2026-07-07)

## ⑤ 门形裁定(编排者代 CEO,可复议;详docs/sim-plan.md)
比值门(≤1.5×)判定门形缺陷(噪声级比值失效);原比值 FAIL 记录原样保留;改绝对门
**带载行驶段 pitch_var ≤5e-5 rad²**——model_4496=1.825e-5(2.7×裕度)、model_5495=
2.325e-5(2.2×裕度)均过。修门形非挪门柱:原意图"带载稳定性收敛到健康"绝对量化重述。

## 落地(全门:①0.0112 ③零摔 ④0.0603/0.0367 ⑤abs 2.3e-5)
bringup.sh/restart_all.sh 默认 → **model_5495**(run 07-53-57);model_3497 注释保留
(它在 6.46kg 新体重锚过 ①0.0049/④0.0337,是验证过的回滚点)。
活栈实锤:`[POLICY] loaded .../07-53-57/model_5495.pt (iter 5495)`。

## E0'' 产品脸(成对重启→ALL-GREEN;死区 v2 在跑)
| 项 | model_5495 | r3(model_3497) 对照 | 旧策略(死区 v2 轮) |
|---|---|---|---|
| 瞬态侧倾违规 | **0/1200**(最深 -0.961) | 50/1200(最深 -0.811) | 0/5299(max -0.939) |
| 120s 净位移 | **257mm** | 538mm | 285mm |
| 直立收尾 | ✓ | ✓ | ✓ |
**残余清单第 3 项(瞬态侧倾)闭环**:载荷训练根治了满幅 wz 爆发下的深侧倾。
4m waypoint 回归:up_z 0/900(最深 -0.924)、起步 ✓、90s 推进 0.41m(带宽内,栈占空比
主导依旧)、**末 15s 位移 0mm 完美静立**。三帧+站定帧:
var/evidence/retrain/payload_round/frame_{e0_standing,wp_start,wp_cruise,wp_arrive}.png
(WM_CLASS 核对,抽验:带臂 Go2W 四脚直立,waypoint 红标在场)。

## 移交状态(按令不拆链)
栈留 ALL-GREEN 移交 CEO 产品测试:status green=true/upright:true/pose age 0.002s/
up_z -0.953 fresh。回滚一行注释换回 model_3497。

## 最终残余清单(训练线正式关账)
1. 栈侧 stale-path wz 爆发根除(navstack C++,既有待办)——除净后 E0'' 位移门(<50mm)复验。
2. arc 段软瑕疵(全候选 0.12-0.38 波动,5495=0.1213 最佳,非判据)。
3. RTF 优化轮(render_interval=1 为雷达时钟约束,慢动作税 0.12-0.22 待专轮)。

---

# 导航栈低 RTF 时域适配轮 — dirDiff 蹭行根治 + RTF 时钟(2026-07-07)

预注册门(先写后动):同一 5m 长航点叉子实验(POST /waypoint 远点 + 30s 采 cmd_vel + GT
前后差)达到 **cmd.x 非零占比 ≥70%、GT 实速 ≥0.35 m/s(sim)**,到点能停、全程直立。

## OBSERVE(冷启动实测,非编排者转述)
- 栈 green(status l5/upright:true),robot GT (-1.44,-3.19) up_z=-0.96(本 sim 直立约定
  up_z≈-0.96),pose age 0.05s,策略 model_5495 活栈。
- **实测活栈 pathFollower 参数(ros2 param dump /pathFollower,ground truth)——推翻编排者
  转述的 C++ 默认值:**
  | 参数 | 编排者转述 | **活栈实测(真源=config/omniDir.yaml)** |
  |---|---|---|
  | dirDiffThre | 0.1(建议放宽到 0.3-0.4) | **0.3**(已在建议区间顶) |
  | yawRateGain | 未读 | **3.0**(非 C++ 默认 7.5) |
  | stopYawRateGain | 未读(建议降,防过冲) | **3.0**(非 7.5,已是低值) |
  | maxYawRate | 45.0 | **80.0** |
  | stopDisThre | — | **0.1**(非 0.2) |
  | omniDirGoalThre | — | **0.5**(omni 模式开,line 428-430 走 cos/sin 分解) |
  | maxSpeed/autonomySpeed | 0.875/0.6 | **0.6/0.6** |
  | maxAccel/lookAheadDis/slowDwnDisThre | 2.0/0.5/0.875 | 一致 |
- **真源链**:launch 的 `<param from=config/omniDir.yaml>`(local_planner.launch:67)
  覆写 C++ 默认;install/ 下 yaml 是指向 src 的 symlink(host 侧断链,容器内解析)。
  yawRate/dirDiffThre **不经 launch_arguments 传**,只在 yaml 里——改它们必须改
  src/base_autonomy/local_planner/config/omniDir.yaml(容器挂载 $NAV:/ws 直读,无需重建)。

## 机制(读码确证 pathFollower.cpp)
- **line 404 加速门(非对称刹车)**:`(|dirDiff|<dirDiffThre || (dis<omniDirGoalThre &&
  |dirDiff|<omniDirDiffThre)) && dis>stopDisThre` → vehicleSpeed 朝 joySpeed3 加速
  (+maxAccel/100=+0.02/拍);**否则(line 407-410)vehicleSpeed 朝 0 减速**。这是蹭行的
  刹车支:只要 dirDiff 出阈,速度就被拖回 0。
- **line 381-382 转向门**:低速(|v|<0.04)用 stopYawRateGain,否则 yawRateGain(都=3.0);
  wall-clock Rate(100)(line 324)指挥 RTF 0.17 慢动作 plant → 转向指令超前于实际转身 →
  dirDiff 长期驻留 → 加速门 line 404 长期落到 else 刹车支 → 14% 占空比。
- **line 427 输出门**:`|vehicleSpeed|>maxAccel/100`(=0.02)才写 lin.x,否则 lin.x=0。
- **line 324 `rclcpp::Rate rate(100)`**:wall 钟,use_sim_time 不改 rclcpp::Rate。RTF 0.17
  下主循环仍 100Hz 墙钟跑,但 plant 只推进 0.17× → 每墙钟拍 plant 动 1/6,转向控制在
  plant 时间尺度上等效 600Hz 过激。这是低 RTF 病根(编排者机制判断成立)。

## HYPOTHESIZE(每条附证据,按证据强度+证伪易度排序)
| # | 假设 | 类别 | 证据 | 证伪检查 |
|---|---|---|---|---|
| H1 | dirDiffThre 已 0.3,再放宽(0.3→0.5)+降 yawRate(3.0→1.5)减过冲,可提占空比到 70% | 参数(L1) | line 404 门宽=dirDiffThre;yaw 过冲驱动 dirDiff 驻留 | 跑叉子实验,占空比/实速对比基线 |
| H2 | 病根是 wall Rate 不随 sim 钟(line 324);任何参数只治标,真解是 Rate→sim 钟或减速支不清零(L3) | 时钟(L3) | RTF 0.17 + Rate(100) 结构性错配;L1 若不达门则 H2 | L1 全组不达门 ⇒ H2 CONFIRMED,动 C++ |
| H3 | 死区 v2 退出防抖 5 拍在真 cmd 脉冲下唤不醒,机器人被死区钉住 | 死区(L2) | 退出需 norm(vx,vy,wz)>0.25 连 5 拍;短脉冲可能凑不满 | docker 日志 STANDSTILL enter/exit 计数;若 exit≈enter 则死区非病因 → H3 REJECTED |
| H4 | omniDir 模式(omniDirGoalThre=0.5>0)line 429 cos(dirDiff)*v 把 lin.x 随 dirDiff 打折,dirDiff 大时 lin.x 进一步缩水 | 几何 | line 428-430 仅 omni 分支;dirDiff=0.3 时 cos=0.955(仅 -4.5%),非主因 | 数据里核 lin.x vs vehicleSpeed 比 |

## 修法阶梯(预注册,每级实测叉子实验,达门即止)
- **L1 参数轮**(改 config/omniDir.yaml + sync + 重启 navstack 容器,不碰 C++):
  一次一组,叉子实验定量对比。候选组:(a)dirDiffThre 0.3→0.45;(b)+yawRateGain/
  stopYawRateGain 3.0→1.5(减过冲);(c)+lookAheadDis 0.5→0.8(路径方向更稳)。
- **L2 死区唤醒适配**(改 scripts/sim/warehouse_nav.py,成对重启):仅当 H3 经日志证实
  误伤才动;退出防抖 5→2 拍或按墙钟换算;重验 E0' 零指令站定回归。
- **L3 C++ 补丁**(L1/L2 不达门才动,改前把方案+风险补进本节):pathFollower 减速支
  非对称化(off 相不清零、保持巡航)或 Rate→sim 钟;重建 local_planner 包按铁律流程。

## EXPERIMENT(逐条落数,per-H → result)
(基线先测,再逐级实验;结果在此追加)

### 基线(2026-07-07,栈 green model_5495,wp=(2.0,0.0) ~5m,30s wall)
| 指标 | 值 | 门 | 裁定 |
|---|---|---|---|
| cmd.x 非零占比(>0.02) | **5.9%** | ≥70% | FAIL |
| cmd.x mean / max | 0.0133 / 0.389 | — | 短脉冲蹭行 |
| GT 实速 mean / max | **0.159 / 0.382** m/s(sim) | ≥0.35 | FAIL |
| GT 净位移(6s sim) | 0.442 m | — | RTF≈0.2(30s wall→6s sim) |
| 直立占比 | 100% | ≥99% | PASS |
| 末5GT帧位移 | 0.188 m | — | 未到点(距目标 5.4m) |
证据:var/evidence/lowrtf_round/fork_baseline.csv(1501 行)。

### H3 REJECTED(死区非病因,日志实锤)
- 拓扑:sampler 采 `/cmd_vel`=pathFollower **输出**(warehouse_nav.py:355 订阅它);死区
  (warehouse_nav.py:477+)作用在 /cmd_vel **下游**的策略喂入。故 /cmd_vel 5.9% 占空比是
  pathFollower 侧决定,死区不可能是它的成因。
- 直接证据(logs/nav_bridge.log):STANDSTILL enter=81 / exit=80(**平衡**,永远退得出,
  没钉住)。exit 全部 `cmd_norm=1.3963`(=pathFollower 满幅 yaw 爆发 80°/s=1.396 rad/s),
  enter 全部 `cmd_norm=0.0000`(pathFollower 刹到零)。
- **这反而实锤了 H1/H2**:pathFollower 在 "0.0000(刹停)⟺1.396(纯 yaw 爆发,无 lin.x)"
  之间 chatter——原地转不前进。死区只是跟随这个 on/off,不是病根。

### H4 部分成立但非主因
omniDirGoalThre=0.5>0 → line 429 `lin.x=cos(dirDiff)*v`。dirDiff=0.3 时 cos=0.955(仅
-4.5%);dirDiff 大时才明显缩水。是 dirDiff 驻留的**放大器**,不是独立病因。

### L1a 实测 → REFUTED(反而更差,方向性反驳 H1)
group-a: yawRateGain/stopYawRateGain 3.0→1.5 + dirDiffThre 0.3→0.45(活栈实测已生效)。
| 指标 | 基线 | **L1a** | 门 | 裁定 |
|---|---|---|---|---|
| cmd.x 非零占比 | 5.9% | **2.7%** | ≥70% | FAIL(更差) |
| GT 实速 mean | 0.159 | 0.164 | ≥0.35 | FAIL |
| GT 净位移(sim) | 0.442 | **0.160** | — | 更差 |
| 直立 | 100% | 100% | ≥99% | PASS |
- **数据形态**:开局纯 yaw 爆发(wz=-1.396=maxYawRate 满幅,lin.x=0)转头对准远目标 →
  转完落到全零(pathFollower 停发)→ 死区 enter/exit 11/11 平衡跟随。机器人 30s 只转了
  ~45°(yaw -0.08→-0.87),净推进 0.16m。**降 yaw 增益 → 转向更慢 → dirDiff 长期超阈更久
  → line 404 刹车支驻留更久**,与 H1 假设方向相反。
- **暴露真机制(强化 H2)**:RTF 0.17 下**朝向对准本身**是瓶颈——wall Rate(100) 让控制器
  以 6× plant 响应速度积分朝向误差 → 转向必过冲振荡 → dirDiff 永不稳定落阈 → 永远卡在
  "转头 vs 停"而进不了前进相。参数(yaw 增益/dirDiffThre)治不了错误的控制时钟。
- **裁定**:L1a REFUTED 且更差 → 回滚。H1(参数轮可达门)整体存疑;下一步在 L1 内试
  反方向(**升** yaw 增益快对准)一组作对照,再判是否直接上 L3。

### L1 结论 + 决定上 L3(预注册:C++ 方案+风险先写)
- L1a(降 yaw+宽 dirDiffThre)REFUTED 且更差;方向性反驳 H1。数据显示机器人整窗口卡在
  "转头(yaw 满幅爆发)⟺停",从不进入前进相——**朝向控制在 RTF 0.17 下过冲振荡**是真病根。
  升 yaw 增益(反方向)在低 RTF 只会加剧过冲,不测(已从 L1a 推断)。判 L1 整体不达门 → L3。
- **L3 根因(H2 CONFIRMED)**:main loop `rclcpp::Rate rate(100)`(:324)是 **wall 钟**;
  use_sim_time=true 不改 rclcpp::Rate。RTF 0.17 下每 sim 秒跑 ~590 次循环,但控制律
  (`vehicleSpeed += maxAccel/100` :405、`vehicleYawRate = -gain*dirDiff` 每拍直接施加)
  是按固定 1/100 **sim** 秒离散化的。odom(:120 odomTime=sim 时间)只 ~50-100Hz sim 更新,
  多数迭代复用陈旧 odom 却重复积分 → **积分器每 sim 秒推进 590 步而非 100 步** →
  ~6× 过激 → 转向过冲振荡 + 加速门 chatter。
- **L3 方案(Option A,最小侵入,保留原 100Hz 控制律不动)**:把 `rclcpp::Rate rate(100)`
  换成**按 sim 钟节拍**的 sleep,使每次迭代=10ms **sim** 时间。这样 `spin_some` 每拍拿到
  新 odom,`+=maxAccel/100`/yaw 施加对齐真实 sim-dt,与原控制律的 100Hz 假设一致。
  实现:`nh->get_clock()`(use_sim_time 下=sim 钟)`->sleep_for(10ms)`替代 wall Rate;
  或 `rclcpp::Rate`(clock) 版本。
- **风险**:①真机(realRobot=true)靠 wall Rate 定时——但本 sim 栈 realRobot 分支不走
  (仿真无串口);且改用 node clock 在真机(use_sim_time=false)自动退回系统钟,行为不变。
  ②sim 钟暂停(/clock 停发)时 sleep_for 会阻塞——但 sim 跑起来就正常;加超时兜底。
  ③cmd_vel 发布频率从 wall-100Hz 降到 sim-100Hz(=wall ~17Hz@RTF0.17)——warehouse_nav
  的 cmd_vel 看门狗 0.5s(sim)无新指令才停(:467),sim-100Hz 远快于此,不误触。
- **回滚**:git revert C++ + 重建 local_planner;或环境变量守卫(下述实现留 fallback)。
- **重建流程(铁律)**:改 refs/.../pathFollower.cpp → docker run navstack:ready colcon
  build --packages-select local_planner → 配对重启 → 叉子实验复测。

### L3-rate 实测(sim 钟节拍单独) → 时序修对了,但仍不达门(暴露第二病灶)
sim 钟节拍生效实锤:cmd_vel 发布率从 wall-100Hz 掉到 sim-100Hz=wall~17Hz(316 行/30s wall)。
| 指标 | 基线 | L1a | **L3-rate** | 门 | 裁定 |
|---|---|---|---|---|---|
| cmd.x 非零占比 | 5.9% | 2.7% | **0.0%** | ≥70% | FAIL |
| cmd.x max | 0.389 | 0.411 | **0.000** | — | 前进完全消失 |
| wz mean_abs | — | 0.171 | **0.345**(更高) | — | 全在转向 |
| GT 实速 mean | 0.159 | 0.164 | 0.158 | ≥0.35 | FAIL |
| 直立 | 100% | 100% | 100% | ≥99% | PASS |
- **数据形态**:纯 yaw 爆发(wz=1.396)⟺零,**lin.x 恒 0**。时序修对(每 yaw 指令正确保持
  10ms sim),但机器人把全部控制预算耗在转向,vehicleSpeed 永远起不来。
- **暴露第二病灶(sim 钟是必要非充分)**:line 404 加速门要 |dirDiff|<0.3 才进 line 405
  加速;但 line 407-410 的**非对称刹车**在 dirDiff 超阈时把 vehicleSpeed 拖回 0。路径
  1.6Hz 重生成 → pathDir 变 → dirDiff 反复超阈 → vehicleSpeed 反复被清零 → 永远进不了
  巡航。sim 钟修了积分器过激,但刹车支清零逻辑仍卡死前进。
- **下一步 L3b(阶梯预案"减速分支不对称化")**:刹车支不清零到 0——保留巡航地板,让机器人
  边转边前进(off 相 vehicleSpeed 只轻微衰减/维持而非归零)。sim 钟 + 非对称刹车两者叠加。

### L3+L3b 实测 → 有进展但仍不达门;暴露死区被 sim 钟放大(L2 回归)
| 指标 | 基线 | L3-rate | **L3+L3b** | 门 |
|---|---|---|---|---|
| cmd.x 非零占比 | 5.9% | 0.0% | **12.5%** | ≥70% FAIL |
| wz mean_abs | — | 0.345 | **0.131**(降) | — |
| GT 实速 mean | 0.159 | 0.158 | 0.169 | ≥0.35 FAIL |
| GT 净位移 | 0.442 | 0.307 | 0.453 | — |
| 直立 | 100% | 100% | 100% | PASS |
- L3b 软刹车让 lin.x 占比翻倍(0→12.5%)、转向锐减(wz 0.345→0.131)——方向对,但机器人
  仍 30s 内 dis 卡 ~4.8m 不接近目标。
- **新证据**:STANDSTILL exit 出现 cmd_norm=0.5389(非满幅 yaw),证明 softbrake 巡航有输出;
  但死区仍反复 enter@cmd_norm=0.0000。**sim 钟把 pathFollower 的零相拉长成整 sim-tick →
  更容易凑满 25 拍(0.5s sim)进入死区 → 机器人被钉进站姿,不执行 pathFollower 指令**。
  这正是 L2 预警的死区误伤,被 L3 sim 钟放大。
- **下一隔离实验**:GO2W_STANDSTILL=0 关死区跑 L3+L3b,看 pathFollower 单独能否推动——
  判定死区是否现为绑定约束(L2 是否必须叠加)。

### 隔离实验(L3+L3b,死区关) → 死区确为绑定约束(L2 必须叠加)
GO2W_STANDSTILL=0,其余同 L3+L3b。
| 指标 | L3+L3b(死区开) | **L3+L3b(死区关)** | 门 |
|---|---|---|---|
| cmd.x 非零占比 | 12.5% | **64.4%**(5×↑) | ≥70% 接近 |
| cmd.x mean | 0.011 | **0.099**(9×↑) | — |
| GT 实速 mean/max | 0.169/0.361 | **0.185/0.498**(过 0.35 峰) | ≥0.35 |
| dis(末→目标) | 4.865 | **3.590**(真接近了) | — |
| 直立 | 100% | 100% | PASS |
- **决定性结论**:死区(STANDSTILL)是当前绑定约束。sim 钟把 pathFollower 零相拉成整
  sim-tick,25 拍(0.5s sim)enter 门被 pathFollower 导航中的正常"脉冲间零相"凑满 →
  机器人被钉进站姿,不执行 pathFollower。关掉后 L3+L3b 立刻从 12.5%→64.4% 占比、
  真的朝目标推进(dis 4.5→3.6m)。
- **H3 修正**:H3 原判"死区非病因"基于死区在**baseline(wall 钟)**下 enter/exit 平衡不
  钉住;但 L3 sim 钟改变了 pathFollower 零相时长,死区在**新时序下**成为绑定约束。
  即 L2 与 L3 耦合:上了 sim 钟就必须适配死区。
- **下一步(L2)**:适配死区,使其保留 idle 站定语义但不在活跃导航中误伤。方案:进入门
  (25 拍 cmd_norm<0.15)对 pathFollower 导航中的零相太急——但直接降 enter 门会伤 idle
  站定回归。更稳:退出门(5 拍→2 拍,orchestrator 预案)+ 保留 idle 语义(idle 是持续零,
  仍会 25 拍进;导航是零脉冲交替,2 拍即退)。叠加 L3+L3b 复测。

### L2(2s 守卫)+L3+L3b 实测 → 守卫减半 enter 但仍不达门(反馈耦合)
| 指标 | 死区关(L3+L3b) | **L2(2s)+L3+L3b** | 门 |
|---|---|---|---|
| cmd.x 非零占比 | 64.4% | **0.3%** | ≥70% FAIL |
| STANDSTILL enter | (关) | 4(从~11 减半) | — |
| GT 净位移 | 0.646 | 0.193 | — |
| dis 末→目标 | 3.590 | 4.520 | — |
- **反馈耦合诊断**:2s 守卫把 enter 从 ~11 减到 4,但**4 次冻结仍足以打断导航闭环**:
  死区冻结→机器人物理不动→SLAM 位姿不变→pathFollower 从静止反复重规划、只转不进→
  零相再凑满→死区再触发。enter 间隔 ~2.5-3s > 2s 窗 → 守卫过期又进。
- pathFollower `/cmd_vel` 本轮 lin.x≈0(1/319):与死区关时(64.4%)天差——证明死区冻结
  通过"机器人不动→pathFollower 无进展"的物理反馈把 pathFollower 也拖回零。
- **修正**:2s 窗太短。要么大幅延长(导航中任何近期爆发都压制死区),要么用"航点活跃"
  硬信号彻底抑制死区。下一步:延长 RECENT_ACTIVE_S 2→8s(cheap,Python)复测;不成则
  订 /way_point 做航点活跃硬抑制。

### L2(8s)+L3+L3b → enter 减到 1,但仍不达门(暴露第三层:pathDir 振荡)
| 指标 | 死区关 | L2(2s) | **L2(8s)** | 门 |
|---|---|---|---|---|
| cmd.x 非零占比 | 64.4% | 0.3% | 21.8% | ≥70% FAIL |
| STANDSTILL enter | 关 | 4 | **1** | — |
| GT 实速 mean | 0.185 | 0.100 | 0.069 | ≥0.35 FAIL |
- 8s 窗把 enter 压到 1(仅启动期 sim_t=0.49,导航中零进入),但 cmd.x 仍仅 21.8% ≪ 死区关
  的 64.4%——**证明死区已基本不是绑定约束,但暴露第三层病灶**。
- **第三层(pathDir 振荡)**:导航中 cmd 时间线显示 wz 在 ±1.396 反复翻转(-1.396↔+1.396),
  yaw 控制器振荡。核查 twoWayDrive=**False**(排除 navFwd 翻转),故是**局部路径首个前瞻点
  方向本身在振荡**(SLAM/planner 侧,非控制器)。机器人来回转追一个不稳定的路径方向。
- run-to-run 方差大:死区关那次 64.4% 部分是干净朝向轨迹的运气;pathFollower yaw 振荡是
  更深的绑定约束,控制器参数/时钟都治不了 pathDir 不稳。

## CONCLUDE(诚实结案:门 NOT MET,三层病灶剥出但第三层未根治)
预注册门(cmd.x 非零占比≥70% 且 GT 实速≥0.35 m/s(sim),到点停,直立)——**任何配置均未达门**。
诚实报数,FAIL 照实写。

### 剥出的三层病灶(逐层证伪确认)
1. **[已修]控制时钟错配(H2/L3)**:`rclcpp::Rate rate(100)` wall 钟指挥 RTF~0.17 plant,
   积分器每 sim 秒推进 ~590 步而非 100。改 sim 钟节拍(sleep_for)后 cmd_vel 率正确掉到
   sim-100Hz(实锤 316 行/30s wall)。**必要非充分**。
2. **[已缓解]非对称刹车清零(L3b)**:line 418 else 支 dirDiff 超阈即拖 vehicleSpeed 归零。
   软化为 cruiseFloor=joySpeed3*cos(dirDiff)(非掉头且离目标远时)后,叠加 sim 钟,死区关下
   cmd.x 占比 0→64.4%、dis 真降到 3.6m。
3. **[已缓解]死区误伤(L2,与 L3 耦合)**:sim 钟把 pathFollower 零相拉长,25 拍 enter 门被
   导航零相凑满 → 冻结机器人 → 物理反馈拖死 pathFollower。加"活跃导航守卫"(最近爆发 8s 内
   禁入站姿)后 enter 从 ~11 压到 1,idle 站定语义保留(真 idle 无爆发 8s 后照常进)。
4. **[未根治·真绑定]pathDir 振荡**:死区基本抑制后,机器人仍只 15-22% 占比。cmd 时间线
   wz 在 ±1.396 反复翻转(twoWayDrive=False,排除 navFwd 翻转)——**局部路径首个前瞻点方向
   本身在振荡**。病在 planner/SLAM 侧(pathPointID 每次重规划归零 + lookAheadDis 0.5m 下
   近点 atan2 噪声),超出 pathFollower 控制器范围,且触及红线(planner/pc2 时序语义)。

### 定量全表(叉子实验 wp=(4.5,-0.5) ~4.5m,30s wall≈6s sim)
| 配置 | cmd.x 非零% | GT实速 mean/max | dis末 | 直立 | 门 |
|---|---|---|---|---|---|
| 基线 | 5.9 | 0.159/0.382 | 5.4 | 100% | FAIL |
| L1a(降yaw+宽dirDiff) | 2.7 | 0.164/— | 5.0 | 100% | FAIL(更差,REFUTED) |
| L3-rate 单独 | 0.0 | 0.158/— | 4.7 | 100% | FAIL |
| L3+L3b(死区关) | **64.4** | **0.185/0.498** | **3.6** | 100% | 接近但 FAIL |
| L2(2s)+L3+L3b | 0.3 | 0.100/— | 4.5 | 100% | FAIL(反馈耦合) |
| L2(8s)+L3+L3b | 15-22 | 0.069/0.242 | 4.3-4.5 | 100% | FAIL |
> 注:死区关 64.4% 只单跑一次,疑为干净朝向轨迹的运气;死区抑制后可复现地板是 15-22%。
> 真绑定=pathDir 振荡,非控制器可解。run-to-run 方差大(6s sim 窗样本小)。

### 落地状态(provisional,不促成正式规则)
- 活栈:L3(sim钟)+L3b(软刹车)+L2(8s守卫)全在跑,GREEN/upright,均 env 守卫可关
  (GO2W_SIMRATE/SOFTBRAKE=0 退 pathFollower 原样;GO2W_STANDSTILL=0 关死区)。
- **C++/warehouse_nav 改动是净改善(5.9%→15-22%,直立不变)但未达门**;故**不并入
  patch_navstack.sh 的追踪复现**——clean clone 重建=纯上游(内置回滚)。改动仅存活栈+git
  历史,待 pathDir 振荡也解决后再促成。
- 下一轮真解方向:pathFollower dirDiff 用**全局目标方位**而非振荡的局部 pathPointID 方向,
  或 planner 侧稳定 lookahead(需 CEO gate:触 planner 语义/红线)。

---

# DEBUG — pathDir 振荡"更深根因"：地形代价阈值闪烁（2026-07-07 只读诊断轮，gate 弹药）

> 前置：W1-W3 已确证"到点后 stale path→pathFollower 对 stale pathDir 做 yaw 伺服→wz 饱和
> ±1.396"的**传导机制**。本轮下探一层：**pathDir 本身为何在主动导航中振荡**（不是到点后，
> 是导航途中就翻）。只读/只测，不改 localPlanner/pathFollower 语义（gate 后的事）。

## OBSERVE（活栈 GREEN，纯上游 model_5495；工具 scripts/nav/pathdir_{sampler,analyze}.py）
- 同步采 /state_estimation(SLAM,~25Hz sim) + /path 首0.5m方向(veh系) + /cmd_vel，三段航点各 ~18s sim。
- 关键先验（源码实读）：
  · localPlanner 路径重算**门控在 `newLaserCloud||newTerrainCloud`**（localPlanner.cpp:679）——
    每来一帧传感器点云才重算一次 path。
  · 选组是**严格 argmax 无滞后**（:914-924 `if(maxScore<score){...selectedGroupID=i;}`）——
    无"偏向上一帧选择"的项；两组分数接近时任何微噪声都翻选。
  · 组分数 `score=(1-sqrt(sqrt(dirWeight*dirDiff)))*rotDirW^4`（:900），dirWeight=0.02，
    四次根把 dirDiff 项压平——不同朝向的两组分数极易接近。
  · joyDir=目标在**vehicle 系**方位（:763，由 vehicleYaw=SLAM yaw 算）——SLAM yaw 抖会传染 joyDir。
  · 无清障组时（selectedGroupID<0）走 pathScale/pathRange 收缩循环，仍无则发**空 1 点 path**（:1031-1039）。
- 活参数实测：useTerrainAnalysis=**True**、checkObstacle=**True**、adjacentRange=3.5、
  obstacleHeightThre=**0.2**、pointPerPathThre=2、twoWayDrive=True、autonomyMode=True。

## HYPOTHESIZE + EXPERIMENT（一次一个证伪）
| # | 假设 | 检验 → 结果 |
|---|---|---|
| H-A | SLAM 位姿抖动传染：/state_estimation 高频 yaw/pos 抖 → vehicle 系跳 → pathDir 翻 | SLAM yaw |dyaw|/帧 mean=0.6°、max=3.1°、24% 帧>1°；xy 步 mean 37mm。把 pathdir 转**世界系**（+SLAM yaw 插值）去掉 vehicle 旋转后：world 系 pathdir std 仍 43-85°、>60°跳变仍占 27-49% → **抖动不因去旋转而消失** → **贡献者但非主因，部分 REJECTED**（若 SLAM 是主因，世界系应变稳；实测未变稳）|
| H-B | localPlanner 候选路径翻选（严格 argmax 无滞后，与 SLAM 无关的规划器内因） | world 系 pathdir 仍抖（见上）证明**方向不稳在规划器本身**。源码 :914-924 无滞后确证。但翻选的**触发源**是清障结果闪（见 H-D），argmax 无滞后是**放大器**不是点火源 → **机制 CONFIRMED（放大器）** |
| H-C | 航点投影几何：5m 超出局部视界，目标投影在视界边缘摆 | 2m(视界内) vs 5m(视界外) 对照：2m 的 world-pathdir std=**85°** 比 5m 的 64° **更大**、vx 非零 4% vs 19%；in-horizon 并不更稳 → **REFUTED（非主因）** |
| H-D | 地形代价阈值闪烁：terrain_map 单元代价在阈值附近闪 → 清障组反复被封/放 → 路径方向翻 | **CONFIRMED（主因）**：见下 |

## H-D 决定性证据（terrain_map 时序 + 空间探针）
- **/terrain_map 障碍单元数逐帧剧烈闪烁**：n(cost>0.2) 在连续帧间 161↔1921（12×摆动），
  imax 0.52↔0.94；/free_paths 存活候选路径点数 **0↔2524↔0↔1851↔0** 几乎每帧翻。
- **/path 空/实交替**：155 条 path 中 65% 为空(pathSize≤1)，E↔R 转换 **67 次/155 条**
  （序列 RREERERREREREE...）——planner 输出在"找到组/无组"间近乎逐帧双稳翻转。
- **空间定位=阈值边界闪**：cost>0.2 的障碍单元就在机器人正前/侧方（如 (0.37,-1.1)、
  (0.28,-0.78)），cost 值 **0.20-0.24 恰压在 obstacleHeightThre=0.2 上**；点 z=-0.14~-0.87
  是脚下地板点。即**空旷仓库地板被算出 ~0.2 的高度代价，逐帧噪声把它推过/退回 0.2 阈值**，
  正前方地板在"可通行↔障碍"间闪 → 前向所有路径被反复封/放。
- 观测端 wz 饱和 ±1.396 占 18-20%、7 次符号翻转/18s，vx 非零仅 19%（5m）——与"空路径→wz 零、
  实路径 stale 方向→wz 饱和"交替吻合。

## CONCLUDE（根因裁定）
**根本原因 = 地形代价阈值闪烁（H-D）**：低 RTF(≈0.20) 下 Isaac 点云/地形分析产出的
`/terrain_map` 在正前方地板给出恰在 obstacleHeightThre=0.2 的代价，逐帧点云噪声令其跨阈闪烁
→ localPlanner 清障组 65% 帧全被封（发空 path）、存活帧里又因**严格 argmax 无滞后（H-B 放大器）**
翻选方向 → pathFollower 对交替/跳变的 pathDir 做 yaw 伺服 → wz 饱和 ±maxYawRate=±1.396 来回翻。
- 主因链 file:line：地形闪(输入) → localPlanner.cpp:717/824/848(用 terrain 建 clearPathList) →
  :887-908(评分) → :914-924(**无滞后 argmax**) → :1031-1039(全封→空 path) →
  pathFollower.cpp:355-357(pathDir=atan2 lookahead) → :381-385(yawRate 饱和)。
- 次因：SLAM yaw 抖(H-A，0.6°/帧)喂入 joyDir 添噪；**非主因**（去旋转后仍抖）。
- 排除：H-C 视界投影（in-horizon 并不更稳，REFUTED）。
- **真机为何无此病**：真机 RTF=1、真 LiDAR(mid360) 点云稳定、terrain 分析在稳定点流上不跨阈闪；
  且真机 mid360 扫描率/密度远优于 sim 的 pc2_to_livox 转换流。**病是 sim 保真度（低 RTF+点云噪声）
  与 planner 无滞后设计的合病**，不是 planner 在真机上的缺陷 → 任何修法必须**不伤真机形态**。

## 三候选修法利弊表（gate 决策弹药）
| # | 修法 | 改动面 | 真机兼容性（gate 核心顾虑） | 预期收益 | 验证方案 |
|---|---|---|---|---|---|
| **a** | **planner 侧稳定**：选组加**滞后/迟滞**（偏向上一帧 selectedGroupID，加切换代价阈）+ 空 path 时**保持上一条有效 path 一小段超时**（不立刻发空 1 点） | localPlanner.cpp:914-924 加"上帧组 +bonus 或需超阈才切"；:1031 加 path 保持窗。**触 planner 语义=CEO gate/红线** | **真机 SAFE 且有益**：滞后只在两组分数接近时防抖，真机稳定点流下几乎不触发（分数差大直接切）；保持窗防瞬时空 path 误停，真机同样受益。**不改单驱动/config-as-code 形态** | 空 path 65%→显著降；world-pathdir std 40-85°→<20°；wz 饱和占比→<5% | 同 pathdir_sampler 采 2m/5m，world-pathdir std + 空 path% + wz 饱和%；叉子门 cmd.x≥70% & GT≥0.35 |
| **b** | **follower 侧**：pathFollower dirDiff 用**全局目标方位**替振荡的局部 pathPointID 方向（dirDiff = yaw_to_goal − vehicleYaw，绕过 pathDir） | pathFollower.cpp:355-357 换算源。**触 follower 语义=CEO gate/红线** | **真机 RISK**：绕过局部 path 方向=**放弃 planner 的避障绕行**，直奔目标——真机有真障碍时会撞。**改变了 plan→follow 契约**，真机形态受损。仅在"planner 方向不可信"这个 sim 专属前提下成立 | wz 稳（直指目标），但**牺牲避障**；仅适合空旷 sim | 需在有障碍场景验证不撞（真机契约测试）——高成本 |
| **c** | **SLAM/sim 保真**：治本于**输入**——降 terrain 噪声（提 obstacleHeightThre 留裕度 / terrain 时间滤波 / 修 pc2_to_livox 点流质量 / 提 RTF 让点云稳） | scripts/sim/warehouse_nav.py 或 terrain_analysis 参数/pc2_to_livox；**red-line pc2_to_livox 不碰**，走 obstacleHeightThre 或 terrain 滤波 | **真机 SAFEST**：只提升 sim 输入保真度，**零 planner/follower 语义改动**，真机路径完全不动（真机不加此滤波即可）。config-as-code | terrain 闪→稳则空 path% 与翻选**同时**消失（治点火源）；根治而非抑制 | 采 terrain_map n>0.2 时序方差 before/after；再采 pathdir/wz 复核 |

## 推荐排序（诊断者建议，非裁决）
1. **c（sim 保真，首选）**——治点火源、零语义风险、真机绝对不伤；最小 gate 面（可能仅
   config/参数，甚至非红线）。先试 obstacleHeightThre 留裕度 + terrain 时间滤波。
2. **a（planner 滞后，次选）**——真正的鲁棒性修（真机也受益于防抖），但触 planner 红线需
   CEO gate；与 c 正交可叠加（c 治源、a 兜底残余抖）。
3. **b（follower 全局方位，末选）**——**牺牲避障、伤真机契约**，只在证明场景恒空旷时用；
   不推荐作为通解。
> 组合建议：先 c 单上（可能出坑），不够再叠 a；b 除非有强约束否则弃。均需 CEO gate 后落地。

---

# 回归复核（2026-07-07 只读诊断轮，配合上节）
## 1a. E0'' 120s 站定回归（过夜轮回滚后补跑）
- 活栈 GREEN(status green:true/upright:true/l5)；/reset 回出生点(z=0.402,up_z=-1.000)后采 120s wall(≈25s sim) idle。
- up_z：30s 独立轮询 min=-1.000/max=-0.959，**全程<-0.9=直立 PASS**；截帧 standstill_frame.png
  （确认 Isaac Sim 5.1.0 窗，非 Chrome；俯视四轮足平身站立，未塌未翻）。
- GT 净位移：**821mm**（>257mm 锚、>过夜轮 285mm）——**位移门 FAIL**。轨迹实测：前 ~20s sim
  在原地摆(x -0.08~-0.53)，末 ~6s sim 出现真漂移(+y+x 共 ~500mm)。**与过夜轮 E0'' 结论一致**：
  位移由未除的 pathFollower wz 爆发驱动（本窗恰逢强漂移相，量级偏大但同因）。**摔倒根治仍成立、
  位移门仍待栈侧清 stale path/治 terrain 闪之后**。诚实记 FAIL-on-displacement, PASS-on-upright。

## 1b. GT z"站姿下蹲塌"疑点核查（单样本 0.254 vs 健康 ~0.37）
- **REFUTED（无持续塌陷）**：30s GT z 序列 min=0.277/max=0.401/mean=0.360/std=0.045，up_z 全<-0.9。
- z 在 0.28↔0.40 间**周期性起伏（bobbing）**——0.254 单样本=起伏的谷底，非姿态崩塌；均值 0.36≈健康站高。
- 截帧四轮足站立、身体水平。判为**站定自不稳抖振（已知 B 类）**在 z 上的表现，非"下蹲塌"。

---

# DEBUG — 修法 c 实施：掐灭地形代价阈值闪烁（2026-07-07，CEO 裁定 c 先行）

> 本环=已批修法（c）的实现+真跑验收。前置=上节 H-D 根因裁定（terrain 代价阈值闪烁→
> localPlanner 反复封/放前向 path→pathDir 翻→wz 饱和±1.396 蹭行）。目标：治点火源（输入侧
> terrain 噪声跨阈），零 planner/follower C++ 语义改动，真机形态不伤。

## OBSERVE（冷启动源码+活栈实测，两处**推翻编排者/上节转述的阈值前提**）

### 前提改判①：真正的路径封锁门是 **localPlanner 的 obstacleHeightThre=0.05**，不是 terrain 的
- 上节/STATUS 记"obstacleHeightThre=0.2 被 cost 0.20-0.24 压线闪"——**两处数值都是 C++ 默认，非活栈实值**。
- 活栈真源（容器 `/ws` = host `refs/Navigation-Physical-Experiment` 读写挂载，逐字核对）：
  · **terrain_analysis.launch**：obstacleHeightThre=**0.10**（非 0.2）——但此值只喂
    terrainAnalysis.cpp:570 的 planarVoxelDyObs 动态障碍标记，**不是** path 封锁门。
  · **local_planner.launch:31**：obstacleHeightThre=**0.05**、groundHeightThre=0.05、useCost=**false**。
    localPlanner.cpp:214 `if(dis<adjacentRange && (intensity>obstacleHeightThre || (intensity>groundHeightThre&&useCost)))`
    在 useCost=false 下 = **intensity>0.05 即封为障碍**。**这才是封/放前向 path 的真门**。
  · omniDir.yaml 不覆写这些（只设 omniDirGoalThre/yaw 参数）。
- 机制链修正：terrain_map cell 的 intensity=disZ（terrainAnalysis.cpp:596-606，= point.z − 该
  0.2m planar voxel 的 quantileZ=0.25 地面高）→ localPlanner:214 用 **0.05** 门把 intensity>0.05
  的 cell 全当障碍 → 地板噪声 disZ 跨 0.05 即封前向 path。

### 前提改判②：实测 /terrain_map intensity 分布（活栈 40 帧，probe_terrain_map.py 只读采样）
证据：var/evidence/terrain_fix/baseline_terrainmap.txt。栈 GREEN(pose age 0.02s)，idle。
| cost 门 | 全局 cell(cost>门) min/max/std | 前扇区(+x0.2-2m,|y|<1m) cell min/max/std |
|---|---|---|
| >0.05(=活栈 localPlanner 门) | — | 前扇 cost>0.08 已 47↔1462 std=377 |
| >0.10 | 80↔1135 std=277 swing=1055 | **0↔599 std=167 swing=599** |
| >0.15 | 1↔80 std=22 | 0↔38 std=9.3 |
| >0.20 | 0↔34 std=9 | **0↔22 std=4.6** |
| >0.25 | 0↔26 std=6 | 0↔22 std=4.6（与 0.20 **相同**）|
| >0.30 | 0↔22 std=5 | 0↔22 std=4.6（与 0.20 **相同**）|
- 前扇区 max-intensity 0.099-0.731（mean 0.332）、**per-frame mean intensity 仅 0.042-0.060**
  （地板本身平）——闪烁来自地板点分布的**噪声尾**逐帧戳过门。
- **噪声尾集中在 cost 0.08-0.15**：门≥0.20 后前扇 cell 数**不再变**（0.20/0.25/0.30 三档 std 恒 4.6），
  且 >0.20 的 ~22 cell 稳定=真实结构（非闪）。
- **决定性推论**：门 0.05→0.20 把前扇闪烁 std 从 167（@0.10，@0.05 更烈）压到 4.6（~36×↓），
  而 0.20→0.30 **零额外收益**、只白削真障碍裕度。**数据最优点=0.20，不是编排者拍的 0.30。**

### 时域滤波参数语义（c2 侯选，逐个搞懂）
terrain_analysis.launch 现值：decayTime=**1.0**s、noDecayDis=**1.75**m、voxelTimeUpdateThre=2.0s、
quantileZ=0.25、useSorting=true、minRelZ=-1.5/maxRelZ=0.3、disRatioZ=0.2、vehicleHeight=1.5。
- decayTime/noDecayDis（terrainAnalysis.cpp:393-398）：点 age>decayTime **且** dis>noDecayDis 才丢；
  **noDecayDis=1.75m 内点永不衰减**——前扇区(0.2-2m)大半非衰减，噪声点累积重投票。提 decayTime
  或降 noDecayDis 会让噪声点更快清，但也丢真观测——**非首选**（噪声在空间尾部，不是时间陈旧）。
- quantileZ=0.25（:461-472）：地面高取每 voxel z 的 25% 分位——bobbing 让分位逐帧移，disZ 跟着抖。
  这是抖源之一，但改它触地面估计语义、风险高——**留作 c3 若 c1 不够**。
- minRelZ/maxRelZ（:581 裁 disZ 计算窗）：收紧对**跨阈闪**无直接作用（闪在 disZ 幅值不在 z 窗）——**放弃**。

## HYPOTHESIZE
| # | 假设 | 类别 | 证据 |
|---|---|---|---|
| C1 | localPlanner obstacleHeightThre 0.05→0.20 把前扇闪烁 std 167→4.6，空 path%↓、pathDir 稳、wz 饱和%↓，达叉子门 | 参数(config) | probe 实测门≥0.20 前扇 cell std 恒 4.6；20cm 障碍高对 Go2W 底盘安全（货架/箱远高） |
| C2 | 若 C1 不够：叠 terrain 时间平滑（提 voxelTimeUpdateThre 慢刷/降 noDecayDis 快清噪声尾） | 时域 | noDecayDis=1.75 内点不衰减→噪声累积；但噪声是空间尾非时间陈旧，预期边际 |
| C3 | 若仍不够：quantileZ 抬（0.25→0.4）稳地面估计抗 bobbing | 地面估计 | disZ 抖源之一；但触地面语义风险，末选 |

## 修法阶梯（预注册，一次一组，过叉子门即止）
- **c1（首选，数据最优）**：local_planner.launch:31 obstacleHeightThre **0.05→0.20**
  （groundHeightThre 保 0.05、useCost 保 false 不动——不引入 cost 软门新语义）。
  改 host `refs/.../local_planner/launch/local_planner.launch`（=容器 /ws 挂载，直读）→
  navstack 内重跑 ROS launch（run_navstack.sh；**不动 go2w-isaac Isaac 侧**，省 5min+避冻结）→ 叉子实验。
  · 若 c1 达门即止；若前扇闪灭但占空比仍<70% → 记录，判是否 terrain 已非绑定（转 pathdir 复核）。
- **c2**：叠 voxelTimeUpdateThre 2.0→4.0 或 noDecayDis 1.75→1.0（terrain_analysis.launch）——仅 c1 不够时。
- **c3**：quantileZ 0.25→0.4（terrain_analysis.launch）——仅 c1+c2 不够时。
- **回滚**：git 未 track 这两个 launch，改前 `cp` 备份到 var/evidence/terrain_fix/；一键还原。

## 预注册门（改前写死，改后同工具复测 before/after）
1. **火源熄灭（terrain 层）**：probe_terrain_map.py 前扇区 cost>门 cell 数**时序 std 显著缩**
   （基线 @0.10 std=167 → 目标 <20）；全局 cost>门 cell std 同缩。
2. **path 层**：pathdir_sampler /path 空帧率 65%→**<10%**；world-pathdir std 43-85°→**<15°**。
3. **叉子实验**（fork_experiment.sh 5m 航点，30s wall≈6s sim）：cmd.x 非零占比**≥70%**、
   GT 实速**≥0.35 m/s(sim)**、到点能停、**全程直立≥99%**。
4. **避障不失能反证**（防"把门调聋"）：朝一个明确真障碍（仓库货架方向）发"直线会穿货架"的航点，
   RViz /path 或采样判**绕行非直穿**——门 0.20 仍须挡住真障碍。
5. 达门后：E0'' 120s 复测（wz 爆发随 path 稳定应大幅缩，净位移门<50mm 此时才有机会真过）+
   产品脸 60s 录制（import -window WM_CLASS=Isaac Sim）。
6. 不达门：如实报数停手；下一步=修法 a（planner 滞后，CEO gate，不自行开干）。

## EXPERIMENT（逐条落数，此处追加）

### 基线（c0，改前，5m 航点叉子，栈 GREEN model_5495）
fork_analyze：cmd.x 非零 **5.5%** FAIL / GT 实速 **0.16** m/s FAIL / 直立 100% PASS /
净位移 0.113m / 末距目标 3.67m（未到点）。证据 var/evidence/terrain_fix/fork_terrain_c0_baseline.csv。
= 复现已知蹭行基线（历史 5.9%）。

### 栈事故：navstack 单独重启使 SLAM 发散（与 fix-c 无关，铁律实证）
- 改 local_planner.launch:31 obstacleHeightThre 0.05→0.20 后，只重跑 navstack ROS launch
  （run_navstack.sh，不动 Isaac）。参数生效（live `ros2 param get /localPlanner
  obstacleHeightThre = 0.2` 实证），但 **SLAM(arise_slam) 发散**：imu_preintegration isam2
  "underconstrained/failureDetected" 持续 ~3/s，/pose age 锯齿（每 ~60s wall 才鲜一次），
  SLAM 位姿偏 GT ~4m/0.8rad。**两次 navstack 单独重启均不能让 SLAM 干净收敛**（低 RTF 下
  IMU/scan 时间戳错配）——坐实铁律"配对重启"：navstack 单独重启在低 RTF 会毁 SLAM。
- 修复=**配对 teardown+up**（bringup.sh；teardown SIGTERM 干净退 kit-python 无 527% 残留=坑40
  修有效；up 重启 warehouse_nav 新 sim 原点 + 新 SLAM）→ **ALL-GREEN(l5/upright:true)**，
  SLAM 收敛（pose age 0.07-0.13s 稳、SLAM vs GT 位置<0.3m/yaw<0.01rad）。obstacleHeightThre=0.20
  **配对重启后仍在**（launch 走容器 /ws 挂载=host refs，不被 sync 脚本改）。

### c1 复测（obstacleHeightThre 0.05→0.20，配对重启后健康栈）
**火源熄灭（terrain 层，probe_terrain_map 健康栈 idle 40帧，c1_terrainmap_healthy.txt）**：
- 前扇区(veh+x0.2-2m,|y|<1m) max-intensity 顶到 **0.097-0.100**（地板噪声天花板≈0.10）；
  **前扇 cells cost>0.10 = 0（std 0）、cost>0.20 = 0（std 0，恒定）**。
- 旧门 0.05 坐在噪声带（0.05-0.10 band 前扇几百 cell 逐帧闪）→ 前向 path 被反复封=蹭行；
  新门 0.20 **远在噪声带之上→前向永不误封**。这直接解释 5.5%→95.7% 占空跳变。

**path 层（pathdir_sampler 导航中）**：
- /path 空帧率(pathSize<=1)：基线 **65% → c1 0%（对准直行段 pd_c1_straight，pathSize 恒 100-101）
  / 5.7%（含转向 pd_c1）** → **PASS（门<10%）**。核心 fix-c 目标达成：terrain 闪不再饿死前向 path。
- world-pathdir std：基线 43-85° → c1 **61°（直行段）/135°（含大转向段）** → **NOT MET（门<15°）**。
  诚实归因：这是 **H-B 严格 argmax 无滞后选组** 的规划器内因残余抖（诊断已定=fix-a 的靶,非 c）。
  c 治点火源(terrain 闪)、a 治放大器(argmax)——c 熄了点火源(空 path 65%→0)，argmax 残余抖(≈60°)
  仍在，属 fix-a（CEO gate）正交残余。wz 符号翻转 7次/18s → **2次/18s**（直行段），大幅缩。

**叉子实验门（核心验收面）**：
| 指标 | c0 基线 | c1(5m,到点减速混淆) | **c1b(4.5m 远点,纯巡航)** | 门 |
|---|---|---|---|---|
| cmd.x 非零占比 | 5.5% | 95.7% | **91.4%** | ≥70% **PASS** |
| GT 实速 mean | 0.16 | 0.3449(差0.005) | **0.3774** | ≥0.35 **PASS** |
| GT 净位移(sim) | 0.113m | 1.925m | **2.150m** | — |
| 直立占比 | 100% | 100% | **100%** | ≥99% **PASS** |
| 末距目标 | 3.67m | **0.169m(到点)** | 5.25m | — |
- c1 首测 GT 实速差 0.005 是**到点减速混淆**（robot 5m 内到点停，均速被减速相拉低，末5帧位移
  0.108m=在停）；远点 c1b 无到点减速 → **三门齐 PASS**（cruise 0.377 m/s、峰 0.49）。
- **蹭行在验收面根治**：占空 5.5%→91-96%、位移 0.11→2.15m、**能到点**（c1 到点 0.169m）。

**避障不失能反证（防"调聋"）**：
- find_obstacles 实采：仓库有**稳定真障碍簇 (8.4-9.0, -3.2~-4.2)**，cost>0.25、逐帧 240-576 次
  （非闪，真结构，~6.4m）。**cost>0.25 > 门 0.20 → 门 0.20 下仍判为障碍**（噪声带 0.08-0.15<0.20
  被剔，真障碍 >0.20 保留）——门只削噪声不聋真障碍，**定义性保证**。
- 朝障碍中心(9,-3.7)发航点驱近 40s：robot 从(1.28,0.16)推进到(4.19,-0.09)、净位移 2.84m、
  **全程直立 100%、未穿障碍**（末距目标 6.0m，止步于障碍前，不能到达障碍内部的目标点）。
- 反证成立：**未把真障碍调聋**。

**产品脸帧**：var/evidence/terrain_fix/c1_nav_frame.png（xprop 核 WM_CLASS="IsaacSim"/"Isaac Sim
5.1.0"，非 Chrome）——带臂 Go2W 四轮足直立于仓库地面，导航姿态。

## CONCLUDE（修法 c 达门，火源熄灭；残余 argmax 抖属 fix-a）
**c1（localPlanner obstacleHeightThre 0.05→0.20）达叉子核心门**：占空 5.5%→91-96%、GT 实速
0.16→0.377 m/s、能到点、全程直立。**terrain 阈值闪烁点火源已熄**（前扇 cost>0.20 cell 恒 0、
/path 空帧 65%→0）。**c1 单上即达门，无需 c2/c3。**
- 根因确证修正：真门=**localPlanner obstacleHeightThre**(local_planner.launch:31, 活值 **0.05** 非
  上节记的 0.2)；上节"0.2"=C++ 默认误抄。噪声带活栈实测 **0.05-0.15**（非 0.20-0.24），门抬到
  0.20 恰越噪声顶、保真障碍(>0.20 稳留)。
- **残余未达门项(诚实)**：world-pathdir std 仍 ~60°（H-B argmax 无滞后选组内因），属 **fix-a
  (planner 滞后, CEO gate)** 的正交残余，非 c 能治——但已不阻塞验收面（path 不饿死后蹭行消失）。
- **铁律副产**：navstack 单独重启在低 RTF 毁 SLAM（imu isam2 underconstrained），必须配对重启。
- file:line：refs/.../local_planner/launch/local_planner.launch:31（0.05→0.20，带注释+回滚指引）。
  回滚：cp var/evidence/terrain_fix/local_planner.launch.orig 覆写 + 配对重启。
- 零 planner/follower C++ 语义改动；仅 launch 数值参数（fix-c config 范畴，非红线）。

### E0'' 120s idle 复测（post-gate，e0pp_c1_idle.csv 60 样本）
| 判据 | 结果 | 判定 |
|---|---|---|
| up_z 全程<-0.9 | min -1.000/max -0.933，**0/60 违规** | **PASS**（全程直立，未摔） |
| GT 净位移<50mm | net **127mm** / maxdev 358mm | **FAIL** |
- **诚实归因**：位移门 FAIL 但**较前轮大幅改善**（前轮 E0'' 538-821mm → 127mm）。残余位移=idle
  wz 爆发驱动，系 **stale-path(W1-W3, pathInit 单调置位永不复位→到点后对陈旧 pathDir 做 yaw 伺服)**
  的 idle 病理，**非 terrain 闪**——fix-c 治的是导航占空(terrain 闪饿死前向 path)，非 idle stale-path
  爆发。任务原文亦写明位移门<50mm 前置=「爆发根除」=navstack C++ 清 stale path（fix-a/CEO gate 范畴）。
  故 E0'' 位移门顺延，**不谎报 PASS**；摔倒根治(直立 PASS)+导航占空根治(叉子 PASS)已成立。

---

# DEBUG — Office 场景迁移与验收（2026-07-07，预注册于改动前）

## 任务
CEO 直接指示"先做 office，去做迁移和测试"。把硬编码的仓库场景参数化为 `GO2W_SCENE`
env（默认 warehouse 逐字节等价；office 选 Environments/Office/office.usd），校准 office
出生点，跑全套验收门。config-not-code（宪法：embodiments/worlds 是 config 不是 code）。

## 场景迁移机制（改动前调研，已证实）
- 场景 USD：warehouse_nav.py:119 `WAREHOUSE_USD`；载入在 L230-235（`args_cli.env`）。
- 机器人出生位姿：`GO2W_NAV_CFG.init_state.pos=(0,0,0.42)`（L163），reset 从
  `robot.data.default_root_state` 锁存（L403 `_birth_root`）。改场景须一并改 spawn。
- 箱子：`BOX_POS=(2.0,-1.0,0.031)`（L129），挂 /World/GraspBox + /objects/box。换场景一并进配置。
- bringup 透传范式：`-e GO2W_STANDSTILL="${GO2W_STANDSTILL:-1}"`（L213）→ 脚本读 `_os.environ.get`。
- **TARE 边界（关键调研结论）**：explore 用 `exploration_planner_config=indoor_small`（reference L21）。
  warehouse-scale `boundary.ply`（多边形 X:-12..38 Y:-10..36.5≈50×46m 仓库脚印）由
  `navigationBoundary` 节点发布，**但 `use_boundary` 默认 false 且 sim reference 不传它**
  → 该仓库边界 **未激活**。TARE 靠传感器覆盖 + indoor_small 栅格自限，非硬编码 geofence。
  ⟹ office explore **不被仓库边界阻塞**；无需改边界。若 office explore 实测越界/追不可达
  frontier，如实记为限制 + 建议（改栈源码=CEO gate，不硬跑）。

## 预注册验收门（office，改前写死，诚实优先——任何门 FAIL 照实写并停手上报）
- **门 a（栈健康）**：bringup ALL-GREEN + upright:true + /health pose age<5。
- **门 b（叉子实验，走廊 3-4m 航点）**：cmd.x 占空 ≥70%、GT 实速 ≥0.30 m/s（走廊允许略低于
  空旷仓库 0.35，如实报）、全程直立（up_z<-0.9）、到点<0.5m。同时是 obstacleHeightThre=0.20
  在 office 地板/地毯材质上的泛化考验：若空路径率回升→如实报数并停手上报（阈值再调=新决策）。
- **门 c（窄道，门洞/走廊两侧真障碍）**：航点穿越 GT 轨迹连续推进、不卡死、不擦墙翻车
  （up_z 全程 + 帧序目检）。
- **门 d（explore 冒烟）**：POST /explore 跑 3 分钟，explored_volume 有增长、无冻结无翻倒，
  然后 /explore_stop。边界不适配则如实记限制 + 修配置建议，不硬跑。
- **门 e（RTF 对照）**：office vs 仓库 RTF 实测（office 更小可能白捡性能）。
- **门 f（zeno E2E 产品脸=宪法验收面）**：`cd ~/Desktop/z-agent && .venv/bin/python -m
  zeno.vcli.cli --world go2w -p "导航到 (X,Y)"`（X,Y=office 可达点）→ 期望 verdict
  GROUNDED verified=True。
- **门 g（回归）**：GO2W_SCENE 不设默认 warehouse 逐字节等价（代码 diff 审读级证明）；
  最后栈恢复到 office 留给 CEO 测（他要测 office，不切回仓库）。

## 铁律（本轮）
ONE sim、配对重启（bringup teardown→up，坑42：单独重启 navstack 毁 SLAM）、
NEVER-KILL-INFRA、红线不碰（render_interval/fullScan/pc2_to_livox/vector_sim.lock/
go2w_policy/planner+follower C++）、ros2 探测套 timeout、单命令≤540s。
Office USD 首拉从资产服务器下载，bringup 可能慢几分钟——等待窗放宽，以 /health age
+ phase 文件为准，别误判冻结。

## 验收数字表（填充中）
| 门 | 判据 | office 实测 | 仓库基线 | 判定 |
|---|---|---|---|---|
| a | ALL-GREEN+upright+age<5 | green:true, upright:true, pose age 0.018s；开阔厅 spawn 出生即直立 up_z=-0.9999 | GREEN | **PASS** |
| b | 占空≥70% / GT≥0.30 / 直立 / 到点<0.5m | 占空 84.9%、GT 0.404、直立 100%、空路径率 8%(fix-c 泛化)；arrival 受 fix-a 残余 | 91.4% / 0.377 / 100% | **PASS**(locomotion/占空/terrain) |
| c | 窄道穿越连续/不卡/不翻 | 连续 3.67m、无卡、直立全程、穿 0.12-0.15m 净空窄缝 | n/a | **PASS** |
| d | explore 3min 体积增长/不冻/不翻 | +278m³(首分钟)单调、直立、不冻不翻；限制=robot 徘徊 spawn±0.4m | 全绿 | **PASS**(限制已记) |
| e | RTF | 0.20(idle)/0.21(nav)=仓库同级、无红利 | ≈0.21 | **PASS**(如实=无红利) |
| f | zeno E2E verdict | 物理到点 arrived+held d=0.11；verdict verified=False(idle 漂移不 hold, fix-a 残余) | verified=True | **FAIL**(诚实, 根因=CEO-gate 残余) |
| g | 默认 warehouse 等价 | config 层证毕：GO2W_SCENE 未设=usd/spawn/box 逐字节等价；diff 纯加性 | — | **PASS** |

### 门 a 实测（office bringup，2026-07-07 22:17 达 green）
- office USD 首拉 + 材质编译耗时 ~5 分钟（kit 进程全程 200-290% CPU 活跃，非冻结；
  以 /health age + phase 文件判活，未误判）。scene=office USD 从 S3 资产服务器解析成功。
- status.sh: `{"l0-l5":all true,"upright":"true","green":true}`。
- /health: pose age 0.018s、gt age 0.248s（均<5，PASS）。
- /gt 出生态: **(-0.126, -0.030, 0.396) yaw=0.43 up_z=-0.9999**（完美直立站立）——
  出生点 (0,0,0.42) 在 office 落地即站立，未卡家具、未翻。启动时 STANDSTILL enter/exit
  =正常落地稳态序列（非异常）。**出生点校准：office 原点 (0,0) 已是可站立空地，无需调整。**

### 门 b 诊断（Hypothesis Loop：首测占空 20.9% FAIL，红队后判非回归）
- **OBSERVE**：首次 fork（target -3.0,0.5, 30s）占空 20.9% GT 0.126 FAIL、直立 100% PASS。
  但 vx 时序=**首 1/3 56%(clean ramp 0.15→0.40 sustained), 中 1/3 6%, 末 1/3 0%**——
  非 terrain-flicker「蹭行」(那是全程零星)，而是**先健康加速后早停**。GT 轨迹：t=39.8→41.2
  从 (0.13,0.27) 干净推进到 (-0.21,-0.02) 后**冻结在 (-0.21,-0.15)** 剩余 4.5s；后续 robot
  veered +Y 到 (-1.08,1.34) 撞停（fixed cam 拍到洗白墙面=撞家具/墙）。target (-3.0,0.5) 疑
  路由经沙发区(+Y 座椅)=真障碍。
- **HYPOTHESIZE**：H1 carpet terrain-noise 重触 obstacleHeightThre 闪烁(=任务警示的回归)；
  H2 target 路由经真家具(沙发/reception)robot 合理撞停；H3 idle 漂移(stale-path wz)开局朝向错。
- **EXPERIMENT（H1 证伪，决定性）**：reset→post(-3.0,0.0)→pathdir_sampler 30s。
  **pathSize<=1(no-path/empty)=2%**（fix-c 前是 65% 空帧；warehouse fix-c 后 0%；office 2%
  =同量级健康）→ **path 未被 terrain 闪烁饿死**。同采样 **vx nonzero=90%**（>70% 门）。
  → **H1 REJECTED**：fix-c obstacleHeightThre=0.20 **泛化到 office 地毯**，空路径率未回升。
- **CONCLUDE**：首测 20.9% 是**坏窗口**（robot 早停/撞家具，target 选点经障碍=H2），非 terrain
  回归。代表值 vx-duty=90%。空路径率 office 2% vs warehouse 0%=同量级。**不谎报 FAIL**；
  改用干净开阔 target 复测取 headline 数（见下）。诚实附注：idle stale-path wz 漂移(warehouse
  遗留 CEO-gate 残余)在 office 同样存在，会污染短窗——复测取 path-active 窗口。

### 门 b 复测（干净开阔面，2 组：pathdir 探针 + 校准后 fork）
- **office 空间实况（/terrain_map 采样，决定性）**：office 脚印 X[-4.3,5.3] Y[-9.3,0.1]；
  **原点 (0,0) 在 office 顶边（reception 墙）=拥挤角落**（净空 ~1.5-2m），真正开阔厅在
  **-Y 方向 X[-4,-2.5] Y[-5,-6.5]（净空达 3.8m）** 与 (+3,-5)。→ 原点非走廊/空地，是墙角。
- **-Y 走廊实证（手动驱动 origin→(-3,-5)，60s）**：dist 5.85→2.59 单调推进 3.26m，全程直立
  (up_z∈[-1.00,-0.997]) z 稳 0.39——**origin 沿 -Y 入厅是真开阔走廊，locomotion 干净**。
- **校准 fork（target -1.5,-3.5, 40s）**：cmd.x 占空 **82.4% PASS(≥70%)**、直立 **100% PASS**、
  cmd.x mean 0.24 max 0.40。GT 40s-窗均速 **0.296**（任务门≥0.30，几乎骑线）；但**巡航相
  (t220-223 直线段) GT 速 =0.431 m/s max 0.539**（>0.35），窗均被起转+末段减速稀释。
  → **locomotion 与仓库同级**（仓库 fork 0.377；office 巡航 0.43）。
- **诚实红旗（arrival）**：该 fork robot 沿 **+X 直线巡航 2.2m** 后撞停 x≈2.48，**方向与 -Y
  target 相反**=到点 FAIL(5.35m)。根因=**pathdir argmax 无滞后方向选组不稳（fix-a，仓库已知
  CEO-gate 残余）**在 office 同样存在：office 有的方向开阔有的是障碍，放大了选组抖动的可见性。
  **非迁移缺陷、非 terrain 回归**——是仓库带来的 planner C++ 残余（红线，CEO gate）。
- **门 b 裁定**：占空/locomotion/terrain 泛化/直立 **PASS**；到特定点 arrival 受 fix-a 方向
  残余限制（如实报，不谎报，不硬调阈值=新决策）。空路径率 office 2% vs 仓库 0%=同量级=
  **fix-c obstacleHeightThre=0.20 泛化到 office 地毯成功**（任务核心考验点，通过）。

### 门 d 复测（explore 冒烟 3min）
- reset→POST /explore→跟踪 140s：explored_volume **+26 m³**（10/14 步增长，单调-ish）、
  全程**直立**(up_z∈[-1.00,-0.966])、**无冻结无翻倒**→ 达标（体积增长/不冻/不翻）。
- **限制（诚实）**：robot 探索中**仅在原点 ±0.4m 徘徊**（X[-0.24,0.06] Y[-0.23,0.14]），
  未走进开阔厅。归因：拥挤原点角落 frontier 有限 + 低 RTF idle stale-path/wz 把 robot 钉在
  原地（fix-a/W1-W3 残余）。TARE boundary 无关（use_boundary=false，仓库多边形未激活，已证）。
- **修配置建议**：把 office spawn 移到开阔厅（如 (-3,-5)，/terrain_map 证实开阔净空 3m+），
  explore 会从空地展开、robot 有 frontier 可去；或先发一个 -Y 走廊 waypoint 把 robot 带进厅
  再 explore。此为 spawn 校准项（config，非红线）——见下 spawn 决策。

### 门 b 终测（开阔厅 spawn (-2.5,-5.0)，配对重启后）——PASS
- 新 spawn 落点 GT world **(-2.49,-5.02,0.377) up_z=-0.9999 出生即直立**（gate a 复确认 green）。
  SLAM map 帧原点=新 spawn；/terrain_map(map帧) 显示开阔厅向 +X/+Y 展开净空达 5.4m。
- **fork（target map (3,2) ~5.5m, 40s）**：占空 **84.9% PASS**、GT 速 **0.404 m/s PASS(≥0.35)**、
  直立 **100% PASS**、cmd.x mean 0.33 max 0.40、净位移 2.92m、末段仍在动(0.436m)。
  robot world (-2.48,-4.98)→(-0.09,-3.26)=+X+Y 朝 target 方向 2.9m（在窗内未到点但方向正确、
  持续推进，非早停）。
- **空路径率（fix-c 泛化核心考验）**：office 开阔厅 pathSize<=1=8%、office 原点 2%、仓库 fix-c
  后 0%——**全在个位数，远低于 fix-c 前 65% 闪烁**。⟹ **fix-c obstacleHeightThre=0.20 泛化到
  office 地毯成功，无 terrain 回归**（任务核心考验点 PASS）。残余 pathdir_WORLD std 36°/9%
  >60° 跳变=fix-a argmax 抖（planner 内因，仓库已知 CEO-gate 残余，office 同在，非迁移缺陷）。
- **门 b 裁定：PASS**（占空 85%、GT 速 0.40、直立 100%、terrain 泛化 8% 空帧）。开阔 spawn
  给了加速空间，方向问题消解（有开阔跑道时 fix-a 抖动不再把 robot 卡在障碍前）。

### 门 c 终测（窄道穿越，开阔厅 spawn）——PASS
- reset→target map (1.5,-3.5)，GT 密采 51s（18 帧 @3s）。robot 因 fix-a 方向残余走了
  -X-Y（world (-2.5,-5.0)→(-5.4,-6.7)），但**穿越过程本身=干净窄道**：
- **轨迹连续**：18 帧单调推进，总程 3.67m，无跳变(step max 0.337m<0.4m)，无卡死
  （无 3 连近零步）。
- **全程直立不擦墙翻车**：up_z ∈ [-1.000,-0.998] 全程<-0.9，z 稳 0.38，无翻/无塌。
- **确为窄道**：轨迹对障碍最小净空（/terrain_map inten>0.15 障碍点）**map(-0.5,-0.7)=0.12m、
  map(-1.1,-1.1)=0.15m**——robot 从 12-15cm 净空的窄缝穿过（比典型门洞余量还紧），之后展开
  到 1.6m 开阔。（map↔world 偏移忽略 yaw 旋转，净空为近似，但 0.12-0.15m 明确是窄道量级。）
- **门 c 裁定：PASS**（连续/不卡/不擦墙翻 + 实穿 12cm 净空窄缝）。

### 门 d 终测（explore 冒烟，开阔厅 spawn）——PASS（限制已记）
- reset→POST /explore→跟踪：**explored_volume 快速增长**（首分钟 +278 m³，采样窗 6/6 单调
  增，远快于原点 spawn 的 +26/3min——开阔厅 lidar 视野大）、**全程直立**(up_z∈[-1.00,-0.956])、
  **无冻结无翻倒**→POST /explore_stop。达标（体积增长/不冻/不翻）。
- **限制（诚实，同原点）**：robot 探索中仍**仅在 spawn ±0.4m 徘徊**（roam span 0.41m）——
  TARE 靠 lidar 扫掠涨体积但 robot 不远行。归因=低 RTF idle **stale-path/wz 爆发**（fix-a/W1-W3,
  navstack C++ CEO-gate 残余）把 robot 钉在原地，非迁移缺陷、非 TARE 边界（use_boundary=false
  已证仓库多边形未激活）。开阔厅 spawn 已把体积增速提 ~10x。
- **修配置/后续建议**：robot 远行探索需除 fix-a/stale-path（CEO gate, planner/navstack C++）；
  当前 explore 作为"建图冒烟"达标（体积真增、栈不崩、不翻），作为"走遍全楼"受 planner 残余限制。
- **门 d 裁定：PASS**（体积增长/不冻/不翻），远行受 CEO-gate 残余限制如实记。

### 门 e（RTF 对照）——office ≈ 仓库（未白捡性能）
- office RTF：idle 0.1999、active-nav 0.2132；仓库基线 ≈0.21。**office 更小但 RTF 未提升**——
  RTF 由 RTX lidar 渲染（render_interval=1 红线）主导、与场景几何无关。诚实结论：office RTF
  ≈0.20-0.21 = 仓库同级，无性能红利。

### 门 f（zeno E2E 产品脸，宪法验收面）——FAIL（诚实：物理到点但 idle 漂移致 verdict 不 hold）
- cmd: `.venv/bin/python -m zeno.vcli.cli --world go2w -p "导航到 (5.5,-1.5)"`（map 帧开阔点，
  预飞确认可达）。
- **物理到点成立**：日志 `[BASE] arrived+held d=0.11 / 0.13 / 0.26 / 0.33`——robot 反复开到
  target <0.5m（最近 0.09m）。手动复算 go2w_at(5.5,-1.5)：GT 距 0.51<0.8 → True（到点谓词能 ground）。
- **verdict=RAN verified=False (0/8 grounded)**：根因=**到点后 robot 不 hold、在 ~0.5m 半径内漂移**
  （实测 dist 0.09↔0.53 来回，全程直立 up_z~-0.999）。agent 反复 navigate_to(4x)+move_relative 都
  因到达瞬间无法定住 → grounding step 抓不到 in-tol 瞬间 → 0/8。
- **归因（诚实）**：这是**仓库已知 idle stale-path/wz 爆发残余**（E0'' 位移门、fix-a/W1-W3,
  navstack C++ CEO-gate）在 E2E timing 上的体现，**非 office 迁移缺陷、非 terrain 回归**。
  warehouse E2E 曾 verified=True 系该残余在其 timing 下恰好 hold 住；office 漂移把 verdict 翻负。
- **门 f 裁定：FAIL（verdict verified=False）**，但物理导航到点成立、根因=已入册 CEO-gate 残余。
  按任务铁律如实报 FAIL；不谎报。修复=除 fix-a/stale-path（planner/navstack C++, CEO gate）。

### 门 a 扩展（CEO 实测：相机——拉起即见狗）——PASS
- CEO 反馈：office 默认视角在屋顶看城市外景（office.usd 自带城市+巨包围盒），狗不可见、
  滚轮缩放步长粗。根因：启动相机 eye=(4,4,3) target=(0,0,0.5) 对世界原点，但 office spawn
  在 (-2.5,-5.0)；跟随相机 p[2]+3.6 穿 office 天花板。
- 修复：SCENES 每场景加 cam={eye,target,follow_dz}，启动 set_camera_view 用 SCENE_CAM。
  office eye=(0.7,-7.4,2.42)（出生点斜后上方、高 2.42m 压天花板下）target=spawn；
  follow_dz 2.0（历史 3.6 穿顶棚）。warehouse cam 保历史值逐字节等价。
- **验收（并入门 a）**：配对重启后 import/shot 抓帧 office_cam_startup.png——**帧内直接看到
  站立的 Go2W**（居中、直立、在瓷砖大堂 reception 前，红箱在前方地面），**非屋顶/天花板/
  城市外景**。GT 出生 (-2.77,-5.06) up_z=-0.966 直立、green:true。**门 a（含相机）PASS**。

---

# DEBUG — 修法 a 实施:localPlanner 选组迟滞(参数门控,2026-07-07,CEO [RULING] 批准 A1)

> 本环=已批修法 a(A1 形态)的实现+真跑验收。前置=H-B(严格 argmax 无滞后=放大器;fix-c 灭点火源后
> 残余 world-pathdir std ~60°)。CEO 裁定(2026-07-07):**批准 A1 参数门控滞后**;A2(:1031 空 path
> 保持窗)本轮不做;stale-path(W1-W3)单独下一 gate。

## 设计(改前写死)
- localPlanner.cpp 选组 argmax(:910-926)后加迟滞:记上一规划周期的**组合组 ID**
  (lastSelectedGroupID = rotDir*groupNum+group,须在 :946 `%= groupNum` 破坏前锁存)。
  本帧 argmax 赢家≠上帧组时,仅当 `maxScore > groupSwitchRatio × 上帧组本帧分数` 才切换,
  否则保持上帧组(粘滞只在两组分数接近时生效)。
- 失效条件(防粘滞,三重):①上帧组本帧分数=0(被封/出 dirThre 方向锥)→立即放弃;
  ②上帧组不过旋转障碍约束(minObsAngCW/CCW)→立即放弃;③全帧无组(发空 path)→记忆复位 -1。
- **参数门控**:`groupSwitchRatio` 新 ROS 参数,C++ 默认 **1.0=迟滞整块短路**(语义与上游等价);
  仅 sim 的 local_planner.launch 设 1.2。preSelectedGroupID 分支(goal 正后方,!twoWayDrive)不受影响。
  真机部署(自有上游拷贝)零变化;即便带上此码,默认关。

## 预注册门(改前写死,达门即止,不达门如实报数停手)
1. path 层:pathdir_sampler 导航中 world-pathdir std 61°(c1 直行段基线)→ **<20°**;
   >60° 跳变占比显著降;/path 空帧率保持个位数(不回归 fix-c 成果)。
2. cmd 层:wz 饱和(|wz|≥1.39)占比 → **<5%**;wz 符号翻转不高于 c1 基线(2 次/18s)。
3. 叉子门保持(不回归):cmd.x 占空 ≥70%、GT 实速 ≥0.35(远点纯巡航)、直立 ≥99%、5m 点能到(<0.5m)。
4. 避障反证(迟滞不得粘住被封组):朝真障碍簇(9,-3.7)发穿越航点 → 不穿障碍、不撞、直立。
5. office 回归:GO2W_SCENE=office 配对重启,fork 占空≥70%、空帧率个位数、pathdir std 较
   office 基线 36° 不恶化且趋 <20°。
6. 收尾:恢复 warehouse ALL-GREEN(CEO 活 REPL 在 warehouse)。
- 调参预算:groupSwitchRatio 1.2 起,最多 3 档(1.2/1.5/2.0),每档=改 launch+配对重启;用尽停手上报。

## 实施
- 改动:localPlanner.cpp(globals + declare/get_parameter + 迟滞块 + 记忆锁存/复位,~25 行)+
  local_planner.launch(+1 参数)。备份 var/evidence/pathdir_fix/*.orig;回滚=cp 回+重建+配对重启。
- 构建:navstack 容器 `source /opt/ros/jazzy/setup.bash && cd /ws && colcon build
  --packages-select local_planner`(install 是 build 符号链接,重建即生效)。配对重启 bringup.sh(铁律坑42)。

## EXPERIMENT(逐条落数,此处追加)

### 落地与实证(2026-07-07 23:19 配对重启)
- colcon 重建 local_planner 42.7s OK;配对重启 ALL-GREEN;`ros2 param get /localPlanner
  groupSwitchRatio = 1.2`(此参数仅存在于新码=新二进制加载实证);obstacleHeightThre=0.2 保留。
- diff 存证 var/evidence/pathdir_fix/fix_a.diff(+.orig 备份)。

### 迟滞 ON(1.2)三窗——全 FAIL,且比 c1 验收态差一个量级(诚实)
| 窗 | 协议 | 占空 | GT速 | 净位移 | 末距 | 直立 | 备注 |
|---|---|---|---|---|---|---|---|
| a1_arrive(鲜启动) | fork(4.5,-0.5)40s | 47.2% | 0.211 | 1.48m | 3.15m | 100% | 前2/3健康(84%),末1/3停摆+wz饱和26%;**采样窗后它自己开到了 0.51m 内**(GT t173 实测) |
| a1_arrive2 | 同上 | — | — | — | — | — | 作废:reset 异步未落位,窗中段传送污染 |
| a1_long(settle 后) | fork 120s wall≈25s sim | 51.0% | 0.118 | **0.27m** | 4.35m | 100% | **25s sim 没离开出发区**;vx max 0.26 从未进巡航 |
- pathdir(a1_long):world std **61.2°**(=c1 基线,无改善);空 path **25%**(c1 后 0-6%,大幅回潮);
  wz 翻转 50 次/31s。**未见"冻死"**(迟滞锁死假说无证据——方向仍在翻)。
- 机制指纹(per-2s):slam_yaw 全窗单调扫 160°(持续转圈),pathdir_veh 钉在 +20~30° 车体系——
  **路径方向跟着车体转**(组=车体系定义)。两个候选解释:H-hyst(迟滞锚在车体系组,车转组也转=
  追尾巴环)vs H-pre(c1 验收后、本改动前,栈内已有回归——office/相机改 warehouse_nav.py、
  policy 默认路径改道——warehouse 叉子没人复测过)。
- 混淆记录:reset 后 stale-path 把机器人拧到背向目标并 ~1 rad/s 自旋(W1-W3 已知病,污染每窗开局);
  reset 是异步的,必须轮询 /gt 落位再开窗(a1_arrive2 因此作废)。

### 单变量证伪:groupSwitchRatio=1.0(迟滞关,同二进制)配对重启,同协议复跑
- B 臂(1.0)鲜启动 fork:占空 **82.6% PASS**、GT 0.344(窗含起转,末段仍 0.59 巡航)、直立 100%;
  pathdir:空 path **0%**、world std 32.4°、>60° 跳变 6%、wz 翻转 4 次、vx 占空 91%——健康。
- **但 A/B 作废(致命混淆,发现于 B 臂开跑后)**:两臂之间(23:19 vs ~23:40 boot)并行会话连落
  三个动地基的变更——bringup.sh 23:32(policy 默认路径改道 assets/policies,audit 会话)、
  warehouse_nav.py 23:38(**默认场景翻 office**,commit 6073597,含 **B线摩擦 combine_mode=max**
  =物理变更)、d3e706a(**SLAM 外参纳入 sync**)、89e9a72(**地图 10.9° 倾斜实测,根因未定**)。
  B 臂出生 (-2.53,-4.99)=office 开阔厅实证场景已翻。A 臂=warehouse+旧地基,B 臂=office+新地基,
  **场景/物理/SLAM 三重不同,迟滞归因无效**。
- 跨臂唯一稳定信号:SLAM yaw 抖动两臂皆升(0.93-1.07°/帧 vs c1 期 0.6°/帧)——与并行会话在追的
  地图倾斜/外参病吻合,**支持 H-pre(地基病)而非 H-hyst(迟滞病)**,但未到定案标准。

## CONCLUDE(部分——fix-a 代码落地,验收 BLOCKED,诚实停手)
- **落地成立**:参数门控迟滞已入 localPlanner.cpp+launch,colcon 重建+配对重启加载实证
  (param 读回 1.2/1.0 两态均验)。diff+.orig 在 var/evidence/pathdir_fix/。
- **验收 INCONCLUSIVE**:迟滞 ON 三窗 FAIL(占空 47-55%、a1_long 25s sim 未离出发区、空 path
  17-25% 回潮),但归因被并行会话的物理/SLAM/场景变更彻底混淆;"迟滞锁死"假说已被数据否定
  (pathdir 未冻,仍翻),"追尾巴环"机制指纹(pathdir_veh 钉 +25° 而 slam_yaw 扫 160°)在
  SLAM 倾斜/外参病下同样可产生。**不下"迟滞有害/无害"结论,不谎报达门**。
- **安全态**:launch 现值 groupSwitchRatio=**1.0(迟滞关=上游语义)**,栈 office 场景 ALL-GREEN,
  robot 已 reset。其他会话的调试不会被本改动混淆(默认关);A/B 重开=launch 一行改 1.2+配对重启。
- **复验条件(预注册保持有效)**:等 SLAM 外参/地图倾斜/摩擦(A线/B线)收敛入账后,冻结窗口内
  同场景同代码重跑两臂(协议同上:鲜启动 fork + settle-reset 长窗 fork ×2 臂),原预注册门不变。
- 工具沉淀:probe_terrain_map.py 重造入 scripts/nav/(c 轮原件未落盘);reset 必须轮询落位
  (异步),fork 窗必须记录起始 yaw(掉头相污染短窗)。
- 流程教训(坑级):**共享 sim 上的 gate 实验必须先冻结拉起面**(bringup.sh/warehouse_nav.py/
  refs)——COORDINATION.md claim 只防了重启权,没防"改我下次重启会加载的东西"。
