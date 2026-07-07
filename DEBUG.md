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

## CONCLUDE（待真跑回填）
