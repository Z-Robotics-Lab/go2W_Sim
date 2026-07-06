# DEBUG — Isaac 里 Go2W 移动异常缓慢、疑似轮子未被驱动（2026-07-06）

## OBSERVE
- CEO 目测：移动异常缓慢；姿态像"走路"而非轮驱；此前同一会话内导航/探索验收均正常
  （E2E d=0.32/d=0.36 到点、探索 2 分钟 +1850 m³）。
- 时间线：正常 E2E → 两次 explore 模式冷拉起 → TARE 持续探索 1.5h+ → CEO 手点过
  RViz TeleopPanel（发 /joy）→ 现在观察到异常。
- 待取证：RTF（仿真时间/墙钟比）、/cmd_vel 实际指令值、/joy 残留、内存/GPU。

## HYPOTHESIZE
| # | 假设 | 类别 | 证据 |
|---|---|---|---|
| H1 | TeleopPanel 残留 joy 锁死速度：joystickHandler 设 joySpeedRaw≠0 后，speedHandler 永远被短路（源码 pathFollower.cpp:195 条件 joySpeedRaw==0），车速被钳在面板残留的小值 | teleop 残留 | CEO 点过面板；源码条件实锤存在；时间线吻合 |
| H2 | teleop 把 autonomyMode 翻成 false（axes[2]>-0.1 即关，坑29），pathFollower 不再跟 /path，只剩微小站立调整 | teleop 残留 | 同上；但探索体积仍缓涨，部分矛盾 |
| H3 | 长跑 RTF 崩塌：Isaac GUI+RTX 雷达+相机跑数小时后物理步率下降，一切动作等比变慢 | 性能退化 | sim 已连续跑数小时 |
| H4 | 轮执行器失效（策略模式 stiffness0/damping0.5 组合在某种状态下扭矩不足） | 物理 | 弱：同配置此前多次验收正常 |

## EXPERIMENT
（按证据强度+证伪成本排序，一次一个）
- E1(H3): 20s 内两读 /gt stamp -> RTF = Δsim/Δwall
- E2(H1/H2): ros2 topic echo /cmd_vel 一帧，看指令 vx 大小
- E3(H1/H2): 发一条"矫正 joy"（axes 全零 + axes[2]=-1 保持自主）解除残留锁，观察速度是否恢复
- E4(H3/H4): 若 cmd_vel 正常而位移率异常 -> 查 free/docker stats/GPU 与轮速响应

## EXPERIMENT 结果
- E1: RTF=0.210（基线 0.3-0.5，长跑退化）→ H3 部分成立（放大观感，非根因）
- E2: /cmd_vel = 0.0 → 上游没给指令，轮子不转是正确行为
- 追加取证: TARE-ALIVE 但 /way_point 无流量、/exploration_finish=true（11306m³）
  → **探索完成了**，H1/H2（teleop 残留）REJECTED——与 teleop 无关
- 追加发现: 桥 volume_age=55.6s 仍 200 → 僵尸桥（kill 后 HTTP 半边带冻结状态存活）

## CONCLUDE（终版，四病共存全部落地）
根因链（按发现序）：
① TeleopPanel joy 锁速+关自主、栈内无 /speed 自愈源 -> 修：桥 1Hz /speed +
   航点矫正 joy + RViz 去毒面板（go2w.rviz）
② （医源）矫正 joy 空 buttons -> terrainAnalysis buttons[5] 无界索引 SIGSEGV
   -> 修：buttons 补满 12 位
③ （医源）FAST_RENDER 关 RTX 特效疑似连累 RTX 雷达 -> 回退默认关
④ 【真核心】fullScan 整帧点云点序完全非时序（解剖实测方位角前向率 48.5%=随机，
   发射器状态交织）-> 按索引铺 offset_time = 逐点随机时戳 -> 静止无感、
   运动中去畸变毁灭性出错（z 俯冲 -0.4/3s）-> 地形窗口被打空 -> localPlanner
   空路径 -> pathFollower 全零 -> 走走停停爬行
   -> 修：Isaac 侧 fullScan=False 增量模式（到达即时序）+ 转换器缓冲聚合
      0.1s 帧、按增量片真实时戳赋 offset_time
策略全程无罪：干净注入 vx 0.15/0.30/0.60 -> 跟踪 171%/128%/107%（hip scale
修复后）；"训练指令死区 0.2"假设被证伪（小指令有 ~0.25 地板但可用）。
修复后实测：运动中 z 稳定（-0.234→-0.236/30s，此前 3s 俯冲 0.4）；
cmd_vel 出现 0.47-0.48 连续指令（此前全零）；直线巡航恢复 ~0.45 量级
（窗口平均 0.209 含 180° 掉头段）。
观察项：长会话 z 缓沉与掉头段 z 下探（-0.48）继续跟踪；RTF 0.12-0.2 的慢动作
是渲染税（雷达时钟正确性约束 render_interval=1），不影响正确性。
