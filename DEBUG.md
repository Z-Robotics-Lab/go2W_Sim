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

## CONCLUDE
根因链（一句话：不是故障，是探索完成 + 两个被调查暴露的真缺陷 + RTF 放大观感）：
1. 直接原因（非缺陷）：TARE 探索完成 → 不再发航点 → pathFollower 发 0 速 →
   机器人站立，RL 策略零指令下的站立微调看起来像"慢走"、轮子正确地不转。
2. 真缺陷 A：探索完成后桥 nav_owner 卡在 explore，手动导航被 409 锁死。
   修复：agent_bridge.py on_exploration_finish 里 finished→owner 释放为 idle。
3. 真缺陷 B（僵尸桥，护城河级）：rclpy 接管 SIGTERM/SIGINT——spin 在子线程时，
   信号只杀 ROS 线程，HTTP 主线程带冻结 STATE 继续应答（实测 55s 陈旧数据仍 200，
   verify 谓词可能读到陈旧 GT）。修复：①结构——rclpy.spin 回主线程、HTTP 进守护
   线程，信号=全进程退出=supervisor 干净重生；②守卫——/pose /gt 超 5s 未更新 503。
4. 放大因素：RTF 0.21（GUI 长跑退化）——teardown+bringup 可恢复。
验证：新桥 age 0.08/0.28s、owner=idle、POST /waypoint 200、GT 恢复位移（0.3m/15s 墙钟）。
回归防护：结构修复使僵尸态不可能存在 + 陈旧守卫堵死"冻结数据喂谓词"路径（坑 30/31）。

