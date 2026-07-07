# DEBUG — Isaac Sim "杀不死" / teardown 不可靠（2026-07-06）

活体僵尸样本取证 → 加固 teardown → 杀灭矩阵实测 → 成对重启验绿。

## OBSERVE（21:48 活体僵尸取证，编排者已确认步态实验被污染，可解剖）

现场（用户 GUI-quit / zeno-stop 尝试后残留的"杀不死"形态）：
- 桥 `/health`: `ok:true`，但 `pose age_s=957.962`、`gt age_s=958.479`（仿真循环冻结 ~16min）。
- `/pose` 被陈旧守卫 503（坑 31 守卫尽职）。phase 文件谎报 `up (green)  21:24:03`。
- 宿主 `pgrep -fc "kit/pytho[n]"` = 1（kit 进程活着）。go2w-isaac Up 27m、navstack Up 25m。

Isaac 三进程树（容器命名空间 PID / STAT）——与阶段 1 健康态同构：
```
623  Ss  bash -c ... /isaac-sim/python.sh warehouse_nav.py ...   # 会话组长, cmdline 无 kit/python
629  S   /bin/bash /isaac-sim/python.sh warehouse_nav.py ...     # python.sh 启动器, 无 kit/python
634  Rl  /isaac-sim/kit/python/bin/python3 warehouse_nav.py ...  # 真 sim, 有 kit/python  ← 目标
```
关键：**634 STAT=Rl（Running），wchan=0，232 线程全扫 D=0 Z=0** —— 不是 D 态不可中断挂起，
是**逻辑活锁/冻结**（主循环 `while simulation_app.is_running()` 还在转，但仿真时钟冻死）。
残留僵尸子（已被 PID-1 收养或挂 634）：`[carb.tasking5/23] [omni.telemetry.]` ×4 —— Z 态，无害噪音。

GPU：nvidia-smi `82059(=容器634) 5279MiB`，类型 C+G（CUDA+GL/X11，GUI 窗口归它）。

`/isaac-sim/python.sh` L72 `$python_exe "${args[@]}"` —— spawn-and-wait 非 exec。
warehouse_nav.py：收尾 `simulation_app.close()`(L508) 仅当 is_running()→False 走；**全文件无 signal handler**。

teardown 模式 DRY-RUN（容器内 cmdline 扫 `*kit/python*`）：**只 634 命中 → WOULD-KILL pid=634**。
宿主侧 `pgrep -f "kit/pytho[n]"` 也只 82059 命中。→ 模式对真 sim 有效。

## HYPOTHESIZE（"用户为什么杀不死"）

| # | 假设 | 类别 | 证据 |
|---|------|------|------|
| H1 | 模式失配，-9 打不中 kit 进程 | 模式 | 证伪：DRY-RUN 命中 634；634=Rl 可被 -9 |
| H2 | D 态不可中断，-9 无效 | GPU/X | 证伪：634=Rl wchan=0，232 线程 0 个 D |
| H3 | 用户的 GUI-quit / zeno-stop 根本没发出容器级 -9 | 路径 | GUI 窗口冻结不服务输入→quit 设不了 is_running=False；zeno "stop"→stop_simulation 在 go2w 世界被 disable("sim") 禁用=无操作 |
| H4 | 唯一有效路径(go2w_bringup teardown)不可信 | 复核缺失 | pkill\|\|true 恒成功；status.sh 退出码被 \|\|true 丢；zeno L149 不读 returncode 不设 is_error |
| H5 | rosm 打不中容器内 kit | 工具错配 | rosm 目标宿主进程；且坑23 记录 rosm 跨命名空间"误伤"navstack，非"清 Isaac" |

## EXPERIMENT / 结果

- H1 REJECTED：容器 dry-run WOULD-KILL 634；宿主 pgrep 命中 82059。
- H2 REJECTED：634=Rl、wchan=0、232 线程 D=0 Z=0 —— 非 D 态，-9 能收割。
- H3 CONFIRMED：GUI 冻结→quit 无效；`disable("sim")` 使 zeno "stop" 无 tool 可路由（go2w.py L615-617）。
- H4 CONFIRMED：读码 bringup.sh L44/L48 + go2w.py L149。
- H5 CONFIRMED：rosm 宿主向；坑 23。

## CONCLUDE — 用户"杀不死"的真因（复合）

**不是** -9 无效、**不是** D 态、**不是** 模式失配。真因链：
1. **用户试的两条路本就发不出容器级 -9**：GUI-quit（窗口活锁不响应）+ zeno "stop"
   （stop_simulation 被 go2w 世界禁用，无操作）。唯一能发 -9 的是 `go2w_bringup(teardown)`。
2. **那条唯一路径不可信 + 静默假成功**（CEO 抱怨核心）：`pkill…||true` 恒 0 退出；
   status.sh 退出码被 `||true` 吞；zeno 层不读 returncode/不设 is_error → "说关了"实为"发了信号没复核"。
3. **rosm 是宿主工具**，打不进容器内 kit，且历史上只会误伤 navstack（坑 23）。

死法附注：本例是 sim 逻辑冻结（Rl 活锁）非内核 D 态；但 -9 跳过 simulation_app.close()
→ GL/CUDA 上下文不释放，是 Isaac 公认的 D-态-on-teardown 风险 → 故 teardown 必须有
**容器级兜底升级**（docker restart go2w-isaac，终结整个 PID namespace，保留容器）。

## FIX（加固，见下游提交）

- bringup.sh teardown()：精确打击 + 分级升级(TERM→KILL→docker restart)+ 逐级复核；
  判据 = 宿主 pgrep -f "kit/pytho[n]" 空 且 status.sh l0=false；失败退非零 + 打印残留表。先杀 navstack（RViz 不回弹）。
- status.sh：新增 L4b pose 新鲜度探针（/health age_s < 阈值），green 不再被冻结僵尸骗（本次它误导了所有人）。
- zeno Go2WBringupTool teardown：读 returncode，非零 → is_error=True 带残留表。
- docs/pitfalls.md：rosm 无效原因+正确姿势 + RViz 自愈说明。

## 杀灭矩阵实测结果（2026-07-06 21:56-21:59，活体僵尸 + 成对重启）

前置（M2 冻结僵尸现场）：host `pgrep -fc kit/pytho[n]`=1、GPU 82059=5279MiB、
`/health` pose age_s=1167、`/pose` 503、phase 文件谎报 `up(green)`、status.sh green=false、
可用内存 46-53G。

| 用例 | 执行 | 结果 | 判据 |
|------|------|------|------|
| **M2 冻结僵尸杀灭** | `bringup.sh teardown` | **PASS** exit 0 | [1/3]rm navstack→[2/3a]SIGTERM→**TERM 即生效无需 KILL**→SUCCESS |
| 杀后复核（宿主） | `pgrep -af kit/pytho[n]` | **空** | 无真实残留（前测 count=1 是 pgrep 自匹配伪影，-af 证空） |
| 杀后复核（GPU） | nvidia-smi | **kit 消失** | GL/CUDA 上下文已释放，5279MiB 归还 |
| 杀后复核（容器） | `docker exec go2w-isaac ps` | **无 kit/warehouse** | 容器内干净 |
| 容器保留 | `docker ps` | **go2w-isaac 仍 Up** | 只杀 sim 进程，容器未重建（满足重建代价高） |
| status.sh | — | **l0=false green=false** | 达"拆净"判据 |
| **M5 成对重启** | `bringup.sh`(waypoint) | **PASS** ~70s green | l0-l5 全 true、green=true、exit 0 |
| 新栈新鲜度 | `/health` | **pose age_s=0.147** | <5 新鲜（freshness 闸放行 live sim） |
| 新栈 /pose | curl | **HTTP 200** | SLAM 有位姿 |
| **M7 green 假象防护** | freshness 闸单测 | **PASS** | age 0.147/4.9→FRESH；5.0/958.4→STALE 挡 green |

关键实测结论：
1. **僵尸一发 SIGTERM 就死**（Rl 活锁非 D 态）——彻底证实"用户杀不死"= 他走的 GUI-quit /
   zeno-stop 根本没发容器级信号，**非** -9/pattern/D 态问题。加固 teardown 一次拆净。
2. **容器全程保留**：go2w-isaac 从 21:24 起连续 Up 38min 跨越 teardown+bringup 未重建，
   重建代价高约束满足；docker restart 兜底本轮未触发（TERM 已够）。
3. **freshness 闸有效**：僵尸 age=1167s→green:false（拒僵尸）；新栈 age=0.147s→green:true
   （放行 live）——green 假象已堵。
4. 离线单测全绿：zeno 17 passed（含 2 个新 teardown 保真单测）；bash -n 双脚本 OK。

**阶段 2 完成**：杀灭矩阵实测通过，新栈已 green（/pose 200, age<1s），可交付步态实验续用。
