# Teardown 杀灭矩阵（Go2W 数字孪生栈）

加固后 `scripts/nav/bringup.sh teardown` 的真实杀灭验收清单。每个用例给出：前置、
执行、**判据（机器可查）**、预期。判据统一以 **宿主侧 `pgrep -f "kit/pytho[n]"` 为空
+ `status.sh` `l0=false`** 为"拆净"真值——不信任工具自报的成功字样。

约束（不变）：ONE sim；scoped 杀灭（容器内 exec / 容器级 docker，绝不宿主 pkill）；
NEVER-KILL-INFRA（不动 sibling 会话/loop）；bringup 前 `free -g` 复核可用内存 ≥20G。

判据速查（一行）：
```bash
pgrep -fc "kit/pytho[n]"                          # 期望 0
bash scripts/nav/status.sh | grep -o '"l0":[a-z]*' # 期望 "l0":false
```

---

## M1 — 脚本路径全灭复核（普通级：TERM/KILL 生效）
- 前置：栈 UP 或 kit-python 在跑（`pgrep -fc "kit/pytho[n]"` ≥1）。
- 执行：`bash scripts/nav/bringup.sh teardown`
- 判据：脚本 `exit 0`；`pgrep -fc "kit/pytho[n]"` == 0；`status.sh` l0=false；
  输出含 `SUCCESS`；`docker ps` 无 navstack、go2w-isaac 仍在（容器保留）。
- 预期：SIGTERM 或 SIGKILL 级即拆净，不触发 docker restart。

## M2 — 冻结僵尸（活锁 Rl / GUI-quit 残留形态）
- 前置：复现"杀不死"现场——kit-python STAT=Rl、`/health` pose age_s 大（sim 冻结）、
  `/pose` 被陈旧守卫 503、phase 文件谎报 `up (green)`。
- 执行：`bash scripts/nav/bringup.sh teardown`
- 判据：同 M1（exit 0 + pgrep 0 + l0=false）；此形态 -9 有效（Rl 可收割），
  一般不需升级到 docker restart。
- 预期：证明"用户杀不死"非 -9 无效，而是他走的 GUI-quit/zeno-stop 根本没发容器级 -9。

## M3 — D 态兜底升级（docker restart 路径）
- 前置：kit-python 卡 D 态（-9 无效）。**难以人为构造**——D 态多由 GPU/X 驱动栈触发；
  若自然出现则采样，否则用注入验证升级逻辑（见 M3b）。
- 执行：`bash scripts/nav/bringup.sh teardown`
- 判据：输出出现 `[3/3] … docker restart go2w-isaac`；重启后 `pgrep -fc` == 0；
  go2w-isaac 仍存在（`docker ps` 有它，重启非重建）；`exit 0`。
- 预期：SIGKILL 后仍存活 → 容器级重启终结整个 PID namespace → 拆净。

## M3b — 升级逻辑注入验证（不需真 D 态，离线可跑）
- 目的：在没有真 D 态样本时，验证"KILL 后仍存活 → docker restart"分支会被走到。
- 执行：临时把 `_isaac_signal` 置空操作（`GO2W_TD_GRACE_S=1` + 环境注入一个 stub，
  或用 `docker exec` 起一个假装匹配 `kit/pytho[n]` 且忽略 TERM/KILL 的进程），
  跑 teardown，断言输出出现 `docker restart` 分支且最终 `exit 0`。
- 判据：日志含 `[2/3a] SIGTERM` → `[2/3b] … SIGKILL` → `[3/3] docker restart`。

## M4 — zeno 工具路径 teardown（对接保真）
- 前置：kit-python 在跑；`GO2W_SIM_DIR` 指向 go2w 仓库。
- 执行：经 zeno `go2w_bringup(action="teardown")` 工具（非直调脚本）。
- 判据：拆净时工具 `is_error=False`；**残留时工具 `is_error=True` 且 content 含残留
  进程表**（脚本非零退出如实上抛，2026-07-06 修复点）。离线单测已覆盖
  （tests/vcli/test_world_go2w_firstclass.py::test_teardown_reports_error_when_script_exits_nonzero）。
- 预期：工具不再"说关了实际没关"。

## M5 — teardown 后立即 bringup（成对重启）
- 前置：M1/M2 刚拆净。
- 执行：`bash scripts/nav/bringup.sh`（NAV_MODE=waypoint 缺省）→ 轮询 status.sh + phase 文件。
- 判据：2-6 min 内 `status.sh` green=true；`/pose` HTTP 200 且 `/health` pose
  `age_s < 5`（新鲜度双闸）；`docker ps` navstack + go2w-isaac 都 Up。
- 预期：拆链不损坏容器，配对重启（navstack↔Isaac 桥）fastdds 重新发现，SLAM 收敛。

## M6 — RViz 不再回弹
- 前置：栈 UP，RViz 窗口在。
- 执行：`bash scripts/nav/bringup.sh teardown`
- 判据：teardown 先 `docker rm -f navstack`（PID-1 supervisor 随容器消亡）→
  `docker exec navstack pgrep -x rviz2` 报容器不存在（非"rviz2 又起来了"）。
- 预期：先杀 supervisor 再动 Isaac，RViz 无宿主可回弹。
- 反例（回归护栏）：手动 `docker exec navstack pkill rviz2` 而不 teardown → supervisor
  3s 内把 rviz2 拉回（自愈设计，见 pitfalls 坑 36）——证明必须走 teardown。

## M7 — green 假象防护（status.sh 新鲜度闸）
- 前置：制造冻结 sim（pose age_s 大）但桥仍应答的场景（M2 现场即是）。
- 执行：`bash scripts/nav/status.sh`
- 判据：`green:false`（l4 因 `_pose_fresh` age≥5s 判假，即使 /pose 返回 200 也不 green）。
- 预期：冻结僵尸不能再冒充 green（2026-07-06 phase 文件谎报教训）。
