# STATUS — go2W_Sim 会话恢复锚点（覆盖式，≤40 行）

更新：2026-07-07（低 RTF 轮 FAIL 已回滚）。仓库：github.com/Z-Robotics-Lab/go2W_Sim（main，与远端同步）。
姐妹仓：github.com/Z-Robotics-Lab/z-agent（agent 运行时，fork 自 vector-os-nano）。

## CEO 任务序列
- ① 导航栈 Isaac 跑通：✅ 方形回归 4/4 + RL 运动策略（3d9dc50）
- ② agent 控狗：✅ verified=true（93e77ef）；F2 后升级为零 shim 一等公民路径 ✅
  （z-agent `--world go2w` E2E：arrived+held d=0.32，GROUNDED verified=True）
- ③ 接入抓取：暂停，WIP 存 feat/grasp-wip（60%：IK伺服+状态机+箱子话题+臂合同）
- P5.1 一键拉起：✅ 真跑验收（4ab8d63）——幂等短路/teardown/冷拉起 ALL-GREEN 至 L5
  （RViz 出图）/ /health 正常；集成缺口"容器跑旧拷贝"已修（sync_navstack_files.sh）
- P5.2 探索：✅ 真跑验收——NAV_MODE=explore 全绿、TARE 订阅实测、互斥 409 实测、
  agent E2E '探索这个仓库' -> explored_volume 10140->10264 -> verified=True EXIT 0

## 运行拓扑（两容器 + 宿主）
- go2w-isaac 容器：warehouse_nav.py（RL策略/传感器桥/GT，DDS 域 42，root）
- navstack 容器：PID-1 supervisor（转换器+SLAM栈+HTTP桥:8042+RViz；-e NAV_MODE 选 launch）
- 幂等拉起/探针/拆链：bringup.sh [up|teardown] · status.sh（L0-L5 分层）
- agent 入口：z-agent 仓库 `vector-cli --world go2w`（旧 run_agent.sh shim 已被取代）
- 铁律：配对重启；scripts/nav 是唯一真相源（sync_navstack_files.sh 拷进 refs）

## 关键文件
- scripts/sim/warehouse_nav.py — Isaac 桥；scripts/nav/{bringup,status,sync_navstack_files,
  restart_all,run_all_forever,agent_bridge,patch_navstack}
- go2w 世界已迁入 z-agent（vcli/worlds/go2w.py）；本仓 scripts/vector_os/ 为历史参考
- 接口合同 docs/agent-bridge-api.md；手册 docs/user-manual.md；NUC docs/nuc-setup.md；
  坑表 docs/pitfalls.md；里程碑 docs/sim-plan.md

## 下一步
1. 【低 RTF 时域适配轮 2026-07-07 FAIL·门未达标·已回滚】叉子实验门(cmd.x 非零占比≥70%
   +GT实速≥0.35 m/s(sim))**未达标**。剥出三层病灶(DEBUG.md 完整假设环+定量表):
   ①控制时钟错配(wall Rate(100) vs RTF 0.17→积分器过激)已用 sim 钟修;②非对称刹车清零
   已软化;③死区被 sim 钟放大误伤已加活跃导航守卫。三者叠加(死区关)达 64.4% 占比、
   dis 真降 3.6m 但仍不达门。**真绑定=pathDir 振荡**(wz±1.396 翻转,twoWayDrive已False)——
   局部路径前瞻点方向本身不稳,病在 planner/SLAM 侧(触红线,超 pathFollower 范围)。
   **活栈已回滚到 model_5495 接受基线**(warehouse_nav L2守卫撤、pathFollower C++ 重建纯上游、
   参数纯上游;GREEN/upright/params 已核)。L3/L3b/L2 补丁存 git 历史(commit 15d681a..44b4813)
   待 pathDir 稳定后续接。真解方向:pathFollower dirDiff 用全局目标方位替振荡的局部
   pathPointID 方向(需 CEO gate:触 planner 语义)。工具留:scripts/nav/fork_{experiment.sh,
   analyze.py}、var/evidence/lowrtf_round/。
2. 【训练线关账 2026-07-07】载荷策略 **model_5495 已落地**(全门过);栈 ALL-GREEN 移交 CEO
   产品测试(回滚=bringup.sh 注释换回 model_3497)。残余:栈侧 wz 爆发根除、arc 软瑕疵。
3. 恢复任务③抓取（feat/grasp-wip 60%，迁 z-agent 体系收尾）
3. TARE 软停缺口（源码只认 start=true）：产品要软停需改栈源码（CEO gate）或接受
   NAV_MODE=waypoint 重启作为硬停
4. P5.4 真机（等 CEO 三决策）；SLAM z 漂移观察项（本次会话 -0.5m，真机标定时一并看）

## 裁决项（待 CEO）
- 真机 verify 语义 / NUC vs Orin 拓扑 / unitree_sdk2_python 依赖（P5.4 前）
- rosm nuke 越权杀跨项目 ROS 进程，建议 scoped 修复（vector_os_nano 侧）
