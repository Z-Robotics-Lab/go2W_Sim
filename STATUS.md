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
0. 【修法 c 达门·蹭行在验收面根治 2026-07-07·CEO 裁定 c 先行已执行】
   **改动=local_planner.launch:31 obstacleHeightThre 0.05→0.20**(掐灭地形代价阈值闪烁点火源)。
   **前提改判(冷启动实测)**:真门=**localPlanner** obstacleHeightThre 活值 **0.05**(非上节记的 0.2
   =C++默认误抄);活栈噪声带实测 **0.05-0.15**(非 0.20-0.24);门抬 0.20 恰越噪声顶、保真障碍(>0.20)。
   **叉子门 PASS**(c1b 远点纯巡航):cmd.x 占空 5.5%→**91.4%**、GT 实速 0.16→**0.377 m/s**、
   直立 100%、能到点(c1 到点 0.169m)。**火源熄灭**:前扇 cost>0.20 cell 恒 0、/path 空帧 65%→0。
   **避障反证 PASS**:真障碍簇(8.4-9,-3.6)cost>0.25>0.20 门下仍判障碍,驱近未穿、全程直立=未调聋。
   **残余未达门(诚实)**:world-pathdir std 仍~60°=H-B argmax 无滞后选组内因,属 **fix-a(CEO gate)**
   正交残余,不阻塞验收面。**c1 单上即达门,无需 c2/c3。**
   **铁律副产**:navstack 单独重启在低 RTF 毁 SLAM(imu isam2 underconstrained)→必须配对重启(已实证)。
   改动只在 refs 克隆(容器/ws 挂载,gitignore);回滚=cp var/evidence/terrain_fix/local_planner.launch.orig
   +配对重启。零 planner/follower C++ 语义改动。证据 var/evidence/terrain_fix/。活栈 ALL-GREEN 留绿。
   **下一步(待令,不自行开火)**:fix-a(planner 选组滞后,CEO gate)灭残余 argmax 抖 → E0'' 位移门
   (<50mm,idle wz 爆发系 stale-path W1-W3,navstack C++,亦 CEO gate)。
1. 【产品脸验收 2026-07-07 晨·Tune+RTF 落地形态 = 基线】活栈=纯上游 pathFollower(rate(100)
   wall 钟,无 cruiseFloor/sim 钟)+model_5495+RTF 渲染旋钮(默认全 0)。**Tune 补丁未落地(已回滚)**。
   实测(green+upright 全程,活栈 var/evidence/lowrtf_round/accept_am/):
   ·5m 航点(0.2,4.5): cmd.x 占空 1.9%、GT 0.093 m/s、净位移 0.087m、末距 4.89m — FAIL(叉在纯偏航,
     wz 饱和±1.396,机器人反向漂到 -y);录像 isaac_5m_wp.mp4(75s)+帧 isaac_5m_frame.png(眼见真 Isaac)。
   ·1.5m 航点(0.7,-0.5): cmd.x 占空 13.3%、GT 0.186 m/s、净位移 0.40m、末距 1.65m — FAIL(复现晨基线 14%)。
   ·两段直立 100%,RTF≈0.20(双测:stamp 0.20+轨迹 12s/60s)。·explore 端点 plumbing 活(owner 翻转+
     409 互斥+stop),但 NAV_MODE=waypoint 下 TARE 未加载,真 explore 需配对重启进 explore 模式
     (free=11G<20G 且会毁绿栈→本轮未跑;explore 早前 P5.2 已 verified)。
   **结论: Tune 未修好蹭行,活栈=已知 FAIL 的 model_5495 基线;dirDiff/pathDir 振荡仍为真绑定,
   真解触 planner CEO gate。** 验收结束已 /reset 回出生点,栈留绿。
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
