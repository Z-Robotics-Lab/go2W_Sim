# STATUS — go2W_Sim 会话恢复锚点（覆盖式，≤40 行）

更新：2026-07-07。仓库：github.com/Z-Robotics-Lab/go2W_Sim（main，与远端同步）。
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
1. 【已收官】策略重训线(2026-07-07,三轮):Round-3(r1 配方 +1000 iter)→ model_3497
   **产品裁定落地**(编排者代 CEO,可复议,记录在 sim-plan):①部署绑定面 0.0101 PASS
   (8.7×);原生随机体群中位 0.0343 照实 FAIL;③零摔④0.050/0.010 最佳②轻过冲。
   bringup/restart_all 默认已切(旧路径注释回滚一行)。E0'' 收官:120s 直立(单次瞬态
   侧倾恢复)+4m WP 回归全程直立、推进优于旧策略;帧证据+全数据 var/evidence/retrain/。
   残余风险与载荷包络待办见 DEBUG.md 收官节。栈已拆,复现 bringup.sh 一条命令。
2. 恢复任务③抓取（feat/grasp-wip 60%，迁 z-agent 体系收尾）
3. TARE 软停缺口（源码只认 start=true）：产品要软停需改栈源码（CEO gate）或接受
   NAV_MODE=waypoint 重启作为硬停
4. P5.4 真机（等 CEO 三决策）；SLAM z 漂移观察项（本次会话 -0.5m，真机标定时一并看）

## 裁决项（待 CEO）
- 真机 verify 语义 / NUC vs Orin 拓扑 / unitree_sdk2_python 依赖（P5.4 前）
- rosm nuke 越权杀跨项目 ROS 进程，建议 scoped 修复（vector_os_nano 侧）
