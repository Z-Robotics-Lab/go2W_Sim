# STATUS — go2W_Sim 会话恢复锚点（覆盖式，≤40 行）

更新：2026-07-06。仓库：github.com/Z-Robotics-Lab/go2W_Sim（main，与远端同步）。
姐妹仓：github.com/Z-Robotics-Lab/z-agent（agent 运行时，fork 自 vector-os-nano）。

## CEO 任务序列
- ① 导航栈 Isaac 跑通：✅ 方形回归 4/4 + RL 运动策略（3d9dc50）
- ② agent 控狗：✅ verified=true（93e77ef）；F2 后升级为零 shim 一等公民路径 ✅
  （z-agent `--world go2w` E2E：arrived+held d=0.32，GROUNDED verified=True）
- ③ 接入抓取：暂停，WIP 存 feat/grasp-wip（60%：IK伺服+状态机+箱子话题+臂合同）
- P5.1 一键拉起：✅ 真跑验收（4ab8d63）——幂等短路/teardown/冷拉起 ALL-GREEN 至 L5
  （RViz 出图）/ /health 正常；集成缺口"容器跑旧拷贝"已修（sync_navstack_files.sh）
- P5.2 探索基建：✅ 代码合入；真跑验收待做（NAV_MODE=explore + TARE 实测，见下）

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

## 下一步（P5.2 真跑验收）
1. NAV_MODE=explore bash scripts/nav/bringup.sh（先 teardown）
2. `ros2 topic info /start_exploration` 确认 TARE 订阅真名（yaml 覆盖 code default）
3. POST /explore -> 目测自主探索 + /explore_progress 的 explored_volume 增长
4. 【已知】/explore_stop 对 TARE 是 no-op（源码只认 true）——硬停用 NAV_MODE=waypoint 重启
5. z-agent 侧：explore 技能 + Δexplored_volume 验证谓词；随后恢复任务③（抓取）

## 裁决项（待 CEO）
- 真机 verify 语义 / NUC vs Orin 拓扑 / unitree_sdk2_python 依赖（P5.4 前）
- rosm nuke 越权杀跨项目 ROS 进程，建议 scoped 修复（vector_os_nano 侧）
