# STATUS — go2W_Sim 会话恢复锚点（覆盖式，≤40 行）

更新：2026-07-06。仓库：github.com/Z-Robotics-Lab/go2W_Sim（main，与远端同步）。

## CEO 任务序列
- ① 导航栈 Isaac 跑通：✅ 方形回归 4/4 + RL 运动策略（commit 3d9dc50）
- ② vector_os_nano agent 控狗：✅ VECTOR_VERDICT verified=true GROUNDED EXIT 0（93e77ef）
- ③ 接入抓取：进行中（刚起步，见下）
- P5.1 基建：✅ bringup/status/teardown 三动作 + RViz 进 supervisor（feat/p5-bringup，未 push）
- P5.2 探索基建：✅ TARE 探索接入 4 件（feat/p5-bringup，未 push；仅静态验证，见下）

## 运行拓扑（两容器 + 宿主）
- go2w-isaac 容器：warehouse_nav.py（RL策略/传感器桥/GT发布，DDS 域 42，root 运行）
- navstack 容器：PID-1 supervisor（run_all_forever.sh：转换器+SLAM栈+HTTP桥:8042）
- 一键全链重启+门控：bash scripts/nav/restart_all.sh（铁律：两侧必须配对重启）
- 幂等拉起/健康探针/拆链：bringup.sh [up|teardown]（-e NAV_MODE 选 waypoint/explore）· status.sh
- agent 入口：bash scripts/vector_os/run_agent.sh [--no-permission -p "目标"]

## 关键文件
- scripts/sim/warehouse_nav.py — Isaac 桥（--policy RL；GT /ground_truth/pose）
- scripts/vector_os/isaac_go2w_world.py — BYO world；run_agent.sh — 3 个 shim（Phase-C 临时补丁）
- scripts/nav/{restart_all,run_all_forever,agent_bridge,patch_navstack,waypoint_regression}
- 接口合同 docs/agent-bridge-api.md；坑(27) README.md；里程碑 docs/sim-plan.md；调研 go2w-dev-roadmap.md

## 任务③（抓取）下一步
1. warehouse_nav.py 加 PiPER 关节 ROS2 接口（关节已是真 ImplicitActuator）
2. 场景放可抓物（小箱/圆柱，Isaac 原生 prim 即可）
3. embodiment 加 grasp base 合同 + holding_object GT 谓词（复用②的全套模式）
4. 验收：agent NL "把箱子捡起来" -> verified=true

## P5.1 / P5.2 下一步（集成阶段实测，本轮均只静态验证）
- P5.1: bash scripts/nav/bringup.sh -> status.sh green + RViz 出图 + /health；teardown 后 l0=false
- P5.2: NAV_MODE=explore bringup -> POST /explore 目测狗自主探索；接口合同见 docs/agent-bridge-api.md
- 【须实测】/explore_stop 发 Bool false 对 TARE 是 no-op（ExplorationStartCallback 只认 true）；
  硬停靠 NAV_MODE=waypoint 重启 system 链
- 【须实测】启动话题真名 /start_exploration（yaml 覆盖 code default /exploration_start），
  `ros2 topic info /start_exploration` 确认 TARE 订阅
- patch 第5步产物仅在隔离克隆验证（未污染主树基线克隆）；集成时对干净克隆真跑 patch

## 裁决项（待 CEO）
- 把 world.register_tools 接线（Phase C）以 PR 贡献回 vector_os_nano -> 删 3 个 shim
- rosm nuke 越权杀跨项目 ROS 进程（违反其自身 teardown 头注），建议 scoped 修复
