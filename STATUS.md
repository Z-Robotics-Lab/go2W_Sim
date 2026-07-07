# STATUS — go2W_Sim 会话恢复锚点（覆盖式，≤40 行）

更新：2026-07-07（修法 c 达门）。仓库：github.com/Z-Robotics-Lab/go2W_Sim（main）。
姐妹仓：github.com/Z-Robotics-Lab/z-agent（agent 运行时，fork 自 vector-os-nano）。

## CEO 任务序列（已完成里程碑）
- ① 导航栈 Isaac 跑通 ✅（方形回归 4/4 + RL 策略）· ② agent 控狗 ✅（z-agent `--world go2w`
  E2E arrived+held verified=True）· ③ 抓取：暂停 WIP feat/grasp-wip 60%
- P5.1 一键拉起 ✅ · P5.2 探索 ✅（NAV_MODE=explore 全绿、agent E2E verified）
- 训练线关账：载荷策略 **model_5495 已落地**（全门过；回滚=bringup.sh 注释换 model_3497）

## 运行拓扑（两容器 + 宿主）
- go2w-isaac：warehouse_nav.py（RL策略/传感器桥/GT，DDS 域 42）
- navstack：PID-1 supervisor（转换器+SLAM+HTTP桥:8042+RViz；-e NAV_MODE 选 launch）
- 幂等：bringup.sh [up|teardown] · status.sh（L0-L5）· agent 入口 z-agent `vector-cli --world go2w`
- 铁律：**配对重启**（单独重启 navstack 在低 RTF 毁 SLAM，坑42 实证）；scripts/nav 唯一真相源

## 关键文件
- scripts/sim/warehouse_nav.py · scripts/nav/{bringup,status,sync_navstack_files,agent_bridge}
- 接口 docs/agent-bridge-api.md · 坑表 docs/pitfalls.md · 里程碑 docs/sim-plan.md · DEBUG.md

## 下一步
0. 【修法 c 达门 2026-07-07·CEO 裁定 c 先行已执行·蹭行验收面根治】
   改动=local_planner.launch:31 obstacleHeightThre **0.05→0.20**（掐灭地形代价阈值闪烁点火源）。
   真门=localPlanner obstacleHeightThre 活值 0.05（非上节记的 0.2=C++默认误抄）；噪声带实测 0.05-0.15。
   **叉子门 PASS**：cmd.x 占空 5.5%→91.4%、GT 实速 0.16→0.377 m/s、能到点、直立 100%；
   火源熄灭（前扇 cost>0.20 cell 恒 0、/path 空帧 65%→0）；避障反证 PASS（真障碍 cost>0.25>0.20 仍判障碍）。
   残余（诚实，非 c 能治，均 CEO gate）：world-pathdir std ~60°=argmax 无滞后（fix-a）；
   E0'' idle 位移 127mm（<50mm FAIL，较前轮 538-821mm 改善）=stale-path wz 爆发（W1-W3, navstack C++）。
   改动只在 refs 克隆（容器/ws 挂载，gitignore）；回滚=cp var/evidence/terrain_fix/local_planner.launch.orig
   +配对重启。零 planner/follower C++ 语义。全在 DEBUG.md 终节 + var/evidence/terrain_fix/。活栈 ALL-GREEN 留绿。
1. 【待令，不自行开火】fix-a（planner 选组滞后灭 argmax 抖，CEO gate）→ 之后 stale-path 清（W1-W3, CEO gate）
   → 二者除净后 E0'' 位移门<50mm 方可复验。工具 scripts/nav/pathdir_{sampler,analyze}.py。
2. 恢复③抓取（feat/grasp-wip 60%，迁 z-agent 收尾）· TARE 软停缺口（改栈源码=CEO gate）· P5.4 真机

## 裁决项（待 CEO）
- 修法 a（planner C++ 滞后）· stale-path 清（navstack C++）· 真机 verify 语义 / NUC vs Orin / unitree_sdk2_python
- rosm nuke 越权杀跨项目 ROS 进程，建议 scoped 修复（vector_os_nano 侧）
