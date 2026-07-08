# STATUS — go2W_Sim 会话恢复锚点（覆盖式，≤40 行）

更新：2026-07-07（Office 场景迁移完成，活栈=office）。仓库：github.com/Z-Robotics-Lab/go2W_Sim。
分支 feat/go2w-office-scene（未 push）。姐妹仓：z-agent（agent 运行时，fork 自 vector-os-nano）。

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
0. 【Office 场景迁移完成 2026-07-07·分支 feat/go2w-office-scene·活栈=OFFICE 留给 CEO 测】
   **GO2W_SCENE 参数化**（config 非 code）：warehouse_nav.py 加 SCENES 字典（usd/spawn/box/cam），
   GO2W_SCENE 未设=warehouse 逐字节等价；=office 选 Office/office.usd。bringup 透传 GO2W_SCENE。
   开机：`GO2W_SCENE=office bash scripts/nav/bringup.sh [teardown|up]`（配对重启，坑42/43）。
   **office spawn 校准 (-2.5,-5.0) 开阔厅**（原点是 reception 拥挤角落；/terrain_map 实证）。
   **门验收（DEBUG 终节数字表）**：a PASS（green+直立+相机拉起即见狗）· b PASS（占空 84.9%/
   GT 0.40/直立 100%/空路径率 8%=fix-c obstacleHeightThre 泛化 office 地毯成功）· c PASS（穿
   0.12m 净空窄缝、连续、不翻）· d PASS（explore 体积 +278m³/不冻/不翻，限制=robot 徘徊 spawn±0.4m）·
   e PASS（RTF 0.20≈仓库，无红利）· g PASS（默认等价，diff 纯加性）· **f FAIL**（zeno E2E 物理到点
   arrived+held d=0.11 但 verdict verified=False——到点后 idle 漂移不 hold，根因=已入册 fix-a/W1-W3
   CEO-gate 残余，非迁移缺陷）。零红线改动。回滚=git 或 GO2W_SCENE 不设。
1. 【待令，不自行开火·门 f 依赖】fix-a（planner argmax 滞后）+ stale-path 清（W1-W3, navstack C++）
   ——除净后 idle 不漂移，门 f E2E 才能 verified=True。工具 scripts/nav/pathdir_{sampler,analyze}.py。
2. 恢复③抓取（feat/grasp-wip 60%；office 箱已随 spawn 迁 (-1.5,-5.0)）· TARE 软停缺口· P5.4 真机

## 裁决项（待 CEO）
- 修法 a（planner C++ 滞后）· stale-path 清（navstack C++）· 真机 verify 语义 / NUC vs Orin / unitree_sdk2_python
- rosm nuke 越权杀跨项目 ROS 进程，建议 scoped 修复（vector_os_nano 侧）
