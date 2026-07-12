# COORDINATION — go2w 多会话协调(单写者声明,覆盖式)

## 当前 claim(2026-07-07 23:55 更新)
- **fix-a 执行会话**(mystifying-cartwright):claim **降级为挂起**,**释放 sim 重启权**。
  - 保持持有:refs/.../local_planner/(localPlanner.cpp+launch)。迟滞码已入、参数门控、
    现值 groupSwitchRatio=**1.0(关=上游语义)**。**任何会话不要动 local_planner/该参数。**
  - 栈已留 office 场景 ALL-GREEN、robot 已 reset;sim 槽位归还给 extrinsic-friction 会话。
- **冲突记录(23:19-23:50,流程缺口非违规)**:fix-a A/B 两臂之间,bringup.sh(23:32,audit)
  与 warehouse_nav.py(23:38,extrinsic-friction,含默认翻 office+摩擦 combine_mode)被并行
  修改——均在各自 claim 范围内、且未碰活栈,但 fix-a 的重启加载了新文件 → **A/B 两臂地基不同,
  作废**。**教训成规**:共享 sim 上的 gate 实验,claim 必须连"拉起面文件"(bringup.sh、
  warehouse_nav.py、refs launch/config)一起冻结,不只是重启权与活栈。
- **fix-a 复验窗口(待排,CEO 排期)**:A线(地图倾斜根因)/B线(摩擦)收敛入账后,冻结拉起面
  ≥40min:2×配对重启+4 窗,预注册门见 DEBUG.md fix-a 节。届时在此重新 claim。

## 并行 claim(2026-07-07 · 新机就绪审计会话)
- **desktop-readiness 审计会话**持有(与 fix-a claim 无重叠):
  - assets/policies/(新增 · 策略权重入库)
  - scripts/nav/bringup.sh + restart_all.sh 的 GO2W_POLICY **默认路径**行(不动配对重启逻辑)
  - docs/user-manual.md · README.md(新机拉起序列文档)
- **不碰**:live sim(仅只读 test -f/md5sum 探针,无重启)、DEBUG.md、STATUS.md、local_planner。
- 依据:CEO 委托"新机就绪审计"(GitHub clone 后从零拉起)。不 push。
- 释放条件:审计提交完成后本节删除。

## 并行 claim(2026-07-07 · extrinsic+friction 会话 = 本任务)
- **extrinsic-friction 会话**持有(CEO 委托 A线雷达外参 + B线摩擦 两实测发现):
  - scripts/sim/warehouse_nav.py(摩擦材质 + GO2W_SCENE 默认翻 office;非红线区)
  - scripts/nav/ground_normal_probe.py(新 · 只读验证工具)
  - DEBUG_extrinsic_friction.md(本任务隔离 DEBUG,避开 fix-a 的 DEBUG.md 写锁)
  - scripts/nav/bringup.sh 的 **GO2W_SCENE 默认行(:217)** — 与 desktop-readiness 的
    GO2W_POLICY 默认行(:32)不同行、无重叠
  - docs/user-manual.md **§2.1.1 换场景段** — 与 desktop-readiness 的 clone/拉起序列段不同段
- **软重叠声明**:bringup.sh + user-manual.md 也在 desktop-readiness claim 内;本会话仅动上述
  精确不同行/段(CEO 默认翻 office 硬指令),不碰其 GO2W_POLICY 行 / clone 序列段。合并由 dispatcher。
- **不碰**:live sim 重启(现已归还,可配对重启到 office)、DEBUG.md、STATUS.md、
  local_planner、红线(render_interval/fullScan/pc2_to_livox/vector_sim.lock/go2w_policy/planner C++)。
- **SLAM 外参裁定**:livox_mid360_calibration.yaml 已正确(20° 已应用,live 实证)——**不改其值**。
- **已完成并释放 sim 槽位(2026-07-08 05:2x)**：A线达门(图倾斜 11.26°→1.65/1.72/1.81°<2°,
  init 定罪+修复+无漂移)·B线达门(slip≈0,转向回归零摔)·栈留 office ALL-GREEN(robot 静止)。
  详 DEBUG_extrinsic_friction.md。拉起面文件冻结解除。
- 后续:fix-a 复验窗(CEO 排期)可按其预注册门重新 claim。

## 冲突记录(2026-07-10 00:37-00:42,/piper/named_pose 双探针互污 — 流程缺口非违规)
- zmanip-m0-verify 会话(增益 K400/D15 复测)与另一并发验收循环(疑=母会话"go2w开发";
  nav_bridge.log [POSE] 行 972-1065,STOW/LOOKOUT/CARRY 周期循环)在 /piper/named_pose 交错:
  **sim 110-162 两边姿态窗口数据全废**(该时段任何 G-b/G-c 读数不可入账)。
- **教训成规**:主动指令话题(/piper/named_pose、grasp 触发等)与拉起面文件同级——发布前必须
  在此 claim;把"复测"交接给子会话后,母会话不得再自跑同通道探针。
- 复测已在安静通道完成(sim 232-272,foreign 守卫三窗全绿):**G-b PASS(LOOKOUT 峰值 1.97°<5°)
  + G-c 3/3 PASS(0.0374/0.0289/0.0240<0.05rad)**,K400/D15 入账,数字详 git commit +
  var/evidence/m0/。[RULING] sim 执行器 PD 整定非 CEO gate(不涉接口/依赖/安全/真机)。
  zmanip-m0-verify claim 已释放(sim 槽位归还,勿再有会话双驱 named_pose)。

## 并行 claim(2026-07-10 02:3x · z-manip-m05 workflow = 主编排会话代持)
- **z-manip-m05 workflow** 持有(M0.5 office 抓取测试角,CEO 已批;宿主唯一活动会话):
  - **sim 槽位**(配对重启权:teardown+bringup,office 默认)
  - **/piper/named_pose 发布权**(Verify 阶段 G-p6 证据帧需 LOOKOUT;按 00:37 冲突新规先 claim)
  - scripts/sim/warehouse_nav.py **SCENES props 段 + GT odom 多路发布块**(新增代码,不碰红线/摩擦/场景默认行)
  - ~/Desktop/z-manip tests/(PROPS 扩展)
- **不碰**:refs/ 源码、local_planner(fix-a 持有)、DEBUG*.md、红线参数。
- 释放条件:workflow 完成(gate 数字入 commit)后本节改"已释放"。届时链留 ALL-GREEN(CEO 眼见要求)。
- **02:4x 更新**:workflow 的 Verify 被 CEO 直令重启打断(TaskStop;旧布局 gate 数据不再入账)。
  claim 转由主编排会话继续持有,执行 CEO reach+立正裁定(托盘缩 XY/三件 Rx90° 立正/贴边一列/
  G-p7 立正 gate)后配对重启+复测;完成后释放,链留 ALL-GREEN。
- **已完成并释放(2026-07-10 03:1x)**:新布局配对重启达门——M0.5 props 18/18 PASS(位置贴点/
  三件立正 G-p7/reach 窄带合规/GT 四路≥4Hz-sim)+ 全量回归 20 pass 0 fail(M0 面无退化)。
  入账 go2w 09cf0cb+7009c57、z-manip 5e3a7c3(均已推)。链留 office ALL-GREEN(robot 静止
  LOOKOUT,CEO 眼见)。sim 槽位归还;/piper/named_pose 无人持有。

## 并行 claim(2026-07-10 03:2x · z-manip-m1 workflow = 主编排会话代持,CEO 开工令)
- **z-manip-m1 workflow** 持有(M1 find+SCAN+两段伺服进近+追踪+感知可视化):
  - **sim 槽位**(配对重启权)
  - **主动指令话题**:/piper/named_pose(LOOKOUT/SCAN 用)、/way_point(远段伺服)、
    **/manip/cmd_vel(新增)+/cmd_vel 仲裁面**(近段直控;仲裁在 warehouse_nav 消费端,
    manip 新鲜优先——绝不双写 /cmd_vel 原话题)
  - scripts/sim/warehouse_nav.py(cmd 仲裁 mux + 其余不动)、scripts/nav/sync_navstack_files.sh
    (rviz 生成器加感知三面板)、~/Desktop/z-manip 全仓(感知/伺服节点+容器+测试)
- **不碰**:local_planner(fix-a 持有)、refs/ 源码、agent_bridge.py(G7 全量后置 M4;若设计判定
  必须动则最小 diff 并在此记录)、红线参数、props 摆位(刚达门)。
- 释放条件:M1 gates 数字入 commit 后本节改"已释放";链留 ALL-GREEN。
- **已释放(2026-07-10 晚)**:M1 五层根因逐层实证收口(感知 PASS conf0.529/伺服 spoke 化/TABLE_VIEW −20.21°落地/RTF0.282/GPU9.6G),唯一遗留 BLOCKER=底盘原地 yaw ~2% 失效(frozen 链,CEO gate,待专项轮)。链留 ALL-GREEN idle,servo IDLE;槽位归还。

## 已释放(2026-07-12 · z-manip-m2 会话 = 抓取候选管道)
- **交付入账**:antipodal 后端+grasp 节点+单测 14/14+回归 107 过(z-manip 8356d5b/5a1b3fb/
  5ffb911;go2w 7992d4d rviz 面板)。**活链 gate = BLOCKED-BY-M1 如实入账不虚判**:管道行为
  全程正确(合格帧全为家具误锁,孔径/θ_app 正确拒绝);真罐从未入镜——底盘 yaw 死区(既有
  CEO gate)+ SLAM 旋转漂移(θ 11°→28°实测,fix-a 范围)+ SEARCH first-past-the-post 锁定
  三缺陷联合复现 6 次,卷宗见 z-manip 5ffb911。别名宽网误检已根修(canned food→高特异性)。
- **M3/M4 关键路径(CEO 决策输入)**:上述三缺陷不解则 M2 gate 数字/M3 抓起/M4 20 连跑全部
  无法稳定过门。链留 ALL-GREEN(robot 静止 STOW,l0-l5 全真);sim 槽位归还;
  /piper/named_pose、/manip/find、/way_point 无人持有。

## (原 claim 存档)2026-07-12 · z-manip-m2 会话(CEO"继续工作"令)
- **z-manip-m2 会话**持有(M2 = 几何 antipodal 通管道 + RViz 候选可视化,plan.md §5 M2 行):
  - ~/Desktop/z-manip 全仓(models/antipodal + grasp 节点 + tests + docker/perception/node.sh)
  - **zmanip-perception 容器重启权**(z-manip 自有容器,attach-only 消费者,重启不碰 Isaac/navstack 配对)
  - go2w scripts/nav/sync_navstack_files.sh **rviz 生成器段**(仅加 /manip/grasp/markers 面板)+
    容器内 rviz 进程重启(viewer,supervisor 自动拉回,非链)
  - **主动指令通道(按 00:37 冲突新规)**:/manip/find(触发 SEARCH/LOCK);Verify 期如需进近:
    servo 既有 /piper/named_pose、/way_point、/manip/cmd_vel(M1 机制,不新增写者)
  - 活链**只读**复用(45h ALL-GREEN 实测 l0-l5 全真);**无配对重启计划**(不改 warehouse_nav/拉起面)
- **[RULING] M2 无 CEO gate 项**:/manip/grasp/{candidates,markers,status} 全用标准 msg
  (PoseArray/MarkerArray/String-JSON),沿 M1 /manip/* 命名空间先例,z-manip 内部 L1→viz 数据流,
  消费端(M3 grasp_exec)同仓——非新 msg/srv/action、非跨包契约变更;实现纯 numpy **零新外部依赖**;
  HGGD(需 sm_120 重编实测=风险项)本轮不接,留后续轮。
- **不碰**:local_planner(fix-a 持有)、refs/、DEBUG*.md、红线、warehouse_nav.py、props 摆位。
- 释放条件:M2 机器 gate 数字入 commit 后本节改"已释放";链留 ALL-GREEN。
