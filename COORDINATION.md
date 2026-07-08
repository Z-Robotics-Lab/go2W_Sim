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
- 依据:CEO 委托。不 push。**sim 槽位已归还本会话(23:55)。**
- 释放条件:A/B 提交 + 上报后本节删除。
