# COORDINATION — go2w 多会话协调(单写者声明,覆盖式)

## 当前 claim(2026-07-07 23:15)
- **fix-a 执行会话**(vector_os_nano worktree mystifying-cartwright)持有:
  - go2w sim(go2w-isaac + navstack 容器,含配对重启权)
  - refs/.../local_planner/(localPlanner.cpp + launch)+ 容器内 colcon 构建
  - DEBUG.md / STATUS.md 追加与覆盖
- 依据:CEO [RULING] 2026-07-07 批准修法 a(A1 参数门控滞后),本会话执行。
- 其他会话:在本 claim 释放前**不要**重启 sim、不要写 DEBUG/STATUS、不要动 local_planner。
- 释放条件:fix-a 验收(达门或停手上报)后,本节改写为"无 claim"。

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
- **不碰**:live sim 重启(fix-a 持有;仅只读订阅探针,无重启/无 kill)、DEBUG.md、STATUS.md、
  local_planner、红线(render_interval/fullScan/pc2_to_livox/vector_sim.lock/go2w_policy/planner C++)。
- **SLAM 外参裁定**:livox_mid360_calibration.yaml 已正确(20° 已应用,live 实证)——**不改其值**。
- 依据:CEO 委托。不 push。sim 验证待 fix-a 归还槽位后配对重启到 office。
- 释放条件:A/B 提交 + 上报后本节删除。
