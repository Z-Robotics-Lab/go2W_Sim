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
