# RL locomotion 策略权重（git 追踪，新机 clone 即用）

Go2W 速度策略 checkpoint（rsl_rl PPO，actor MLP 57-512-256-128-16 ELU）——部署侧由
`scripts/sim/go2w_policy.py` 加载（只读 `model_state_dict` 的 `actor.*`）。

## 为什么入库
`robot_lab/` 是 gitignored 的 vendored 检出（`clone_deps.sh` 重新拉取，训练产物 `logs/`
不随之而来）。若 `bringup.sh`/`restart_all.sh` 的 `GO2W_POLICY` 默认仍指向
`robot_lab/logs/rsl_rl/.../model_*.pt`，新机 clone 后该文件不存在，bringup 的
`test -f $POLICY` 守卫会拒启。故把现役 ckpt + 两个回滚锚放进这里（git 追踪）。

容器把整个仓库 bind-mount 到 `/workspace/go2w`（`setup_container.sh` 的
`-v $REPO:/workspace/go2w`），所以这里的文件在容器内路径
`/workspace/go2w/assets/policies/...` 直接可见——**无需拷贝、无需重建镜像**。

每个 ckpt 约 4.6MB（普通 torch zip，非 LFS）。附 `params/{env,agent}.yaml` 训练快照，
记录 obs/act 规格与执行器增益（复现/审计用；加载时不读，仅溯源）。

## 目录
| 目录 | ckpt | 角色 | 体重锚 |
|---|---|---|---|
| `go2w_flat_payload_5495/` | `model_5495.pt` | **现役默认**（载荷轮，⑤门形修正后全门过） | 6.5kg 载荷 |
| `go2w_flat_payload_3497/` | `model_3497.pt` | 回滚锚（载荷轮前默认；6.46kg 新体重锚 ①0.0049/④0.0337+0.0063 验过） | 6.46kg |
| `go2w_flat_factory_1999/` | `model_1999.pt` | 出厂锚（robot_lab v2.3.2 go2w flat 原始 2000 iters；6.92kg 裸躯干训练——载荷不匹配根因，见 docs/sim-plan.md） | 6.92kg 裸躯干 |

切换/裁定记录见 `docs/sim-plan.md`（策略重训段）与 `scripts/sim/retrain_runbook.md`。
换新 ckpt：训练+`scripts/sim/policy_acceptance.py` 全门过后，把 ckpt（连同 `params/`）
拷进本目录新子目录，改 `bringup.sh`/`restart_all.sh` 的 `GO2W_POLICY` 默认路径。

## 覆盖
运行时可 `export GO2W_POLICY=<容器内绝对路径>` 覆盖默认（如仍想用 robot_lab/logs 下的
训练输出做 A/B）。
