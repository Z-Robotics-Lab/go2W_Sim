#!/usr/bin/env python3
"""M1 断崖取证：shim 观测输入/动作输出 dump（只读旁路，红线 go2w_policy.py 不改）。

用途：把 Go2WPolicy shim 每个 policy step 实际吃到的 57 维 obs 与吐出的 16 维 act
逐拍追加写 jsonl，供 live 部署链 vs 评估态离线对拍——定位"同 policy+shim，评估 wz≈0.98
但 live wz≈0"是否源于 **shim 收到的观测输入在 live 链上异常**（输入源/坐标系/量纲）。

设计约束（AGENTS 红线 + 诚实取证）：
  - go2w_policy.py 是红线，不改：本模块在**调用方**旁路重建 obs，用的是 shim 自己的
    公开常量（从 go2w_policy 导入 SCALE_ANG/SCALE_JVEL/OBS_DIM/ACT_DIM）与 shim 自己的
    实例态（policy.ids / policy.default_pos / policy.last_action），逐位复刻 act() 内
    73-81 行的拼装公式 —— 因此 dump 出的 obs 与 shim 真喂进网络的 obs **逐位一致**，
    且若 shim 改了 scale，本 dump 自动跟随（导入的是同一常量），不产生保真漂移。
  - obs 必须在 act() **之前**采（此刻 policy.last_action 正是 shim 本拍将用的旧值）；
    act 的返回动作在 act() **之后**采。调用方按此时序调用（见 dump_step docstring）。
  - 限频防爆盘：GO2W_OBS_DUMP_HZ（默认 50，=每 policy 拍都记）设采样上限；两拍最小
    sim-dt 间隔不足则跳过该拍（长跑不撑爆磁盘）。GO2W_OBS_DUMP_MAX_ROWS 硬顶（默认 20000）。
  - 环境开关：GO2W_OBS_DUMP=<路径> 存在才启用；未设=零开销、零行为改变（旁路完全 no-op）。

诚实性：本模块只 **读** robot.data / policy 公开态并写盘，绝不 set 任何 target、绝不改
policy.last_action、绝不 step 物理。对被测链的物理行为零影响（纯观测）。
"""
import json
import os

import torch

# 从红线 shim 导入其自身常量与拼装配方所需符号（同一真源，保真不漂移）。
from go2w_policy import OBS_DIM, ACT_DIM, SCALE_ANG, SCALE_JVEL


class ObsDumper:
    """Env-gated, rate-limited, read-only shim obs/act recorder.

    构造：ObsDumper.maybe(chain_tag, sim_t_getter) —— 未设 GO2W_OBS_DUMP 返回 None
    （调用方用 `if dumper: dumper.record(...)` 完全跳过，零开销）。
    """

    def __init__(self, path: str, chain: str, hz: float, max_rows: int, sim_t_getter):
        self.path = path
        self.chain = chain
        self.min_dt = (1.0 / hz) if hz > 0 else 0.0
        self.max_rows = max_rows
        self._sim_t = sim_t_getter
        self._n = 0
        self._last_t = -1e18
        self._fh = open(path, "a")  # append：多段/多 ckpt 累积到同一 dump（增量落盘）
        print(f"[OBSDUMP] {chain} -> {path} (hz<={hz}, max_rows={max_rows})", flush=True)

    @classmethod
    def maybe(cls, chain: str, sim_t_getter):
        """未设 GO2W_OBS_DUMP => None（旁路完全禁用）。"""
        path = os.environ.get("GO2W_OBS_DUMP")
        if not path:
            return None
        hz = float(os.environ.get("GO2W_OBS_DUMP_HZ", "50"))
        max_rows = int(os.environ.get("GO2W_OBS_DUMP_MAX_ROWS", "20000"))
        return cls(path, chain, hz, max_rows, sim_t_getter)

    def _rebuild_obs(self, policy, cmd_vx, cmd_vy, cmd_wz):
        """逐位复刻 go2w_policy.Go2WPolicy.act() 73-81 的 obs 拼装（红线不改，此处只读重建）。

        与 shim 用**同一** policy.ids / policy.default_pos / policy.last_action / 同一
        robot.data，故 obs 逐位一致。返回 (obs57 list, 诊断 dict)。
        """
        r = policy.robot.data
        ang = r.root_ang_vel_b[:1] * SCALE_ANG              # (1,3) 本体系角速度*0.25
        grav = r.projected_gravity_b[:1]                    # (1,3) 本体系重力投影
        cmd = torch.tensor([[max(-1., min(1., cmd_vx)),
                             max(-1., min(1., cmd_vy)),
                             max(-1., min(1., cmd_wz))]], device=policy.device)
        jpos = (r.joint_pos[:1, policy.ids] - policy.default_pos[:1]).clone()
        jpos[:, 12:] = 0.0                                  # joint_pos_rel_without_wheel
        jvel = r.joint_vel[:1, policy.ids] * SCALE_JVEL
        obs = torch.cat([ang, grav, cmd, jpos, jvel, policy.last_action], dim=1)
        assert obs.shape[1] == OBS_DIM, f"obs 维度 {obs.shape[1]} != {OBS_DIM}"
        # 诊断字段：原始信号（未经 shim scale）+ 世界系角速度，供对拍分辨"输入源/坐标系"异常。
        diag = {
            # 本体系角速度（shim 观测源；乘 0.25 前的原始 rad/s）
            "ang_vel_b": [round(v, 6) for v in r.root_ang_vel_b[0].tolist()],
            # 世界系角速度（GT，act 里没用；z 分量=评估 3b 判据的真 yaw rate）
            "ang_vel_w": [round(v, 6) for v in r.root_ang_vel_w[0].tolist()],
            # 本体系重力投影（姿态；up_z=proj_gravity_b.z，站立≈-1）
            "proj_grav_b": [round(v, 6) for v in r.projected_gravity_b[0].tolist()],
            # 世界系线速度（vx/vy/wz 好坏的运动真值）
            "lin_vel_w": [round(v, 6) for v in r.root_lin_vel_w[0].tolist()],
            # root 世界四元数 (w,x,y,z)，姿态/坐标系核对
            "quat_w": [round(v, 6) for v in r.root_quat_w[0].tolist()],
            # shim 收到的 cmd（clip 后，就是 obs 命令槽）
            "cmd_clip": [round(max(-1., min(1., c)), 6)
                         for c in (cmd_vx, cmd_vy, cmd_wz)],
            # shim 拿到的原始 cmd（clip 前，看调用方是否已 ramp/仲裁改动过）
            "cmd_raw": [round(cmd_vx, 6), round(cmd_vy, 6), round(cmd_wz, 6)],
        }
        return [round(v, 6) for v in obs[0].tolist()], diag

    def record(self, policy, cmd_vx, cmd_vy, cmd_wz, act_out=None, extra=None):
        """一步式记录 —— 保真前提：必须在 policy.act() **之前**调用。

        原因（时序铁律）：shim act() 内 obs 用的是 **旧** last_action（上拍动作），采完 obs
        才把 last_action 覆盖成本拍新动作。故要复刻 shim 真喂进网络的 obs，obs 必须在
        act() 之前重建（此刻 policy.last_action 尚是旧值）。

        act_out：本拍动作（16 维）。因为要在 act() 前调用，本拍动作此刻还没算出，故：
          - live/eval 两链都传 act_out=None（一步式，只在 act 前采 obs），本拍动作靠**下一拍**
            的 obs.last_action 承载对拍已足够；或
          - 调用方想同拍带 act，则改用 begin()/finish() 两段式（见下）。
        限频：距上次记录 sim-t 不足 min_dt 则跳过；超 max_rows 硬停。
        返回 True=已记录（可用于两段式的 finish 判定），False=被限频/封顶跳过。
        """
        if self._n >= self.max_rows:
            return False
        t = self._sim_t()
        if (t - self._last_t) < self.min_dt:
            return False
        self._last_t = t
        obs, diag = self._rebuild_obs(policy, cmd_vx, cmd_vy, cmd_wz)
        row = {"chain": self.chain, "sim_t": round(t, 5), "i": self._n,
               "obs": obs, **diag}
        if act_out is not None:
            row["act"] = [round(v, 6) for v in act_out[0].tolist()]
        if extra:
            row.update(extra)
        self._fh.write(json.dumps(row) + "\n")
        self._fh.flush()
        self._n += 1
        return True

    # ---- 两段式（想同拍带 act 时用；obs 在 act 前采，act 在 act 后补）----
    def begin(self, policy, cmd_vx, cmd_vy, cmd_wz, extra=None):
        """act() 之前调用：重建并暂存本拍 obs（用旧 last_action，逐位对齐 shim）。
        返回 pending 句柄（dict）或 None（被限频/封顶）。"""
        if self._n >= self.max_rows:
            return None
        t = self._sim_t()
        if (t - self._last_t) < self.min_dt:
            return None
        self._last_t = t
        obs, diag = self._rebuild_obs(policy, cmd_vx, cmd_vy, cmd_wz)
        row = {"chain": self.chain, "sim_t": round(t, 5), "i": self._n,
               "obs": obs, **diag}
        if extra:
            row.update(extra)
        return row

    def finish(self, pending, act_out):
        """act() 之后调用：把本拍动作补进 pending 行并落盘。pending=None 则 no-op。"""
        if pending is None:
            return
        if act_out is not None:
            pending["act"] = [round(v, 6) for v in act_out[0].tolist()]
        self._fh.write(json.dumps(pending) + "\n")
        self._fh.flush()
        self._n += 1

    def close(self):
        try:
            self._fh.close()
        except Exception as e:  # 收尾失败留痕，绝不静默吞（coding-style）
            print(f"[OBSDUMP][WARN] close failed: {e}", flush=True)
