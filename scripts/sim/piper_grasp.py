# PiPER 抓取控制器：阻尼最小二乘 IK 伺服 + 顶抓状态机（跑在 Isaac 主循环里）。
#
# 设计：导航/接近由外部（agent 技能层）负责；本控制器假定目标已在臂的工作域内，
# 只管 臂 8 关节（6 转动 + 2 指爪）的位置目标。每个伺服拍（50Hz）：
#   PREGRASP: EE 到目标正上方 -> DESCEND: 下探跨骑 -> CLOSE: 合爪(靠被撑开判抓住)
#   -> LIFT: 直上提起 -> done（保持握持）；任一阶段超时 -> failed + 收臂
# 雅可比索引约定抄 IsaacLab task_space_actions.py：浮动基座 body 索引不减 1、
# 关节列 +6（前 6 列是基座 DOF）。
import torch

import isaaclab.utils.math as math_utils

_ARM_EXPR = "piper_joint[1-6]"
_GRIP_EXPR = "piper_joint[78]"
_EE_BODY = "piper_gripper_base"
_ARMBASE_BODY = "piper_base_link"

# 几何/行为常量（首测后按 GT 实测微调）
GRIP_FRAME_OFFSET = 0.17   # gripper_base 原点沿 +Z 到指垫夹持中心的距离（指爪根 0.1358 + 指长）
PREGRASP_DZ = 0.20         # 预抓取点在箱心上方
GRASP_DZ = 0.0             # 夹持中心对准箱心（EE 深度由 GRIP_FRAME_OFFSET 吃掉）
LIFT_DZ = 0.35             # 提起高度（oracle 的 lifted 阈值是 0.10，留足余量）
REACH_MAX = 0.58           # 目标与臂基座的水平距离上限（PiPER 半径 ~0.62）
OPEN_POS = (0.035, -0.035)
CLOSED_POS = (0.0, 0.0)
HOLD_APERTURE_MIN = 0.014  # 合爪指令下实测开度仍大于此 -> 指间有物（6cm 箱约 0.05+）
DQ_MAX = 0.02              # 每伺服拍关节步长上限（50Hz -> 1 rad/s）
DLS_LAMBDA = 0.05
POS_TOL = 0.03
# 阶段超时（仿真秒）
_TIMEOUTS = {"PREGRASP": 10.0, "DESCEND": 8.0, "CLOSE": 2.0, "LIFT": 8.0}


class PiperGraspController:
    """臂关节目标的唯一属主。status: idle|running:<PHASE>|done|failed:<why>"""

    def __init__(self, robot, device: str) -> None:
        self._robot = robot
        self._device = device
        self.arm_ids, self.arm_names = robot.find_joints(_ARM_EXPR)
        self.grip_ids, self.grip_names = robot.find_joints(_GRIP_EXPR)
        self.all_ids = list(self.arm_ids) + list(self.grip_ids)
        ee_ids, _ = robot.find_bodies(_EE_BODY)
        ab_ids, _ = robot.find_bodies(_ARMBASE_BODY)
        self._ee_idx = ee_ids[0]
        self._ab_idx = ab_ids[0]
        # 浮动基座：jacobian body 索引=body_idx，关节列 +6
        self._jac_cols = [6 + int(j) for j in self.arm_ids]
        # 目标张量（8 关节），从默认姿态初始化——控制器从此刻起独占臂目标
        dp = robot.data.default_joint_pos[0]
        self.q_tgt = dp[self.all_ids].clone().to(device)
        lim = robot.data.joint_pos_limits[0]  # (n_joints, 2)
        self._q_lo = lim[self.all_ids, 0].to(device)
        self._q_hi = lim[self.all_ids, 1].to(device)
        self._tuck = self.q_tgt.clone()  # 收臂姿态 = URDF 默认（折叠）

        self.status = "idle"
        self._phase = None
        self._t_phase = 0.0
        self._target = None      # 箱心（世界系, torch (3,)）
        self._p_des = None       # 当前阶段的夹持中心目标点（世界系）
        self._lift_from = None
        print(f"[GRASP] ctrl ready arm={self.arm_names} grip={self.grip_names} "
              f"ee_body_idx={self._ee_idx}", flush=True)

    # ------------------------------------------------------------------ 读数
    def ee_pose(self):
        """gripper_base 世界位姿 (pos(3,), quat_wxyz(4,))。"""
        return (self._robot.data.body_pos_w[0, self._ee_idx],
                self._robot.data.body_quat_w[0, self._ee_idx])

    def grip_center(self):
        """指垫夹持中心（世界系）= gripper_base + R @ (0,0,GRIP_FRAME_OFFSET)。"""
        p, q = self.ee_pose()
        off = torch.tensor([0.0, 0.0, GRIP_FRAME_OFFSET], device=p.device)
        return p + math_utils.quat_apply(q, off)

    def aperture(self) -> float:
        jp = self._robot.data.joint_pos[0]
        return float(jp[self.grip_ids[0]] - jp[self.grip_ids[1]])  # j7 - j8 >= 0

    def cmd_closed(self) -> bool:
        g = self.q_tgt[len(self.arm_ids):]
        return bool(abs(float(g[0])) + abs(float(g[1])) < 0.005)

    # ------------------------------------------------------------------ 控制
    def start(self, target_xyz) -> bool:
        """请求抓取 target（世界系箱心）。工作域外立即 failed:unreachable。"""
        t = torch.as_tensor(target_xyz, dtype=torch.float32,
                            device=self._robot.data.body_pos_w.device)
        ab = self._robot.data.body_pos_w[0, self._ab_idx]
        horiz = float(torch.norm(t[:2] - ab[:2]))
        if horiz > REACH_MAX or horiz < 0.08:
            self.status = f"failed:unreachable(d={horiz:.2f})"
            print(f"[GRASP] {self.status}", flush=True)
            return False
        self._target = t
        self._set_grip(OPEN_POS)
        self._enter("PREGRASP", t + torch.tensor([0.0, 0.0, PREGRASP_DZ], device=t.device))
        print(f"[GRASP] start target=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f}) d={horiz:.2f}",
              flush=True)
        return True

    def _enter(self, phase: str, p_des) -> None:
        self._phase = phase
        self._p_des = p_des
        self._t_phase = 0.0
        self.status = f"running:{phase}"

    def _set_grip(self, pos) -> None:
        n = len(self.arm_ids)
        self.q_tgt[n] = pos[0]
        self.q_tgt[n + 1] = pos[1]

    def _fail(self, why: str) -> None:
        self.status = f"failed:{why}"
        self._phase = None
        self._set_grip(OPEN_POS)
        self.q_tgt[: len(self.arm_ids)] = self._tuck[: len(self.arm_ids)]
        print(f"[GRASP] {self.status}", flush=True)

    def step(self, dt: float) -> None:
        """一个伺服拍（外层按 50Hz 调用；dt = 该拍覆盖的仿真秒）。"""
        if self._phase is None:
            return
        self._t_phase += dt
        if self._t_phase > _TIMEOUTS[self._phase]:
            if self._phase == "LIFT" and self.aperture() > HOLD_APERTURE_MIN:
                self.status = "done"  # 提起超时但仍握持（收敛差点）——算成
                self._phase = None
                print(f"[GRASP] done (lift timeout, still holding "
                      f"ap={self.aperture():.3f})", flush=True)
                return
            self._fail(f"timeout:{self._phase}")
            return

        if self._phase == "CLOSE":
            if self._t_phase > 1.2:
                if self.aperture() > HOLD_APERTURE_MIN:
                    self._lift_from = self.grip_center().clone()
                    self._enter("LIFT", self._lift_from
                                + torch.tensor([0.0, 0.0, LIFT_DZ],
                                               device=self._lift_from.device))
                else:
                    self._fail(f"empty_close(ap={self.aperture():.3f})")
            return

        # 伺服阶段（PREGRASP / DESCEND / LIFT）
        err = self._servo_tick()
        if err is None:
            return
        tol = POS_TOL if self._phase != "LIFT" else 0.05
        if err < tol:
            if self._phase == "PREGRASP":
                self._enter("DESCEND", self._target
                            + torch.tensor([0.0, 0.0, GRASP_DZ],
                                           device=self._target.device))
            elif self._phase == "DESCEND":
                self._phase = "CLOSE"
                self._t_phase = 0.0
                self.status = "running:CLOSE"
                self._set_grip(CLOSED_POS)
            elif self._phase == "LIFT":
                if self.aperture() > HOLD_APERTURE_MIN:
                    self.status = "done"
                    print(f"[GRASP] done ap={self.aperture():.3f}", flush=True)
                else:
                    self._fail("dropped")
                self._phase = None

    def _servo_tick(self):
        """一步 DLS-IK：更新 q_tgt，返回夹持中心位置误差（米）。"""
        p_gc = self.grip_center()
        _, q_ee = self.ee_pose()
        # 期望姿态：+Z 指向世界 -Z（顶抓）；X 取臂基座->目标的水平方向（腕滚≈0）
        ab = self._robot.data.body_pos_w[0, self._ab_idx]
        u = self._target[:2] - ab[:2]
        n = torch.norm(u)
        u = u / n if n > 1e-4 else torch.tensor([1.0, 0.0], device=u.device)
        R_des = torch.tensor([[u[0], u[1], 0.0],
                              [u[1], -u[0], 0.0],
                              [0.0, 0.0, -1.0]], device=u.device).t()
        q_des = math_utils.quat_from_matrix(R_des.unsqueeze(0))[0]

        e_pos = self._p_des - p_gc
        q_err = math_utils.quat_mul(q_des.unsqueeze(0),
                                    math_utils.quat_conjugate(q_ee.unsqueeze(0)))
        e_rot = math_utils.axis_angle_from_quat(q_err)[0]
        e = torch.cat([e_pos, 0.5 * e_rot])  # 姿态误差降权，位置优先

        jac = self._robot.root_physx_view.get_jacobians()
        J = jac[0, self._ee_idx][:, self._jac_cols]  # (6, 6)
        # 夹持中心雅可比修正：J_p += -skew(r) 作用于角速度列（r = R@offset）
        r = math_utils.quat_apply(
            q_ee, torch.tensor([0.0, 0.0, GRIP_FRAME_OFFSET], device=q_ee.device))
        skew = torch.tensor([[0.0, -r[2], r[1]],
                             [r[2], 0.0, -r[0]],
                             [-r[1], r[0], 0.0]], device=r.device)
        J = J.clone()
        J[:3] = J[:3] - skew @ J[3:]

        JJt = J @ J.t() + (DLS_LAMBDA ** 2) * torch.eye(6, device=J.device)
        dq = J.t() @ torch.linalg.solve(JJt, e)
        dq = torch.clamp(dq, -DQ_MAX, DQ_MAX)

        n_arm = len(self.arm_ids)
        q_meas = self._robot.data.joint_pos[0, self.arm_ids].to(dq.device)
        self.q_tgt[:n_arm] = torch.clamp(q_meas + dq,
                                         self._q_lo[:n_arm], self._q_hi[:n_arm])
        return float(torch.norm(e_pos))
