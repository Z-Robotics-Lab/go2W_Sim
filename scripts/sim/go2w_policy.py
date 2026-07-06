#!/usr/bin/env python3
"""robot_lab Go2W 速度策略的部署侧推理——真机 wheeled_sport 的仿真等价物。

观测/动作规格逐位对齐 robot_lab v2.3.2 的
RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0（flat: 无 height_scan）：
  obs(57) = [ang_vel_b*0.25(3), proj_gravity(3), cmd(3),
             joint_pos_rel*1.0(16, 轮位清零), joint_vel_rel*0.05(16), last_action(16)]
  （policy 组无 base_lin_vel——特权观测只进 critic，rough_env_cfg.py:94）
  act(16) = 腿位置 = default + 0.25*a[:12]（FR,FL,RR,RL 各 hip/thigh/calf）
            轮速度 = 5.0*a[12:16]（FR,FL,RR,RL foot）
执行器增益必须用训练态（腿 25/0.5，轮 0/0.5——UNITREE_GO2W_CFG），策略 50Hz。
指令范围（训练分布）：vx,vy,wz ∈ [-1,1]。
"""
import torch
import torch.nn as nn

import isaaclab.utils.math as math_utils

LEG_JOINTS = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]
WHEEL_JOINTS = ["FR_foot_joint", "FL_foot_joint", "RR_foot_joint", "RL_foot_joint"]
JOINT_NAMES = LEG_JOINTS + WHEEL_JOINTS

OBS_DIM, ACT_DIM = 57, 16
SCALE_LIN, SCALE_ANG, SCALE_JVEL = 2.0, 0.25, 0.05
# 训练动作 scale 分关节：hip=0.125、thigh/calf=0.25（robot_lab rough_env_cfg
# 的 scale 字典，训练快照 env.yaml 复核；vx-audit M1 坐实此前统一 0.25 让
# 4 个 hip 的目标位移是训练态 2 倍——姿态扰动、步态失真）。LEG_JOINTS 序中
# hip 位于 0,3,6,9。
ACT_SCALE_WHEEL = 5.0
LEG_ACT_SCALE = [0.125 if i % 3 == 0 else 0.25 for i in range(12)]


class Go2WPolicy:
    def __init__(self, ckpt_path: str, robot, device: str):
        self.device = device
        self.robot = robot
        ids, names = robot.find_joints(JOINT_NAMES, preserve_order=True)
        assert list(names) == JOINT_NAMES, f"关节顺序不符: {names}"
        self.ids = ids
        self.leg_ids = ids[:12]
        self.wheel_ids = ids[12:]
        self.default_pos = robot.data.default_joint_pos[:, ids].clone()

        # rsl_rl 3.1.2 ActorCritic: actor = MLP 60-512-256-128-16, ELU
        self.net = nn.Sequential(
            nn.Linear(OBS_DIM, 512), nn.ELU(),
            nn.Linear(512, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(),
            nn.Linear(128, ACT_DIM),
        ).to(device)
        ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
        sd = ckpt["model_state_dict"]
        actor_sd = {k.replace("actor.", "", 1): v for k, v in sd.items()
                    if k.startswith("actor.")}
        missing, unexpected = self.net.load_state_dict(actor_sd, strict=False)
        loadable = len(actor_sd) - len(unexpected)
        assert loadable >= 8, f"actor 权重装载异常: missing={missing} unexpected={unexpected}"
        self.net.eval()
        self.last_action = torch.zeros(1, ACT_DIM, device=device)
        self.leg_act_scale = torch.tensor([LEG_ACT_SCALE], device=device)
        print(f"[POLICY] loaded {ckpt_path} (iter {ckpt.get('iter', '?')}), "
              f"joints={list(names)}")

    @torch.no_grad()
    def act(self, cmd_vx: float, cmd_vy: float, cmd_wz: float):
        """返回 (leg_ids, leg_pos_targets, wheel_ids, wheel_vel_targets)。50Hz 调用。"""
        r = self.robot.data
        ang = r.root_ang_vel_b[:1] * SCALE_ANG
        grav = r.projected_gravity_b[:1]
        cmd = torch.tensor([[max(-1., min(1., cmd_vx)),
                             max(-1., min(1., cmd_vy)),
                             max(-1., min(1., cmd_wz))]], device=self.device)
        jpos = (r.joint_pos[:1, self.ids] - self.default_pos[:1]).clone()
        jpos[:, 12:] = 0.0  # joint_pos_rel_without_wheel: 轮位清零
        jvel = r.joint_vel[:1, self.ids] * SCALE_JVEL
        obs = torch.cat([ang, grav, cmd, jpos, jvel, self.last_action], dim=1)
        a = self.net(obs)
        self.last_action = a.clone()
        leg_targets = self.default_pos[:1, :12] + self.leg_act_scale * a[:, :12]
        wheel_vels = ACT_SCALE_WHEEL * a[:, 12:]
        return self.leg_ids, leg_targets, self.wheel_ids, wheel_vels
