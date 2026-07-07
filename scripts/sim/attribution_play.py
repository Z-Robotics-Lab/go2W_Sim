#!/usr/bin/env python3
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
"""E-T: attribution experiment — run the SHIPPED ckpt in robot_lab's OWN training env.

Goal: split the zero-command creep cause between the TRAINING recipe and our DEPLOYMENT shim
(scripts/sim/go2w_policy.py). The A/B (policy_acceptance.py) ran BOTH bodies through our shim,
so a residual shim mismatch would corrupt both equally and cannot be distinguished from a policy
defect. This script runs the ckpt through robot_lab's NATIVE pipeline — native USD asset, native
ImplicitActuator gains, native obs manager, native rsl_rl normalizer+actor — i.e. ZERO shim.

Method (command forcing, obs-clean):
  We overwrite the LIVE command term so `vel_command_b` is pinned to a forced value every step.
  The policy observation reads the command via `env.command_manager.get_command("base_velocity")`,
  which returns exactly this tensor — so forcing it IS the native obs pipeline seeing that command,
  identical to how a standing env (is_standing_env) is fed a zero command in training. We patch
  `_resample_command` and `_update_command` to no-ops that re-pin the value, so neither the norm<0.2
  threshold, the heading logic, nor the pit-terrain override can perturb it. Nothing downstream of
  the command tensor is touched, so scaling / obs order / normalizer are all the untouched training
  path. => the command is clean; the obs is the training obs.

GT read: `env.scene["robot"].data.root_lin_vel_w` (world-frame root linear velocity, ground truth
the actor cannot author) and root position `root_pos_w`. Drift speed = net planar displacement /
elapsed sim-time; also report mean instantaneous planar speed.

Segments (defaults): zero-cmd 30 s; then vx=0.15/0.30/0.60 for 15 s each. Each segment resets the
env first (native reset), lets it settle, then measures over the window. num_envs small (default 4);
we read env 0 as the reported robot but log the mean over all envs too (robustness).

Runs INSIDE the go2w-isaac container (imports isaaclab). Headless. One JSON row per segment to --out.
"""
import argparse
import json
import os
import sys

# robot_lab's rsl_rl cli_args helper lives next to play.py; make the import cwd-independent.
_RSL_RL_DIR = "/workspace/go2w/robot_lab/scripts/reinforcement_learning/rsl_rl"
if _RSL_RL_DIR not in sys.path:
    sys.path.insert(0, _RSL_RL_DIR)

from isaaclab.app import AppLauncher

import cli_args  # noqa: E402  # isort: skip  (robot_lab rsl_rl cli helper)

parser = argparse.ArgumentParser(description="E-T attribution: shipped ckpt in native robot_lab env.")
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--task", type=str, default="RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--zero_secs", type=float, default=30.0, help="zero-cmd measure window (sim-s)")
parser.add_argument("--ladder_secs", type=float, default=15.0, help="per-vx measure window (sim-s)")
parser.add_argument("--settle_secs", type=float, default=2.0, help="post-reset settle before measuring")
parser.add_argument("--out", type=str, default=None, help="append JSONL rows here")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---- rest follows (after sim app boot) ----
import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.utils.assets import retrieve_file_path  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils import get_checkpoint_path  # noqa: E402
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402

import robot_lab.tasks  # noqa: F401, E402  # isort: skip


def _log(row, out_path):
    print("[E-T] " + json.dumps(row), flush=True)
    if out_path:
        with open(out_path, "a") as f:
            f.write(json.dumps(row) + "\n")


def _force_command(env, vx, vy, wz, device):
    """Pin the live command term's vel_command_b to (vx,vy,wz) and neutralize resample/update.

    Returns a restore() closure (not strictly needed; env torn down after run)."""
    cm = env.command_manager
    term = cm.get_term("base_velocity")
    n = term.vel_command_b.shape[0]
    forced = torch.tensor([[vx, vy, wz]], device=device).repeat(n, 1)

    def _pin(*a, **k):
        term.vel_command_b[:] = forced
        # heading command (if present) must not fight the pin
        if getattr(term.cfg, "heading_command", False) and hasattr(term, "heading_target"):
            term.heading_target[:] = 0.0

    # replace the two mutators so nothing perturbs the pinned value
    term._resample_command = _pin           # type: ignore[assignment]
    term._update_command = _pin             # type: ignore[assignment]
    _pin()
    return forced


def _measure(wrapped_env, robot, policy, policy_nn, device, forced, secs, dt, settle_secs, label,
             out_path, cmd_tuple):
    """Reset, settle, then measure GT root velocity over `secs`. Re-pins command each step.

    `wrapped_env` is the RslRlVecEnvWrapper (for policy/step); GT reads come from `robot`
    (= wrapped_env.unwrapped.scene["robot"])."""
    term = wrapped_env.unwrapped.command_manager.get_term("base_velocity")
    n_settle = int(round(settle_secs / dt))
    n_meas = int(round(secs / dt))

    obs = wrapped_env.get_observations()
    env = wrapped_env
    # settle
    for _ in range(n_settle):
        term.vel_command_b[:] = forced
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

    # measure
    pos0 = robot.data.root_pos_w[:, :2].clone()
    speed_sum = torch.zeros(robot.num_instances, device=device)
    speed_max = torch.zeros(robot.num_instances, device=device)
    vx_sum = torch.zeros(robot.num_instances, device=device)
    any_fall = torch.zeros(robot.num_instances, dtype=torch.bool, device=device)
    steps = 0
    for _ in range(n_meas):
        term.vel_command_b[:] = forced
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)
        lin = robot.data.root_lin_vel_w[:, :2]           # GT world planar velocity
        sp = torch.linalg.norm(lin, dim=1)
        speed_sum += sp
        speed_max = torch.maximum(speed_max, sp)
        # body-frame vx for tracking (root_lin_vel_b x)
        vx_sum += robot.data.root_lin_vel_b[:, 0]
        # fall detection via projected gravity z (upright ~ -1); |tilt|>60deg => grav_z > -0.5
        grav_z = robot.data.projected_gravity_b[:, 2]
        any_fall |= (grav_z > -0.5)
        # if an env reset mid-window (done), position baseline is invalid -> flag
        any_fall |= dones.bool()
        steps += 1

    pos1 = robot.data.root_pos_w[:, :2]
    disp = torch.linalg.norm(pos1 - pos0, dim=1)          # net planar displacement per env
    elapsed = steps * dt
    drift_speed = disp / elapsed                          # net-displacement speed
    mean_speed = speed_sum / steps                        # mean instantaneous planar speed
    mean_vx = vx_sum / steps

    # report env 0 (primary) + population stats
    row = {
        "experiment": "E-T_native_robot_lab",
        "label": label,
        "cmd": list(cmd_tuple),
        "measure_secs": elapsed,
        "num_envs": int(robot.num_instances),
        "env0_drift_speed_mps": round(float(drift_speed[0]), 4),
        "env0_mean_speed_mps": round(float(mean_speed[0]), 4),
        "env0_max_speed_mps": round(float(speed_max[0]), 4),
        "env0_mean_vx_bodyframe_mps": round(float(mean_vx[0]), 4),
        "env0_net_disp_m": round(float(disp[0]), 4),
        "env0_fell": bool(any_fall[0]),
        "pop_median_drift_speed_mps": round(float(drift_speed.median()), 4),
        "pop_mean_drift_speed_mps": round(float(drift_speed.mean()), 4),
        "pop_median_mean_speed_mps": round(float(mean_speed.median()), 4),
        "pop_median_mean_vx_mps": round(float(mean_vx.median()), 4),
        "pop_fall_rate": round(float(any_fall.float().mean()), 3),
    }
    _log(row, out_path)
    return row


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg, agent_cfg):
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # play-style determinism: no corruption, no pushes/forces, no curriculum, no timeout resets
    env_cfg.observations.policy.enable_corruption = False
    env_cfg.events.randomize_apply_external_force_torque = None
    env_cfg.events.push_robot = None
    if hasattr(env_cfg.curriculum, "command_levels_lin_vel"):
        env_cfg.curriculum.command_levels_lin_vel = None
    if hasattr(env_cfg.curriculum, "command_levels_ang_vel"):
        env_cfg.curriculum.command_levels_ang_vel = None
    # do NOT let episode time-out reset the robot mid-measurement
    if hasattr(env_cfg.terminations, "time_out"):
        env_cfg.terminations.time_out = None
    # spawn at grid, not terrain levels
    env_cfg.scene.terrain.max_init_terrain_level = None

    # resolve ckpt
    if args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
        resume_path = get_checkpoint_path(log_root, agent_cfg.load_run, agent_cfg.load_checkpoint)
    print(f"[E-T] ckpt = {resume_path}", flush=True)

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    robot = env.unwrapped.scene["robot"]
    device = env.unwrapped.device
    dt = env.unwrapped.step_dt
    print(f"[E-T] step_dt={dt} (policy Hz={1.0/dt:.1f}), num_envs={robot.num_instances}, "
          f"joints={robot.joint_names}", flush=True)

    out_path = args_cli.out

    segments = [
        ("1_zero_cmd", (0.0, 0.0, 0.0), args_cli.zero_secs),
        ("2_ladder_vx_0.15", (0.15, 0.0, 0.0), args_cli.ladder_secs),
        ("2_ladder_vx_0.30", (0.30, 0.0, 0.0), args_cli.ladder_secs),
        ("2_ladder_vx_0.60", (0.60, 0.0, 0.0), args_cli.ladder_secs),
    ]

    # provenance row
    _log({"experiment": "E-T_native_robot_lab", "label": "0_provenance", "ckpt": resume_path,
          "task": args_cli.task, "step_dt": dt, "policy_hz": round(1.0 / dt, 1),
          "num_envs": int(robot.num_instances), "joints": list(robot.joint_names),
          "note": "native robot_lab env, ZERO deployment shim; cmd forced via vel_command_b pin"},
         out_path)

    for label, cmd, secs in segments:
        # native reset for a fresh episode each segment
        env.reset()
        forced = _force_command(env.unwrapped, cmd[0], cmd[1], cmd[2], device)
        _measure(env, robot, policy, policy_nn, device, forced, secs, dt,
                 args_cli.settle_secs, label, out_path, cmd)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
