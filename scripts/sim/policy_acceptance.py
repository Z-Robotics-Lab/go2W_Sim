#!/usr/bin/env python3
"""Go2W policy acceptance suite — the four-segment symptom battery (CEO-approved retrain 2026-07-07).

RUN INSIDE the go2w-isaac container (imports isaaclab). OFFLINE-WRITTEN here; runs later at a
clean sim window (ONE-sim rule). Example (inside container, headless):

    /isaac-sim/python.sh scripts/sim/policy_acceptance.py \
        --policy /workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/<NEW_TS>/model_1999.pt \
        --out /workspace/go2w/logs/acceptance_<NEW_TS>.jsonl --headless

Compare a NEW ckpt against the shipped baseline by running it twice (old ckpt, new ckpt) and
diffing the JSON rows. Same loader path, same feed path, same judge — so the only variable is
the ckpt.

WHY THIS FORM (deployment truth):
  - Robot = go2w_sensored.urdf (arm + NUC payload). This is the SAME asset warehouse_nav.py
    spawns, i.e. the real "6.5 kg front-offset load" deployment form. The acceptance runs on
    the load state by construction, no runtime payload injection needed.
    (Fidelity caveat: the URDF's nuc_weight link is a 0.5 kg PLACEHOLDER, so the URDF payload
    (~5.16 kg over bare trunk) is LIGHTER than the audited ~6.5 kg. If acceptance passes here
    but marginally, re-check with a heavier nuc_weight — see retrain_runbook.md "payload fidelity".)
  - Actuator gains forced to TRAINING values (legs 25/0.5, wheels 0/0.5) exactly as
    warehouse_nav.py does for --policy, so the policy sees its trained plant.
  - Policy fed via the frozen scripts/sim/go2w_policy.py Go2WPolicy (obs57/act16), 50 Hz on
    100 Hz physics (act every 2 steps) — identical to warehouse_nav.py's cadence.

FOUR SEGMENTS (symptoms from DEBUG.md / vx-audit):
  (1) zero-cmd 30 s        -> PASS if GT drift speed < 0.02 m/s (baseline crept 0.075 m/s).
  (2) low-cmd ladder       -> vx = 0.15 / 0.30 / 0.60 for 10 s each; report tracking RMS.
                              baseline forward bias: 0->0.075, 0.15->0.257, 0.30->0.383,
                              0.60->0.644 m/s (over-shoot spectrum). PASS if achieved speed is
                              within tol of commanded (default |err|/cmd <= 0.35).
  (3) wz = +/-1.4 step x5   -> fall rate. FALL = |roll|>60deg OR |pitch|>60deg. PASS if 0 falls.
  (4) arc (0.3, 0.5)       -> vx=0.3, wz=0.5 for 10 s; report speed + fall + curvature.

Judgements are printed AND written as one JSON object per segment (append) to --out. Exit 0
always (this is a MEASUREMENT tool; the verdict is in the rows), unless a fatal load/setup error.
"""
import argparse
import json
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--policy", type=str, required=True, help="Go2W velocity ckpt (.pt)")
parser.add_argument("--out", type=str, default=None, help="append JSONL result rows here")
parser.add_argument("--body", choices=["loaded", "bare"], default="loaded",
                    help="loaded=go2w_sensored.urdf (deployment payload); bare=go2w_bare.urdf "
                         "(trained trunk, A/B control). Sets --urdf if that is left default.")
parser.add_argument("--urdf", type=str, default=None,
                    help="robot URDF (default: chosen by --body)")
parser.add_argument("--drift-thresh", type=float, default=0.02, help="seg1 PASS drift m/s")
parser.add_argument("--track-tol", type=float, default=0.35, help="seg2/4 PASS rel speed error")
parser.add_argument("--fall-deg", type=float, default=60.0, help="fall if |roll|or|pitch|>this")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

DEVICE = args_cli.device or "cuda:0"
PHYS_DT = 1.0 / 100.0          # 100 Hz physics (matches warehouse_nav.py)
POLICY_EVERY = 2              # act every 2 physics steps -> 50 Hz policy
SPAWN_Z = 0.42               # match warehouse_nav.py init height

URDF_LOADED = "/workspace/go2w/assets/urdf/go2w_sensored.urdf"
URDF_BARE = "/workspace/go2w/assets/urdf/go2w_bare.urdf"
URDF_PATH = args_cli.urdf or (URDF_LOADED if args_cli.body == "loaded" else URDF_BARE)
HAS_ARM = args_cli.body == "loaded"


def build_robot_cfg():
    """Build the go2w cfg for the chosen body. bare = no arm/gripper actuators or piper init.

    Same spawn + TRAINING actuator gains as warehouse_nav.py --policy path, on flat ground.
    """
    joint_pos = {
        ".*_hip_joint": 0.0, ".*_thigh_joint": 0.8, ".*_calf_joint": -1.5,
        ".*_foot_joint": 0.0,
    }
    actuators = {
        # TRAINING gains (robot_lab UNITREE_GO2W_CFG): legs 25/0.5, wheels 0/0.5.
        "legs": ImplicitActuatorCfg(
            joint_names_expr=["(FL|FR|RL|RR)_(hip|thigh|calf)_joint"],
            effort_limit_sim=23.5, velocity_limit_sim=30.0, stiffness=25.0, damping=0.5),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*_foot_joint"],
            effort_limit_sim=23.5, velocity_limit_sim=30.0, stiffness=0.0, damping=0.5),
    }
    if HAS_ARM:
        joint_pos.update({
            "piper_joint2": 0.8, "piper_joint3": -1.2,
            "piper_joint[1456]": 0.0, "piper_joint[78]": 0.0,
        })
        actuators["arm"] = ImplicitActuatorCfg(
            joint_names_expr=["piper_joint[1-6]"],
            effort_limit_sim=30.0, velocity_limit_sim=5.0, stiffness=100.0, damping=5.0)
        actuators["gripper"] = ImplicitActuatorCfg(
            joint_names_expr=["piper_joint[78]"],
            effort_limit_sim=50.0, velocity_limit_sim=1.0, stiffness=800.0, damping=20.0)
    return ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=URDF_PATH,
            fix_base=False,
            merge_fixed_joints=False,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=1,
            ),
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, SPAWN_Z), joint_pos=joint_pos, joint_vel={".*": 0.0},
        ),
        actuators=actuators,
    )


def quat_to_rp(q):
    """(w,x,y,z) -> (roll, pitch) in radians."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def write_row(out_path, row):
    print("[ACCEPT] " + json.dumps(row), flush=True)
    if out_path:
        with open(out_path, "a") as f:
            f.write(json.dumps(row) + "\n")


def reset_to_birth(robot, birth_root, birth_jpos, birth_jvel, policy):
    """Return robot to spawn pose + zero velocities + clear policy carry-state."""
    robot.write_root_state_to_sim(birth_root.clone())
    robot.write_joint_state_to_sim(birth_jpos.clone(), birth_jvel.clone())
    robot.reset()
    if policy is not None:
        policy.last_action = torch.zeros_like(policy.last_action)


def apply_policy(robot, policy, default_pos, cmd):
    """One 50 Hz policy application: hold arm/gripper at default, override legs+wheels.

    Mirrors warehouse_nav.py exactly: default_pos is written FIRST so the PiPER arm/gripper
    hold their default pose (payload stays mounted, not limp), then policy legs/wheels override.
    """
    robot.set_joint_position_target(default_pos)  # arm/gripper hold
    leg_ids, leg_tgt, wheel_ids, wheel_vel = policy.act(*cmd)
    robot.set_joint_position_target(leg_tgt, joint_ids=leg_ids)
    robot.set_joint_velocity_target(wheel_vel, joint_ids=wheel_ids)


def settle(sim, robot, policy, default_pos, steps=100):
    """Let the robot settle standing (zero cmd) before a segment; not measured."""
    for i in range(steps):
        if i % POLICY_EVERY == 0:
            apply_policy(robot, policy, default_pos, (0.0, 0.0, 0.0))
        robot.write_data_to_sim()
        sim.step()
        robot.update(PHYS_DT)


def run_segment(sim, robot, policy, default_pos, cmd_vx, cmd_vy, cmd_wz, duration_s, fall_deg):
    """Drive the policy at a fixed cmd for duration_s; return GT trajectory stats."""
    n_steps = int(round(duration_s / PHYS_DT))
    p0 = robot.data.root_pos_w[0, :2].clone()
    fell = False
    max_tilt = 0.0
    speeds = []
    pitches = []
    for i in range(n_steps):
        if i % POLICY_EVERY == 0:
            apply_policy(robot, policy, default_pos, (cmd_vx, cmd_vy, cmd_wz))
        robot.write_data_to_sim()
        sim.step()
        robot.update(PHYS_DT)
        v = robot.data.root_lin_vel_w[0, :2]
        speeds.append(float(torch.linalg.norm(v).item()))
        q = robot.data.root_quat_w[0].tolist()
        roll, pitch = quat_to_rp(q)
        pitches.append(pitch)
        tilt = max(abs(roll), abs(pitch))
        max_tilt = max(max_tilt, tilt)
        if tilt > math.radians(fall_deg):
            fell = True
    p1 = robot.data.root_pos_w[0, :2].clone()
    disp = float(torch.linalg.norm(p1 - p0).item())
    mean_speed = sum(speeds) / len(speeds) if speeds else 0.0
    # pitch variance (rad^2): body-pitch instability discriminator for the OOD A/B judgement.
    pmean = sum(pitches) / len(pitches) if pitches else 0.0
    pitch_var = sum((p - pmean) ** 2 for p in pitches) / len(pitches) if pitches else 0.0
    return {
        "disp_m": round(disp, 4),
        "mean_speed_mps": round(mean_speed, 4),
        "displacement_speed_mps": round(disp / duration_s, 4),
        "max_tilt_deg": round(math.degrees(max_tilt), 2),
        "pitch_var_rad2": round(pitch_var, 6),
        "fell": fell,
    }


def main():
    sim = SimulationContext(SimulationCfg(dt=PHYS_DT, render_interval=1, device=DEVICE))
    sim.set_camera_view(eye=(4.0, 4.0, 3.0), target=(0.0, 0.0, 0.5))
    ground = sim_utils.GroundPlaneCfg(); ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2000.0); light.func("/World/Light", light)

    robot = Articulation(build_robot_cfg())
    sim.reset()

    import sys, os  # noqa: E402
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from go2w_policy import Go2WPolicy
    policy = Go2WPolicy(args_cli.policy, robot, DEVICE)

    birth_root = robot.data.default_root_state.clone()
    birth_jpos = robot.data.default_joint_pos.clone()
    birth_jvel = robot.data.default_joint_vel.clone()
    default_pos = robot.data.default_joint_pos.clone()  # arm/gripper hold target

    meta = {"ckpt": args_cli.policy, "body": args_cli.body, "urdf": URDF_PATH,
            "drift_thresh": args_cli.drift_thresh, "track_tol": args_cli.track_tol,
            "fall_deg": args_cli.fall_deg}

    # -------- Segment 1: zero-cmd 30 s (creep test) --------
    reset_to_birth(robot, birth_root, birth_jpos, birth_jvel, policy)
    settle(sim, robot, policy, default_pos, steps=100)
    r = run_segment(sim, robot, policy, default_pos, 0.0, 0.0, 0.0, 30.0, args_cli.fall_deg)
    r_speed = r["displacement_speed_mps"]
    write_row(args_cli.out, {**meta, "segment": "1_zero_cmd_30s",
                             "cmd": [0.0, 0.0, 0.0], **r,
                             "criterion": f"displacement_speed < {args_cli.drift_thresh} m/s",
                             "pass": (r_speed < args_cli.drift_thresh) and not r["fell"]})

    # -------- Segment 2: low-cmd ladder 0.15 / 0.30 / 0.60, 10 s each --------
    baseline = {0.15: 0.257, 0.30: 0.383, 0.60: 0.644}  # old-policy forward-bias spectrum
    for cmd in (0.15, 0.30, 0.60):
        reset_to_birth(robot, birth_root, birth_jpos, birth_jvel, policy)
        settle(sim, robot, policy, default_pos, steps=100)
        r = run_segment(sim, robot, policy, default_pos, cmd, 0.0, 0.0, 10.0, args_cli.fall_deg)
        achieved = r["mean_speed_mps"]
        rel_err = abs(achieved - cmd) / cmd if cmd > 0 else 0.0
        write_row(args_cli.out, {**meta, "segment": f"2_ladder_vx_{cmd}",
                                 "cmd": [cmd, 0.0, 0.0], **r,
                                 "cmd_vx": cmd, "achieved_speed_mps": achieved,
                                 "rel_err": round(rel_err, 4),
                                 "old_policy_speed_mps": baseline[cmd],
                                 "criterion": f"|achieved-cmd|/cmd <= {args_cli.track_tol}",
                                 "pass": (rel_err <= args_cli.track_tol) and not r["fell"]})

    # -------- Segment 3: wz = +/-1.4 step x5 (fall test) --------
    falls = 0
    max_tilt_overall = 0.0
    for k in range(5):
        reset_to_birth(robot, birth_root, birth_jpos, birth_jvel, policy)
        settle(sim, robot, policy, default_pos, steps=100)
        wz = 1.4 if k % 2 == 0 else -1.4
        r = run_segment(sim, robot, policy, default_pos, 0.0, 0.0, wz, 3.0, args_cli.fall_deg)
        max_tilt_overall = max(max_tilt_overall, r["max_tilt_deg"])
        if r["fell"]:
            falls += 1
        write_row(args_cli.out, {**meta, "segment": f"3_wz_step_{k}",
                                 "cmd": [0.0, 0.0, wz], **r,
                                 "criterion": "no fall (|roll|or|pitch| <= fall_deg)",
                                 "pass": not r["fell"]})
    write_row(args_cli.out, {**meta, "segment": "3_wz_step_summary",
                             "n_steps": 5, "falls": falls,
                             "fall_rate": round(falls / 5.0, 3),
                             "max_tilt_deg": max_tilt_overall,
                             "criterion": "fall_rate == 0",
                             "pass": falls == 0})

    # -------- Segment 4: arc (vx=0.3, wz=0.5), 10 s --------
    reset_to_birth(robot, birth_root, birth_jpos, birth_jvel, policy)
    settle(sim, robot, policy, default_pos, steps=100)
    r = run_segment(sim, robot, policy, default_pos, 0.3, 0.0, 0.5, 10.0, args_cli.fall_deg)
    rel_err = abs(r["mean_speed_mps"] - 0.3) / 0.3
    write_row(args_cli.out, {**meta, "segment": "4_arc_0.3_0.5",
                             "cmd": [0.3, 0.0, 0.5], **r,
                             "rel_err_vs_vx": round(rel_err, 4),
                             "criterion": f"no fall AND |speed-0.3|/0.3 <= {args_cli.track_tol}",
                             "pass": (not r["fell"]) and (rel_err <= args_cli.track_tol)})

    print("[ACCEPT] suite complete.", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
