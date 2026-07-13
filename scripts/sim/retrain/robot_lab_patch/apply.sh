#!/usr/bin/env bash
# Apply the Go2W payload-envelope retrain config (plan-d + plan-a) onto a robot_lab checkout.
#
# WHY THIS EXISTS: robot_lab is a git-ignored vendored dep (scripts/clone_deps.sh clones the
# pristine v2.3.2 tag). Local training-config edits would be LOST on a fresh clone_deps. This
# script re-applies them idempotently from a patch tracked in the go2w main repo. Called by
# clone_deps.sh after the robot_lab checkout; also safe to run by hand.
#
# WHAT IT DOES (recipe v2, 2nd fire order 2026-07-07; plan-a = opt-in fallback):
#   recipe v2  rough_env_cfg.py : rel_standing_envs 0.02->0.12 + wheel_vel_penalty
#              0->-0.005 (half the ddtrobot_tita provenance value :198 — conservative for
#              fine-tuning FROM the shipped ckpt). v1 (0.25/-0.01, from-scratch) FAILED
#              §4b — lazy-basin collapse, run 2026-07-07_05-53-47 — kept as comments in
#              the cfg for ablation traceability.
#   plan-d     WITHDRAWN this round (post-mortem H2 isolation): mass/CoM envelope back to
#              parent defaults (-1..+3kg, ±5cm); kept as comments. Payload envelope gets
#              its own later round (docs/sim-plan.md todo).
#   plan-a     payload_env_cfg.py + __init__.py : opt-in Go2W-Payload-v0 task (fixed
#              ~6.5kg front-offset payload) — inherits the recipe via Flat->Rough.
# Everything else (other rewards, actuator gains, decimation, obs57/act16) is UNTOUCHED —
# the new ckpt stays isomorphic to the frozen scripts/sim/go2w_policy.py shim.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$HERE/../../../.." && pwd)"          # go2w main repo root
RL="${1:-$REPO_DIR/robot_lab}"                       # robot_lab checkout (override as arg 1)
CFG_DIR="source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_go2w"

if [ ! -d "$RL/$CFG_DIR" ]; then
  echo "[apply] ERROR: robot_lab go2w config dir not found under $RL/$CFG_DIR" >&2
  echo "[apply] run scripts/clone_deps.sh first." >&2
  exit 1
fi

# 1) plan-a variant file (new file — copy, idempotent).
cp "$HERE/payload_env_cfg.py" "$RL/$CFG_DIR/payload_env_cfg.py"
echo "[apply] plan-a: payload_env_cfg.py installed"

# The two patches STACK on rough_env_cfg.py (yaw's inserted block shifts the envelope hunk
# offsets), so a `git apply --reverse --check` on the envelope patch FAILS once yaw sits on
# top — it is NOT a reliable "already applied?" probe here. Detect application by a unique
# SENTINEL comment each patch introduces (order-independent). Anchors:
INIT_FILE="$RL/$CFG_DIR/__init__.py"
ROUGH_FILE="$RL/$CFG_DIR/rough_env_cfg.py"
ENV_SENTINEL="Plan-a fallback (CEO-approved retrain 2026-07-07)"   # from the envelope patch (__init__.py)
YAW_SENTINEL="YAW ROUND (CEO B-plan, 2026-07-12)"                  # from the yaw patch (rough_env_cfg.py)

cd "$RL"

# 2) plan-d + plan-a registration patch (idempotent via sentinel).
if grep -qF "$ENV_SENTINEL" "$INIT_FILE" 2>/dev/null; then
  echo "[apply] envelope patch already applied — skipping (idempotent)"
elif git apply --check "$HERE/go2w_payload_envelope.patch" >/dev/null 2>&1; then
  git apply "$HERE/go2w_payload_envelope.patch"
  echo "[apply] plan-d + plan-a registration patch applied"
else
  echo "[apply] ERROR: patch does not apply cleanly to $RL (robot_lab not at pristine v2.3.2?)." >&2
  echo "[apply] inspect: git -C $RL apply --check $HERE/go2w_payload_envelope.patch" >&2
  exit 1
fi

# 3) YAW-round patch (CEO B-plan 2026-07-12): pure-yaw command distribution + deployment
#    wheel-friction domain + ang_vel tracking-weight bump. LAYERS ON TOP of the envelope
#    patch (anchors on rough_env_cfg.py lines that exist only after step 2). Idempotent via
#    sentinel. See docs/retrain-yaw.md for the pre-registered gates.
if grep -qF "$YAW_SENTINEL" "$ROUGH_FILE" 2>/dev/null; then
  echo "[apply] yaw patch already applied — skipping (idempotent)"
elif git apply --check "$HERE/go2w_yaw_command_friction.patch" >/dev/null 2>&1; then
  git apply "$HERE/go2w_yaw_command_friction.patch"
  echo "[apply] yaw command+friction patch applied"
else
  echo "[apply] ERROR: yaw patch does not apply cleanly to $RL (envelope patch missing/changed?)." >&2
  echo "[apply] inspect: git -C $RL apply --check $HERE/go2w_yaw_command_friction.patch" >&2
  exit 1
fi

echo "[apply] DONE. plan-d is now the default for RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0."
echo "[apply] plan-a opt-in task id: RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-Payload-v0"
echo "[apply] yaw-round edits (rel_heading_envs 0.5, ang_vel_z ±1.5, wheel-friction max 1.4-2.0/1.2-1.8, track_ang_vel_z 2.0) layered on."
