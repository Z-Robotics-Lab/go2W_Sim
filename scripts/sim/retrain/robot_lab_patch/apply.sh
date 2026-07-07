#!/usr/bin/env bash
# Apply the Go2W payload-envelope retrain config (plan-d + plan-a) onto a robot_lab checkout.
#
# WHY THIS EXISTS: robot_lab is a git-ignored vendored dep (scripts/clone_deps.sh clones the
# pristine v2.3.2 tag). Local training-config edits would be LOST on a fresh clone_deps. This
# script re-applies them idempotently from a patch tracked in the go2w main repo. Called by
# clone_deps.sh after the robot_lab checkout; also safe to run by hand.
#
# WHAT IT DOES (plan-d envelope + recipe-round; plan-a = opt-in fallback):
#   plan-d  rough_env_cfg.py : base mass-add +3kg->+8kg, base CoM +/-5cm->+/-10cm
#   recipe  rough_env_cfg.py : rel_standing_envs 0.02->0.25 + wheel_vel_penalty 0->-0.01
#           (CEO fire order 2026-07-07 after attribution E-T verdict "training recipe";
#           weight provenance: ddtrobot_tita rough_env_cfg.py:198, the only wheeled
#           config in robot_lab v2.3.2 that enables the term)
#   plan-a  payload_env_cfg.py + __init__.py : opt-in Go2W-Payload-v0 task (fixed ~6.5kg
#           front-offset payload) — inherits the recipe via Flat->Rough.
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

# 2) plan-d + plan-a registration patch (idempotent: skip if already applied).
cd "$RL"
if git apply --reverse --check "$HERE/go2w_payload_envelope.patch" >/dev/null 2>&1; then
  echo "[apply] patch already applied — skipping (idempotent)"
elif git apply --check "$HERE/go2w_payload_envelope.patch" >/dev/null 2>&1; then
  git apply "$HERE/go2w_payload_envelope.patch"
  echo "[apply] plan-d + plan-a registration patch applied"
else
  echo "[apply] ERROR: patch does not apply cleanly to $RL (robot_lab not at pristine v2.3.2?)." >&2
  echo "[apply] inspect: git -C $RL apply --check $HERE/go2w_payload_envelope.patch" >&2
  exit 1
fi

echo "[apply] DONE. plan-d is now the default for RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0."
echo "[apply] plan-a opt-in task id: RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-Payload-v0"
