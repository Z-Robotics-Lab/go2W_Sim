#!/usr/bin/env bash
# Run the E-T attribution experiment: shipped ckpt in robot_lab's NATIVE play env, zero shim,
# forced zero/low commands, GT root velocity read. Splits the zero-cmd creep cause between the
# TRAINING recipe and the DEPLOYMENT sim2sim shim. See DEBUG.md "归因判决" + scripts/sim/attribution_play.py.
#
# ONE-sim rule (AGENTS.md Inv.5): refuses if any Isaac kit/python.sh is live OR the sim slot is
# held by a sibling. Memory: host-side docker exec wrapped in systemd-run --scope MemoryMax
# (container is already --memory=40g). NEVER kills a sibling (Inv.8).
set -euo pipefail

CONTAINER="${GO2W_ISAAC_CONTAINER:-go2w-isaac}"
CKPT="${GO2W_ET_CKPT:-/workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-04_15-52-42/model_1999.pt}"
TASK="${GO2W_ET_TASK:-RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0}"
NUM_ENVS="${GO2W_ET_ENVS:-4}"
ZERO_SECS="${GO2W_ET_ZERO_SECS:-30}"
LADDER_SECS="${GO2W_ET_LADDER_SECS:-15}"
MEM_MAX="${GO2W_ET_MEMMAX:-45G}"
OUT="${GO2W_ET_OUT:-/workspace/go2w/var/evidence/retrain/attribution/et_native.jsonl}"

# ---- preflight: ONE sim + memory ----
if pgrep -fa "isaac-sim/kit|/isaac-sim/python.sh|warehouse_nav.py|warehouse_scene.py" | grep -qv "pgrep"; then
  echo "[E-T] REFUSE: an Isaac kit/python.sh/warehouse sim process is live (sim slot busy)." >&2
  pgrep -fa "isaac-sim/kit|/isaac-sim/python.sh|warehouse_nav.py|warehouse_scene.py" | grep -v pgrep >&2 || true
  exit 3
fi
# authoritative slot gate (the vector_os_nano predicate, if present) — covers idle sibling REPLs
SLOT="/home/yusen/Desktop/vector_os_nano/tools/sim_slot.py"
if [ -f "$SLOT" ]; then
  if ! python3 "$SLOT" --quiet 2>/dev/null; then
    echo "[E-T] REFUSE: sim slot HELD (per $SLOT):" >&2
    python3 "$SLOT" >&2 || true
    echo "[E-T] WAIT — do NOT kill the holder (Inv.8). Re-run when the slot frees." >&2
    exit 3
  fi
fi
AVAIL=$(free -g | awk '/^Mem:/{print $7}')
if [ "${AVAIL:-0}" -lt 30 ]; then
  echo "[E-T] REFUSE: only ${AVAIL}G free (<30G). WAIT." >&2
  exit 3
fi
if ! docker ps --filter "name=${CONTAINER}" --format '{{.Names}}' | grep -q "${CONTAINER}"; then
  echo "[E-T] REFUSE: container ${CONTAINER} not Up." >&2
  exit 3
fi

echo "[E-T] preflight OK. ckpt=$CKPT task=$TASK envs=$NUM_ENVS zero=${ZERO_SECS}s ladder=${LADDER_SECS}s"
echo "[E-T] out=$OUT (host: ~/Desktop/go2w/var/evidence/retrain/attribution/et_native.jsonl)"

# cwd = robot_lab so hydra task registers + logs/ resolves; script imports cli_args via abs path.
RUN_CMD="cd /workspace/go2w/robot_lab && \
  /isaac-sim/python.sh /workspace/go2w/scripts/sim/attribution_play.py \
    --task $TASK --checkpoint $CKPT --num_envs $NUM_ENVS \
    --zero_secs $ZERO_SECS --ladder_secs $LADDER_SECS \
    --out $OUT --headless"

TS="$(date +%Y-%m-%d_%H-%M-%S)"
HOST_LOG="var/evidence/retrain/attribution/et_run_${TS}.log"
mkdir -p "$(dirname "$HOST_LOG")"
echo "[E-T] host log -> $HOST_LOG"

# NOTE: pass docker exec as systemd-run's direct argv so $CONTAINER/$RUN_CMD are expanded by
# THIS shell. (The previous `bash -c "$(declare -f run_exec); run_exec"` pattern serialized the
# function body with UNEXPANDED variables into a child bash where they were empty ->
# "invalid container name or ID". run_retrain.sh carries the same latent bug.)
if command -v systemd-run >/dev/null 2>&1; then
  systemd-run --scope --user -p "MemoryMax=$MEM_MAX" -p "MemorySwapMax=0" \
    docker exec -u 0 "$CONTAINER" bash -lc "$RUN_CMD" 2>&1 | tee "$HOST_LOG"
else
  echo "[E-T] WARN: systemd-run unavailable — relying on container --memory cap only." >&2
  docker exec -u 0 "$CONTAINER" bash -lc "$RUN_CMD" 2>&1 | tee "$HOST_LOG"
fi

echo "[E-T] DONE. Verdict rows in $OUT — read segment 1_zero_cmd env0_drift_speed_mps:"
echo "      <0.02 => DEPLOYMENT-side (shim/physics-rate);  >=0.05 => TRAINING recipe;  0.02-0.05 => both."
