#!/usr/bin/env bash
# Go2W payload-envelope retrain launcher (plan-d default; plan-a via GO2W_RETRAIN_PLAN=a).
# CEO-approved 2026-07-07. See scripts/sim/retrain_runbook.md for the full protocol.
#
# HOST-SAFETY RED LINES (rules/common/performance.md + AGENTS.md Inv.5):
#   - ONE sim at a time. This script REFUSES to launch if any kit/Isaac process or the
#     warehouse_nav sim is already running (the go2w-isaac container is shared).
#   - Training runs INSIDE the go2w-isaac container (already capped at docker --memory=40g).
#     The host-side `docker exec` is additionally wrapped in systemd-run --scope
#     -p MemoryMax=45G as a host backstop.
#   - NO ulimit -v: torch/CUDA reserve tens of GB of VIRTUAL address space that is never
#     physically backed; a -v cap spuriously kills bounded GPU runs (host rule #2 CAVEAT).
#     Real memory is bounded by serialization (ONE sim) + the container/systemd caps.
#   - This script NEVER kills a sibling sim/loop. On contention it EXITS non-zero (Inv.8).
set -euo pipefail

# --------------------------------------------------------------------------- config
PLAN="${GO2W_RETRAIN_PLAN:-d}"                    # d (default) | a (payload fallback)
NUM_ENVS="${GO2W_RETRAIN_ENVS:-2048}"             # training anchor (matches shipped ckpt)
MAX_ITER="${GO2W_RETRAIN_ITERS:-2000}"            # training anchor (~30 min on 5080)
SEED="${GO2W_RETRAIN_SEED:-42}"
MEM_MAX="${GO2W_RETRAIN_MEMMAX:-45G}"             # host-side systemd scope backstop
CONTAINER="${GO2W_ISAAC_CONTAINER:-go2w-isaac}"
MIN_FREE_G="${GO2W_RETRAIN_MIN_FREE_G:-30}"       # require >= this many GB free before launch

case "$PLAN" in
  d) TASK="RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0" ;;
  a) TASK="RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-Payload-v0" ;;
  *) echo "[retrain] ERROR: GO2W_RETRAIN_PLAN must be 'd' or 'a' (got '$PLAN')" >&2; exit 2 ;;
esac

echo "[retrain] plan=$PLAN task=$TASK envs=$NUM_ENVS iters=$MAX_ITER seed=$SEED"

# --------------------------------------------------------------- preflight: ONE sim
echo "[retrain] preflight: checking for a live sim (ONE-sim rule)..."
if pgrep -fa "warehouse_nav.py|warehouse_scene.py" >/dev/null 2>&1; then
  echo "[retrain] REFUSE: a Go2W sim (warehouse_nav/scene) is already running." >&2
  echo "[retrain] The go2w-isaac container is SHARED — serialize. WAITING is your only move." >&2
  pgrep -fa "warehouse_nav.py|warehouse_scene.py" >&2
  exit 1
fi
if pgrep -fa "isaac-sim/kit|/isaac-sim/python.sh" >/dev/null 2>&1; then
  echo "[retrain] REFUSE: an Isaac kit/python.sh process is already running (sim slot busy)." >&2
  pgrep -fa "isaac-sim/kit|/isaac-sim/python.sh" >&2
  exit 1
fi

# --------------------------------------------------------------- preflight: memory
FREE_G="$(free -g | awk '/^Mem:/{print $7}')"
echo "[retrain] preflight: ${FREE_G}G available (need >= ${MIN_FREE_G}G)"
if [ "${FREE_G:-0}" -lt "$MIN_FREE_G" ]; then
  echo "[retrain] REFUSE: only ${FREE_G}G free, need >= ${MIN_FREE_G}G. Another heavy job may hold RAM." >&2
  ps -eo pid,rss,comm --sort=-rss | head -6 >&2
  exit 1
fi

# --------------------------------------------------------------- preflight: GPU idle
if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_MEM_USED="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')"
  echo "[retrain] preflight: GPU mem used = ${GPU_MEM_USED} MiB"
  # heuristic: a live Isaac sim holds several GB; warn (do not hard-block — a fresh 5080 idles ~a few hundred MiB)
  if [ "${GPU_MEM_USED:-0}" -gt 4000 ]; then
    echo "[retrain] WARN: GPU already using ${GPU_MEM_USED} MiB — a sim/other job may be resident. Verify before continuing." >&2
    echo "[retrain] set GO2W_RETRAIN_FORCE=1 to proceed anyway." >&2
    [ "${GO2W_RETRAIN_FORCE:-0}" = "1" ] || exit 1
  fi
else
  echo "[retrain] WARN: nvidia-smi not found on host — cannot verify GPU idle." >&2
fi

# --------------------------------------------------------------- preflight: container
if ! docker ps --filter "name=$CONTAINER" --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "[retrain] REFUSE: container '$CONTAINER' is not running. Start it via scripts/setup_container.sh." >&2
  exit 1
fi

# --------------------------------------------------------------- launch (foreground)
# rsl_rl writes to logs/rsl_rl/<experiment_name>/<YYYY-MM-DD_HH-MM-SS>/ with model_*.pt +
# params/{env,agent}.yaml. experiment_name is 'unitree_go2w_flat' for both plan-d and plan-a
# (plan-a reuses the flat runner cfg), so new ckpts land beside the shipped one — DISTINGUISH
# them by timestamp. A log line is emitted so the runbook can record the exact dir.
TS="$(date +%Y-%m-%d_%H-%M-%S)"
HOST_LOG="logs/retrain_${PLAN}_${TS}.log"
echo "[retrain] launching (foreground). host log -> $HOST_LOG"
echo "[retrain] to monitor from another shell: tail -f $HOST_LOG   (reward: grep 'Mean reward')"

TRAIN_CMD="cd /workspace/go2w/robot_lab && \
  /isaac-sim/python.sh scripts/reinforcement_learning/rsl_rl/train.py \
    --task $TASK --headless --num_envs $NUM_ENVS --max_iterations $MAX_ITER --seed $SEED"

# systemd-run scope is a HOST backstop around the docker exec; the container itself is capped
# at --memory=40g (setup_container.sh). If systemd-run is unavailable, fall back to a bare
# docker exec (container cap still applies).
run_exec() { docker exec -u 0 "$CONTAINER" bash -lc "$TRAIN_CMD"; }
if command -v systemd-run >/dev/null 2>&1; then
  systemd-run --scope --user -p "MemoryMax=$MEM_MAX" -p "MemorySwapMax=0" \
    bash -c "$(declare -f run_exec); run_exec" 2>&1 | tee "$HOST_LOG"
else
  echo "[retrain] WARN: systemd-run unavailable — relying on container --memory=40g cap only." >&2
  run_exec 2>&1 | tee "$HOST_LOG"
fi

echo "[retrain] DONE. Newest ckpt dir:"
ls -dt logs/rsl_rl/unitree_go2w_flat/*/ 2>/dev/null | head -1
echo "[retrain] NEXT: run scripts/sim/policy_acceptance.py against model_1999.pt in that dir."
