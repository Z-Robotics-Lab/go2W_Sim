#!/bin/bash
# L2-T4 multi-segment: chain N relative goals (fwd left) via l2_goto_probe, one CSV per leg.
# Each leg starts from wherever the previous ended; "left" offsets force a turn.
# Runs inside navstack container. Usage:
#   l2_multi_probe.sh <out_prefix> <tol> <timeout_each> "fwd1,left1" "fwd2,left2" ...
# NOTE: no `set -u` — ROS setup.bash references unbound vars and aborts under it.
PREFIX="$1"; TOL="$2"; TOE="$3"; shift 3
source /ws/install/setup.bash 2>/dev/null || true
i=0
for leg in "$@"; do
  fwd="${leg%,*}"; left="${leg#*,}"
  i=$((i+1))
  echo "=== LEG $i: fwd=$fwd left=$left ==="
  python3 /ws/l2_goto_probe.py "${PREFIX}_leg${i}.csv" "$fwd" "$left" \
    --relative --tol "$TOL" --timeout "$TOE" --hold 3
done
echo "=== MULTI DONE ($i legs) ==="
