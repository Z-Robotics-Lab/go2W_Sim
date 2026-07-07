#!/usr/bin/env python3
"""Offline A/B verdict — is the Go2W fault OOD payload (not the policy itself)?

PURE OFFLINE (no sim). Reads two JSONL result files produced by policy_acceptance.py with the
SAME (old) ckpt, one per body:
    A = bare  (go2w_bare.urdf,     the trunk the policy was TRAINED on)
    B = loaded(go2w_sensored.urdf, the ~6.5 kg front-offset deployment payload)

The OOD hypothesis: the shipped policy is FINE on the body it trained on (A), and degrades only
when the untrained payload is added (B). If B is significantly WORSE than A on the discriminators
below, the fault is the payload being out-of-distribution — i.e. a RETRAIN (widen the envelope)
is the right fix, not a policy redesign. If A is ALSO bad, the policy itself is suspect and a
retrain of the same recipe would not help — escalate.

Discriminators (CEO-named): tracking error, pitch variance, fall rate.
  - tracking (seg2 ladder + seg4 arc): rel_err loaded vs bare.
  - pitch_var_rad2 (all driving segments): body-pitch instability.
  - fall rate (seg3 wz steps): loaded falls where bare stands.

Verdict = OOD_CONFIRMED if B is materially worse than A on >= 2 of the 3 discriminators AND A
itself is healthy (bare passes seg1 drift + seg3 no-fall). Otherwise INCONCLUSIVE/POLICY_SUSPECT.

Usage:
    python3 scripts/sim/ab_verdict.py --bare acceptance_bare.jsonl --loaded acceptance_loaded.jsonl
"""
import argparse
import json


def load_rows(path):
    rows = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            rows[r.get("segment")] = r
    return rows


def seg_track_relerr(rows):
    """mean relative tracking error across ladder+arc segments."""
    errs = []
    for seg, r in rows.items():
        if seg.startswith("2_ladder_vx_") and "rel_err" in r:
            errs.append(r["rel_err"])
        if seg == "4_arc_0.3_0.5" and "rel_err_vs_vx" in r:
            errs.append(r["rel_err_vs_vx"])
    return sum(errs) / len(errs) if errs else None


def seg_pitch_var(rows):
    """mean pitch variance across all driving segments (ladder + arc)."""
    pv = []
    for seg, r in rows.items():
        if (seg.startswith("2_ladder_vx_") or seg == "4_arc_0.3_0.5") and "pitch_var_rad2" in r:
            pv.append(r["pitch_var_rad2"])
    return sum(pv) / len(pv) if pv else None


def fall_rate(rows):
    s = rows.get("3_wz_step_summary")
    return s.get("fall_rate") if s else None


def bare_healthy(rows):
    """A must itself be healthy for the OOD argument to hold: bare passes drift + no falls."""
    seg1 = rows.get("1_zero_cmd_30s", {})
    fr = fall_rate(rows)
    return bool(seg1.get("pass")) and (fr == 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bare", required=True, help="JSONL from --body bare (A, control)")
    ap.add_argument("--loaded", required=True, help="JSONL from --body loaded (B, deployment)")
    ap.add_argument("--track-worse-x", type=float, default=1.5,
                    help="B tracking rel_err must be >= this x A to count as worse")
    ap.add_argument("--pitch-worse-x", type=float, default=1.5,
                    help="B pitch variance must be >= this x A to count as worse")
    ap.add_argument("--out", default=None, help="write verdict JSON here")
    args = ap.parse_args()

    A = load_rows(args.bare)
    B = load_rows(args.loaded)

    a_track, b_track = seg_track_relerr(A), seg_track_relerr(B)
    a_pv, b_pv = seg_pitch_var(A), seg_pitch_var(B)
    a_fall, b_fall = fall_rate(A), fall_rate(B)

    def worse(a, b, x):
        if a is None or b is None:
            return None
        return b >= max(a * x, a + 1e-9)

    track_worse = worse(a_track, b_track, args.track_worse_x)
    pitch_worse = worse(a_pv, b_pv, args.pitch_worse_x)
    fall_worse = (b_fall is not None and a_fall is not None and b_fall > a_fall)

    n_worse = sum(1 for w in (track_worse, pitch_worse, fall_worse) if w)
    a_ok = bare_healthy(A)

    if a_ok and n_worse >= 2:
        verdict = "OOD_CONFIRMED"
        note = ("Bare (trained body) is healthy; loaded (payload) is materially worse on "
                f"{n_worse}/3 discriminators -> the fault is out-of-distribution payload. "
                "Retrain with the widened envelope (plan-d) is the correct fix.")
    elif not a_ok:
        verdict = "POLICY_SUSPECT"
        note = ("Bare (trained body) is ALSO unhealthy -> the policy itself is suspect; a "
                "same-recipe retrain may not help. Diagnose the policy/training before firing.")
    else:
        verdict = "INCONCLUSIVE"
        note = (f"Bare healthy but loaded worse on only {n_worse}/3 discriminators -> payload "
                "degradation is not clearly established. Re-examine segments / thresholds.")

    result = {
        "verdict": verdict, "note": note,
        "bare_healthy": a_ok,
        "discriminators": {
            "tracking_rel_err": {"bare": a_track, "loaded": b_track, "loaded_worse": track_worse},
            "pitch_var_rad2": {"bare": a_pv, "loaded": b_pv, "loaded_worse": pitch_worse},
            "fall_rate": {"bare": a_fall, "loaded": b_fall, "loaded_worse": fall_worse},
        },
        "n_discriminators_worse": n_worse,
    }
    print(json.dumps(result, indent=2))
    if args.out:
        with open(args.out, "w") as f:
            f.write(json.dumps(result) + "\n")
    # exit 0 on OOD_CONFIRMED (proceed to retrain), 2 on POLICY_SUSPECT (stop), 1 inconclusive.
    return {"OOD_CONFIRMED": 0, "INCONCLUSIVE": 1, "POLICY_SUSPECT": 2}[verdict]


if __name__ == "__main__":
    import sys
    sys.exit(main())
