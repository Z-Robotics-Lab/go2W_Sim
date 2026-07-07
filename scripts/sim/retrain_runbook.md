# Go2W policy retrain runbook — payload-envelope fix (CEO-approved 2026-07-07)

All offline prep is in git. This runbook is executed at a **clean sim window** (ONE-sim rule).
Nothing here starts training or a sim automatically — you run the steps deliberately.

## 0. Why we retrain (root cause, from the vx-audit — do not re-litigate)

The shipped policy `robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-04_15-52-42/model_1999.pt`
was trained on a **6.92 kg bare trunk**. Deployment bolts on a **~6.5 kg front-offset payload**
(PiPER 4.66 kg + NUC/mounts; CoM shifts +6.5 cm forward / +7.5 cm up). That pushes the base
complex **1.3-2.2x outside** the trained randomization envelope (base mass `+3 kg` cap, CoM
`±5 cm`), and `obs57` is blind to the load. Symptoms: zero-cmd creep (0.075 m/s), low-cmd
forward over-shoot (0.15→0.257, 0.30→0.383, 0.60→0.644 m/s), degraded gait at command edges.

**Fix (plan-d, default):** widen ONLY the base mass/CoM randomization so the payload sits inside
training. **Everything else is frozen** (rewards, command distribution, actuator gains,
decimation, obs57/act16) — the deployment-consistency red line — so the new ckpt stays
**isomorphic to the frozen shim** `scripts/sim/go2w_policy.py` and is drop-in.

## 0.5. FIRST SIM EXPERIMENT — confirm it's the payload, not the policy (A/B judgement)

**Do this BEFORE training.** A 30-minute retrain is wasted if the fault is actually in the policy
itself. The A/B experiment isolates the cause: run the SHIPPED (old) ckpt on two bodies and
compare.

- **A = bare** (`go2w_bare.urdf`, auto-derived by `make_bare_urdf.py`) — the ~6.92 kg trunk the
  policy was TRAINED on.
- **B = loaded** (`go2w_sensored.urdf`) — the ~6.5 kg front-offset deployment payload.

```bash
# regenerate the bare control body (offline, no sim) if the sensored URDF changed:
python3 scripts/sim/make_bare_urdf.py

# then, at a clean sim window, run the SAME old ckpt on each body:
CKPT=/workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/2026-07-04_15-52-42/model_1999.pt
docker exec -u 0 go2w-isaac bash -lc "cd /workspace/go2w/scripts/sim && \
  /isaac-sim/python.sh policy_acceptance.py --policy $CKPT --body bare \
    --out /workspace/go2w/logs/ab_bare.jsonl --headless"
docker exec -u 0 go2w-isaac bash -lc "cd /workspace/go2w/scripts/sim && \
  /isaac-sim/python.sh policy_acceptance.py --policy $CKPT --body loaded \
    --out /workspace/go2w/logs/ab_loaded.jsonl --headless"

# offline verdict (no sim):
python3 scripts/sim/ab_verdict.py --bare logs/ab_bare.jsonl --loaded logs/ab_loaded.jsonl
```
Verdict (exit code): `OOD_CONFIRMED` (0) → the payload is out-of-distribution; **proceed to
retrain (plan-d).** `POLICY_SUSPECT` (2) → the policy is bad even on its trained body; **STOP —
a same-recipe retrain won't help, diagnose the policy first.** `INCONCLUSIVE` (1) → re-examine.

Discriminators: tracking rel-error, body-pitch variance, fall rate — B must be materially worse
than A on ≥2 of 3, AND A must itself be healthy, to conclude OOD. This gate is the "确定是 policy
问题" check the CEO demanded before firing.

`warehouse_nav.py` also honours the same switch for eyeballing in the warehouse:
`GO2W_WITH_ARM=0` loads the bare body and skips the PiPER grasp controller (all arm/grasp writes
are None-guarded); `=1` (default) is the loaded deployment body.

## 1. What the diff actually changes

Config source is git-ignored vendored `robot_lab`, so the change lives as a **tracked patch** in
this repo and is re-applied after any `clone_deps.sh`:

- `scripts/sim/retrain/robot_lab_patch/go2w_payload_envelope.patch` — plan-d + plan-a registration
- `scripts/sim/retrain/robot_lab_patch/payload_env_cfg.py` — plan-a variant cfg (opt-in)
- `scripts/sim/retrain/robot_lab_patch/apply.sh` — idempotent applier (wired into `clone_deps.sh`)

**Plan-d** (`rough_env_cfg.py __post_init__`, inherited by flat):
```
randomize_rigid_body_mass_base.mass_distribution_params : (-1.0, 3.0)  ->  (-1.0, 8.0)   # +3kg -> +8kg
randomize_com_positions.com_range x/y/z                 : ±0.05        ->  ±0.10          # ±5cm -> ±10cm
```
Trained values verified against the shipped ckpt's `params/env.yaml`. Override placed in the
go2w-specific cfg (NOT the shared parent `velocity_env_cfg.py`) so no other robot is touched —
same idiom the `unitree_go2` sibling already uses for `body_names`.

**Plan-a** (opt-in fallback, task id `RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-Payload-v0`):
base mass-add centered on the real payload `(5.0, 8.0)` + front/up-biased CoM
`x:(0.03,0.10) z:(0.03,0.10)`. See §6.

To (re)apply by hand: `scripts/sim/retrain/robot_lab_patch/apply.sh`

## 2. Preflight (the launcher enforces these; know them anyway)

Host rules (`rules/common/performance.md`, AGENTS.md Inv.5/Inv.8):
- **ONE sim at a time.** `run_retrain.sh` refuses if any `warehouse_nav.py`/`warehouse_scene.py`
  or `isaac-sim/kit`/`python.sh` process is live. The `go2w-isaac` container is SHARED — if a
  sibling holds it, **WAIT**; never kill it.
- **Memory:** need `free -g` available ≥ 30 G. The container is already `--memory=40g`
  (setup_container.sh); the host-side `docker exec` is additionally wrapped in
  `systemd-run --scope -p MemoryMax=45G`.
- **No `ulimit -v`** for this torch/CUDA run — CUDA reserves tens of GB of *virtual* address
  space that is never physically backed; a `-v` cap would spuriously kill a bounded GPU run.
  Real memory is bounded by ONE-sim serialization + the container/systemd caps.
- **GPU idle:** `nvidia-smi` should show only a few hundred MiB used (a live sim holds several GB).

Manual check before you launch:
```bash
pgrep -fa "warehouse_nav.py|warehouse_scene.py|isaac-sim/kit|/isaac-sim/python.sh"   # want: empty
free -g | awk '/^Mem:/{print $7" G free"}'                                            # want: >= 30
nvidia-smi --query-gpu=memory.used --format=csv,noheader                              # want: idle
docker ps --filter name=go2w-isaac --format '{{.Names}} {{.Status}}'                   # want: Up
```

## 3. Launch training (plan-d, foreground)

```bash
cd ~/Desktop/go2w
scripts/sim/run_retrain.sh          # plan-d, 2048 env, 2000 iter, seed 42, ~30 min on 5080
```
It runs, inside `go2w-isaac`:
```
/isaac-sim/python.sh robot_lab/scripts/reinforcement_learning/rsl_rl/train.py \
    --task RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0 \
    --headless --num_envs 2048 --max_iterations 2000 --seed 42
```
Overrides via env: `GO2W_RETRAIN_ENVS`, `GO2W_RETRAIN_ITERS`, `GO2W_RETRAIN_SEED`,
`GO2W_RETRAIN_MEMMAX`, `GO2W_RETRAIN_PLAN` (`d`|`a`).

**Anchors (match the shipped ckpt):** 2048 env / 2000 iter / ~30 min. The shipped ckpt is proof
this converges in that budget on this hardware.

Output: `robot_lab/logs/rsl_rl/unitree_go2w_flat/<NEW_TIMESTAMP>/` with `model_*.pt` +
`params/{env,agent}.yaml`. Both plan-d and plan-a use `experiment_name=unitree_go2w_flat`, so the
new ckpt lands **beside** the shipped one — distinguish by timestamp. The launcher prints the
newest dir at the end.

## 4. Training verification discipline (HARD checkpoints — don't skip)

**4a. First thing after fire — verify the config actually took effect.** A cfg that silently
didn't apply = 30 wasted minutes. As soon as the new run writes its `params/env.yaml`, confirm
the widened envelope is really there:
```bash
NEWDIR=$(ls -dt robot_lab/logs/rsl_rl/unitree_go2w_flat/*/ | head -1)
grep -A2 'randomize_rigid_body_mass_base' "$NEWDIR/params/env.yaml" | grep -A2 mass_distribution_params
#   expect:  - -1.0   /   - 8.0        (plan-d: +8kg upper bound, NOT 3.0)
grep -A6 'randomize_com_positions' "$NEWDIR/params/env.yaml" | grep -A1 -E ' x:| y:| z:'
#   expect:  - -0.1 / - 0.1  on x,y,z  (plan-d: ±10cm, NOT ±0.05)
```
If these still show `3.0` / `0.05`, the patch did not apply — STOP, run
`scripts/sim/retrain/robot_lab_patch/apply.sh`, re-check `git -C robot_lab diff`, relaunch.

**4b. Mid-run — compare against the shipped run's tfevents anchors; bail on clear divergence.**
The shipped ckpt's curves are the anchor (`.../2026-07-04_15-52-42/events.out.tfevents...`). If
by ~500 iter the new run's mean reward is not tracking toward the old run's trajectory (still
near-zero / negative, episode length not rising), **stop and diagnose** — do not hard-run to 2000.
Widening the envelope should slow convergence only modestly, not break it.

**4c. Post-train — anchor on the OLD ckpt first.** Run the acceptance battery on the OLD ckpt
FIRST (same suite, same segment order), then the NEW ckpt, then diff the JSON rows side by side.
Same loader/feed/judge → the only variable is the ckpt.

**4d. Rollback discipline.** The `GO2W_POLICY` default in `bringup.sh`/`restart_all.sh` is changed
ONLY after the full pre-registered criteria pass (§8). When you change it, leave the old ckpt path
as a comment so rollback is one edit away. Keep the old ckpt file until the new one is proven on
the real face.

## 5. Monitoring mid-run (what "healthy" looks like)

Host log: `logs/retrain_d_<TS>.log` (tee'd). From another shell:
```bash
tail -f logs/retrain_d_<TS>.log
grep -E "Mean reward|Mean episode length" logs/retrain_d_<TS>.log | tail
```
rsl_rl prints a per-iteration block. Watch:
- **Mean reward** rises over the first few hundred iters and plateaus. Flat locomotion on this
  cfg should reach a healthy positive mean reward well before iter 2000.
- **`stand_still` reward** (weight -2.0, penalizes leg motion under zero cmd): should trend
  toward ~0 (less violated) as the policy learns to stand — this is the term most relevant to
  the zero-cmd creep symptom. If it stays strongly negative late, the creep may persist →
  escalate to plan-a.
- **`track_lin_vel_xy_exp` / `track_ang_vel_z_exp`** (velocity tracking, weights 3.0/1.5) should
  climb — this is the command-following the low-cmd ladder tests.
- **Mean episode length** rising = fewer early terminations (falls) under the wider payload.

Rough convergence expectation: reward curve knee by ~300-600 iter; stand-still should be visibly
converging (not still crashing) by ~800-1000 iter. If reward is flat/negative past ~1000 iter,
something is off (check the cfg applied — `git -C robot_lab diff` should show the +8/±0.10 edit).

## 6. Acceptance (after training completes — run at a clean sim window)

Run the four-segment symptom battery on the payload deployment form, inside the container:
```bash
docker exec -u 0 go2w-isaac bash -lc '
  cd /workspace/go2w/scripts/sim && \
  /isaac-sim/python.sh policy_acceptance.py \
    --policy /workspace/go2w/robot_lab/logs/rsl_rl/unitree_go2w_flat/<NEW_TS>/model_1999.pt \
    --out /workspace/go2w/logs/acceptance_new_<NEW_TS>.jsonl --headless'
```
Then run the SAME command against the OLD ckpt (`.../2026-07-04_15-52-42/model_1999.pt`,
`--out .../acceptance_old.jsonl`) and diff the JSON rows — same loader/feed/judge, only the
ckpt changes, so any delta is the retrain's doing.

The per-segment `pass` field is a coarse in-script sanity flag. The **binding** go/no-go is the
PRE-REGISTERED criteria (§9, written into `docs/sim-plan.md` to stop goalpost-moving). Those are
computed by diffing new-vs-old JSON rows — see §9 for the exact 4 conditions and the d→a decision
table.

Acceptance runs on `go2w_sensored.urdf` (the arm+NUC deployment body) with **training gains**
(legs 25/0.5, wheels 0/0.5) — so it exercises the real load state by construction.

**Payload fidelity caveat:** the URDF's `nuc_weight` link is a **0.5 kg placeholder**, so the
URDF payload (~5.16 kg over bare trunk) is *lighter* than the audited ~6.5 kg. If segment
verdicts pass only marginally, re-run acceptance with a heavier `nuc_weight` mass in
`assets/urdf/go2w_sensored.urdf` (bump 0.5 → ~1.8 kg to hit ~6.5 kg total payload) to confirm
the margin holds at true deployment mass. (URDF mass is a hardware-model change — treat a bump
as a deliberate edit, not a silent tweak.)

## 7. Plan-a escalation (only if plan-d acceptance fails)

If plan-d does not clear the bar (e.g. creep still ≥ 0.02, or falls under wz steps), the fixed
front-offset payload is trained explicitly. **CEO-approved as the pre-planned escalation.**
```bash
GO2W_RETRAIN_PLAN=a scripts/sim/run_retrain.sh    # task: ...-Go2W-Payload-v0
```
Plan-a keeps the 16-DOF locomotion morphology (legs+wheels only) — it does NOT swap in the armed
URDF for training, because that would add 8 arm DOFs and break `obs57/act16` isomorphism with the
frozen shim. Instead it bakes the payload into base mass/CoM randomization (centered `5-8 kg`,
CoM front/up-biased). Accept the plan-a ckpt with the same §6 battery.

## 8. Deployment switch (after acceptance passes — this is the go-live)

1. Point deployment at the new ckpt: set `GO2W_POLICY` to the new container path, or update the
   default in `scripts/nav/bringup.sh` and `scripts/nav/restart_all.sh` (currently
   `.../2026-07-04_15-52-42/model_1999.pt`).
2. Restart the nav stack (`scripts/nav/restart_all.sh`) at a clean sim window.
3. **Gait re-verification** on the real acceptance face (bare stack + real cmd_vel, eyes on the
   sim): re-run the DEBUG.md gait checks (zero-cmd idle drift, low-cmd tracking, wz-step
   stability) to confirm the deployed behavior matches acceptance. Only then is the switch done.
4. Keep the old ckpt in place (do not delete) until the new one is proven on the real face.

## 9. Pre-registered acceptance criteria (frozen in docs/sim-plan.md — no goalpost-moving)

All measured on the **loaded** body (`go2w_sensored.urdf`), new ckpt vs the old ckpt run through
the same battery:

1. **Zero-cmd drift** < 0.02 m/s (seg 1).
2. **0.3-cmd tracking error improved ≥ 50%** vs old ckpt (seg 2 `2_ladder_vx_0.3` rel_err).
3. **wz=±1.4 step ×5: zero falls** (seg 3 fall_rate == 0).
4. **0.6-cmd tracking not degraded** vs old ckpt (seg 2 `2_ladder_vx_0.6` no worse).

Decision:
| Outcome | Meaning |
|---|---|
| ①②③④ all pass | **plan-d SUCCESS** → proceed to §8 deployment switch |
| ①③ pass, ② misses | **usable but escalate to plan-a** and re-iterate |
| ① or ③ fails | **plan-d FAILED** → straight to plan-a (§7) |

These are frozen before firing so the verdict can't be rationalized after the fact.

## 10. Pre-fire checklist (run through this before starting training)

- [ ] **Cause confirmed**: §0.5 A/B verdict = `OOD_CONFIRMED` (NOT `POLICY_SUSPECT`). If policy
      suspect, DO NOT fire — diagnose the policy first.
- [ ] **Sim slot free**: `pgrep -fa "warehouse_nav.py|warehouse_scene.py|isaac-sim/kit|/isaac-sim/python.sh"`
      is empty. No sibling holds the go2w-isaac container. (ONE-sim; never kill a sibling.)
- [ ] **Memory**: `free -g` available ≥ 30 G; no other session holding tens of GB.
- [ ] **GPU idle**: `nvidia-smi` shows a few hundred MiB, not a resident sim.
- [ ] **Container up**: `docker ps --filter name=go2w-isaac` = Up.
- [ ] **Config present**: `git -C robot_lab diff` shows the +8kg/±10cm edit (or
      `scripts/sim/retrain/robot_lab_patch/apply.sh` was run on this checkout).
- [ ] **Baseline anchored**: old-ckpt acceptance JSONL saved (for the new-vs-old diff and the §9
      ②④ comparisons).
- [ ] **Pre-registered criteria (§9) read and agreed** — you will not move them after seeing results.
- [ ] **Launch is FOREGROUND** via `scripts/sim/run_retrain.sh` (it re-checks the ONE-sim/mem/GPU
      guards and refuses if the slot is busy). Right after fire, do §4a config verification.
