# Stability & Deploy A/B — Pre-registered Gates (frozen wording)

Pre-registered acceptance gates for Package 1 (stance + parking) and Package 2
(deploy A/B). **DEFINITIONS ONLY — none of these have been run.** The sim
measurement windows are scheduled separately by the main session (this was a
pure dev round: no sim / container / process was touched).

Pre-registration discipline (AGENTS.md): the thresholds and wording below are
frozen BEFORE any run. A cleared bar is recorded via a `provisional` ledger row
first, promoted only after a round boundary + red-team. Numbers a source branch
"claims" (codex) are reference only, never evidence.

Acceptance face (AGENTS invariant 2): bare `vector-cli` REPL + natural language,
eyes on the sim. These probes/gates RECORD verdicts from that face; they never
replace it.

---

## Package 1 gates (stance + parking)

### S1 — Standstill (stance-hold) gate

Config: payload body (arm + NUC mass present, `GO2W_WITH_ARM=1`), **zero command**
(no nav goal, no `/manip/cmd_vel`), **60 sim-seconds** after settle.

Pass criteria (ALL must hold):
- **0 falls** over the window.
- **|pitch| < 5°** throughout (body stays upright).
- **When parking is engaged: base XY drift < 0.05 m** over the 60 s.
  - Reference (not evidence): the codex branch measured zero-command creep of
    0.075 m/s ⇒ with NO parking brake a 60 s window drifts ~4.5 m. The gate
    asserts the parking brake kills that creep to < 0.05 m total.

Arms (A/B), recorded as separate rows:
- **A = legs 25 / 0.5** (`GO2W_POLICY_LEG_STIFFNESS=25.0 GO2W_POLICY_LEG_DAMPING=0.5`,
  i.e. the current default, training-native gains).
- **B = legs 100 / 5** (`GO2W_POLICY_LEG_STIFFNESS=100.0 GO2W_POLICY_LEG_DAMPING=5.0`,
  the 4× payload-calibrated gains).

Parking off/on recorded as separate sub-rows for each arm
(`GO2W_STANDSTILL=0` vs `GO2W_STANDSTILL=1`), so the parking brake's drift
contribution is isolated from the leg-gain contribution.

Probe: base XY from GT odom (`/objects/...` is a read-only oracle, actor does not
consume it); pitch from IMU / GT; fall = body height collapse or |pitch| ≥ 5°.

### S2 — Motion no-regression gate

Config: payload body, two motion segments:
- **vx = 0.5 m/s for 20 sim-s** (straight drive).
- **wz = 1.0 rad/s for 20 sim-s** (in-place yaw).

Pass criteria (ALL must hold):
- **Regression ≤ 10%** vs the G2-retest baseline for each segment. Baseline
  anchor: **wz = 1.0 → 0.9583 rad/s** measured (G2 retest, model_7494). So the
  yaw segment passes if measured wz ≥ 0.9583 × 0.90 = **0.8625 rad/s**.
- **0 falls** on both segments.

Rationale: S1 (parking + leg gains) must not degrade normal locomotion. The
standstill machine's MANIP-override and exit hysteresis must release cleanly so a
real drive/yaw command is never eaten (this is the same regression hard-constraint
that the deadzone-v2 EXIT=0.25 band already guards).

---

## Package 2 — deploy A/B measurement (env-switched; launch surface FROZEN)

During any A/B the launch surface is frozen: switch arms ONLY via env, never edit
files mid-experiment (shared-sim gate discipline, COORDINATION.md 2026-07-07
lesson). All switches below are generation-time (`sync_navstack_files.sh`) or
launch-time; runtime is frozen.

### 2a — IMU route: `GO2W_IMU_ROUTE=rotate|raw`

navstack-side extrinsic config, per route (values READ from this branch, 2026-07-13):

| Item | rotate (default = main current) | raw (codex-ported) |
|---|---|---|
| sim IMU acc/gyr | rotated to trunk (level-IMU) frame | raw passthrough (20°-pitched sensor frame) |
| sim IMU orientation | `Ry(+20°) ⊗ q_imu_raw` | `q_imu_raw` |
| lidar `frameId` | `sensor` | `mid360_raw` |
| `livox_mid360_calibration.yaml` → `imu_laser_rotation_offset` | **`[0.0, 20.0, 0.0]`** (degrees; "laser pitched +20° vs a horizontal IMU") — file's current true value | **`[0.0, 0.0, 0.0]`** (IMU & laser co-pitched, no relative rotation) — set by `sync_navstack_files.sh` sed when `GO2W_IMU_ROUTE=raw` |
| `livox_mid360_calibration.yaml` → `extrinsicRotation_imu_laser` | identity 3×3 (unchanged both routes) | identity 3×3 (unchanged) |
| `livox_mid360.yaml` → `init_pitch` | `0.0` (mapping mode; gated off by `local_mode:false`) | `0.0` (unchanged) |
| gravity alignment owner | split: sim pre-levels IMU + navstack applies 20° extrinsic | ARISE self-aligns from the raw pitched IMU |

Current actual value in-repo: `imu_laser_rotation_offset = [0.0, 20.0, 0.0]`
(rotate route, verified live 2026-07-08: both SLAM nodes `updated pitch:20.0`,
raw ground-normal-vs-sensor-Z = 19.70°, map ~1.65–1.81° < 2°). The raw route's
`[0,0,0]` is applied generation-time and frozen for the run; **never both a rotate
orientation and a raw extrinsic — that is the double-compensation that turns a
self-consistent level map into a real 20°/40° tilt.**

### 2b — bringup order: `GO2W_BRINGUP_ORDER=nav_first|isaac_first`

- `nav_first` (default = main current): navstack → Isaac bridge → wait `imu sample`.
- `isaac_first`: Isaac bridge → wait `imu settled` (step ≥ 800 = 8 sim-s IMU
  settle) → navstack/RViz. Mechanism: ARISE estimates gravity once at startup;
  launching navstack first freezes the landing transient into the map tilt.

### 2c — DDS domain: `GO2W_ROS_DOMAIN_ID` (default 42)

Parameterized only; default 42 unchanged (both containers + host agent on 42).
A/B operating note: if arm B uses domain 184 for DDS isolation,
**zmanip-perception (hardcoded 42) goes off-network** — acceptable because P2
measurement does not need manip, but it must be stated in the A/B run sheet.

### 2e — P2 adjudication protocol (pre-registered, frozen)

For each arm (rotate / raw for 2a; nav_first / isaac_first for 2b):

1. **Paired restart ×2 per arm** (repeatability: two independent bringups of the
   same arm), launch surface frozen (md5 the bringup/scene/launch files before
   each restart per the shared-sim lesson).
2. After settle, measure the **`registered_scan` ground tilt** with
   `scripts/sim/probe_slam_tilt.py` (`--secs 12`, near-robot ground band), and
   record the ARISE IMU pitch it reports alongside.
3. **Winner = the arm whose |ground tilt| is smaller AND whose two paired
   restarts are mutually consistent.**
4. **Tie rule: if the margin between arms is < 0.3°, declare a tie; a tie holds
   the main-current default (rotate for 2a, nav_first for 2b).**
5. The codex branch's self-reported numbers (0.71° / 2.11°) are **reference
   only, never evidence**; only this branch's live `probe_slam_tilt.py` verdicts,
   recorded on the acceptance face, count.

Promotion of any winner into `docs/DECISIONS.md` follows AGENTS.md: bank a
`provisional` ledger row this window, promote only after a round boundary +
red-team.

---

## Env switch reference (this branch)

| Env | Default (main current) | Purpose | Consumed by |
|---|---|---|---|
| `GO2W_POLICY_LEG_STIFFNESS` | `25.0` | leg Kp (payload A/B B-arm = 100) | warehouse_nav.py |
| `GO2W_POLICY_LEG_DAMPING` | `0.5` | leg Kd (B-arm = 5) | warehouse_nav.py |
| `GO2W_STANDSTILL` | `1` | parking/standstill on/off | warehouse_nav.py |
| `GO2W_STANDSTILL_WHEEL_KP` | `100.0` | parked wheel position Kp (20→100: KP=20 let the arm-extension reaction roll parked wheels +13cm fore during a grasp sting, z-manip run65→67; park is standstill-only so nav is unaffected) | standstill_control.py |
| `GO2W_STANDSTILL_WHEEL_DAMPING` | `40.0` | parked wheel damping (8→40 with the Kp bump) | standstill_control.py |
| `GO2W_STANDSTILL_GAIN_TICKS` | `10` | drive↔park gain blend ticks | standstill_control.py |
| `GO2W_IMU_ROUTE` | `rotate` | IMU frame route (2a) | warehouse_nav.py + sync_navstack_files.sh |
| `GO2W_BRINGUP_ORDER` | `nav_first` | launch order (2b) | bringup.sh |
| `GO2W_ROS_DOMAIN_ID` | `42` | DDS domain (2c) | bringup.sh |
| `GO2W_OBS_DUMP` | (unset) | obs-dump instrumentation opt-in | bringup.sh → obs_dump.py |
