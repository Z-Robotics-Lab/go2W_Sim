# navstack customization ownership manifest

`refs/Navigation-Physical-Experiment` is a **git-ignored** clone of our upstream fork
(`Yuxin916/Navigation-Physical-Experiment` @ `jazzy`). A fresh clone on a new machine is
**pristine upstream** and loses every local customization. Three mechanisms restore them.
This manifest is the authoritative map: **every customized file belongs to exactly one
mechanism** — the sets are mutually exclusive and jointly cover all customizations.

New-machine order (all offline; no sim/container needed for the file layer):
1. `git clone -b jazzy … refs/Navigation-Physical-Experiment`
2. `scripts/nav/patch_navstack.sh refs/Navigation-Physical-Experiment`
   ← runs **A, then B (calls `sync_navstack_files.sh`), then C (calls `navstack_patch/apply.sh`)**
   in one shot. The file layer is fully restored after this single command.
3. (only if hand-tweaking) `scripts/nav/navstack_patch/apply.sh <NAV>` re-applies **C** alone —
   idempotent, and already invoked by step 2, so a fresh machine does NOT need it separately.
4. in-container build of `arise_slam_mid360 (+_msgs)` — patch 01 is C++, and this pkg is in the
   main build's `--packages-skip`, so it is NOT otherwise compiled; **mandatory** or SLAM lacks
   the module / the de-tilt. Then `docker commit` → `navstack:ready`. (user-manual §1.5 step 4b.)

---

## Mechanism A — `patch_navstack.sh` (sed/python transforms; regenerated each run)

Deterministically re-derived from the pristine clone. Not stored as files; produced by transforms.

| File | Transform |
|------|-----------|
| `src/slam/arise_slam_mid360/config/*.yaml` (use_sim_time) | sed false→true |
| `src/slam/arise_slam_mid360/launch/arize_slam.launch.py` (SetParameter use_sim_time) | sed false→true |
| `src/base_autonomy/**/launch/*.launch` (use_sim_time param inject) | python rglob; **idempotent** (skips if already present) — covers terrain_analysis, terrain_analysis_ext, visualization_tools, waypoint_example, local_planner, ... |
| `system_isaac_sim.launch.py` | generated from system_real_robot.launch |
| `system_isaac_sim_with_exploration.launch.py` | generated (also snapshotted for sync B) |
| `src/exploration_planner/tare_planner/config/indoor_small.yaml` (kAutoStart) | sed true→false |
| `src/slam/.../livox_mid360_calibration.yaml` (imu_laser_rotation_offset 20deg) | sed (then overwritten by sync B's true-source copy) |

## Mechanism B — `sync_navstack_files.sh` (true source in `scripts/nav/`, copied in)

Files whose canonical source lives in `scripts/nav/`. Called by patch_navstack.sh (step 6)
and again before every container start (bringup/restart) to defeat stale copies.

| Copied into refs | True source |
|------------------|-------------|
| `pc2_to_livox.py`, `run_navstack.sh`, `agent_bridge.py`, `run_all_forever.sh` | `scripts/nav/<same>` |
| `src/slam/arise_slam_mid360/config/livox_mid360.yaml` | `scripts/nav/livox_mid360.yaml` — **contains `sensor_mount_pitch_deg: 20.0` + `use_sim_time: true`** |
| `src/slam/arise_slam_mid360/config/livox/livox_mid360_calibration.yaml` | `scripts/nav/livox_mid360_calibration.yaml` — 20° IMU↔laser extrinsic |
| `system_isaac_sim_with_exploration.launch.py` | `scripts/nav/*.reference` |
| `go2w.rviz` | generated (TeleopPanel stripped) |

> **Note on commit 36b8e1b** (`feat(arise): sensor_mount_pitch_deg`): it touched 3 files.
> Two are yaml → **owned by mechanism B** (true source already carries both the 20° param and
> use_sim_time; verified byte-identical to refs). Only the C++ file needs mechanism C — see
> patch 01. This is why 36b8e1b is **not** re-applied as a whole-commit `format-patch`.

## Mechanism C — `navstack_patch/apply.sh` (this dir; refs-only edits as static patches)

Customizations reproduced by **neither** A nor B. Patch base = the **post-patch_navstack**
pristine clone (so patches compose on top of A's use_sim_time injections). Applied AFTER A.

| Patch | File | Change | Provenance |
|-------|------|--------|------------|
| `01_sensor_mount_pitch_deg.patch` | `src/slam/arise_slam_mid360/src/ImuPreintegration/imuPreintegration.cpp` | `sensor_mount_pitch_deg` param + publish-time Ry(-pitch) de-rotation | C++ half of commit 36b8e1b (committed-unpushed). **Needs in-container rebuild** of `arise_slam_mid360`. |
| `02_local_planner_obstacleHeightThre.patch` | `src/base_autonomy/local_planner/launch/local_planner.launch` | `obstacleHeightThre` 0.05→0.20 | fix-c (terrain-cost flicker root-cause). Was uncommitted in refs. |
| `03_arize_slam_respawn.patch` | `src/slam/arise_slam_mid360/launch/arize_slam.launch.py` | `respawn=True` + `respawn_delay=3.0` on the 3 SLAM nodes | uncommitted refs-only robustness edit. |

---

## Deliberately EXCLUDED from every mechanism (fix-a WIP)

`src/base_autonomy/local_planner/localPlanner.cpp` **and** the `groupSwitchRatio` param line in
`local_planner.launch` are the **active WIP of another session** (`fix-a`, group-selection
hysteresis; COORDINATION.md: "任何会话不要动 local_planner/该参数"). They are:

- **NOT** in patch 02 (verified: patch 02 contains `obstacleHeightThre` but zero
  `groupSwitchRatio`). Omitting the param is behaviorally identical to its current live value —
  the C++ default `groupSwitchRatio=1.0` means hysteresis-off = upstream-equivalent.
- **NOT** synced. **NOT** patched. They will not be reproduced by this mechanism on a new
  machine, by design. When fix-a is accepted and committed upstream (or promoted to a
  customization), it gets its own patch/sync entry here — until then it stays out.

This exclusion is the one place the "every customization has an owner" rule is intentionally
suspended: fix-a is *in-flight*, not an accepted customization, and is owned by its session's
worktree, not by refs persistence.
