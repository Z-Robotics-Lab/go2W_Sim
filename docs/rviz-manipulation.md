# RViz mobile-manipulation view

The normal `GO2W_SCENE=office NAV_MODE=waypoint` bringup now loads one combined
RViz configuration.  It keeps the upstream navigation displays in one compact
group and adds two live dock panes plus grouped 3-D manipulation diagnostics.
It is an observer:
none of these displays or the status bridge publish motion commands, and none
subscribe to `/ground_truth/*` or `/objects/*`.

## Default layout

- `Navigation | Registered Scan + Path`: `/registered_scan` plus the line-only
  navigation path. Planner tuning products such as the vehicle axes,
  free-path fan, waypoint, boundary, duplicate maps, and goal-pose glyph stay
  available but default off.
- `Perception | Wrist RGB [LIVE]`: `/camera/color/image_raw`.
- `Perception | Aligned Depth [LIVE]`:
  `/camera/aligned_depth_to_color/image_raw`, normalized by RViz for inspection.
- `Perception 3D (measured RGB-D only)`: validated target cloud, scene collision
  cloud, and target pose.  These stay disabled until their upstream publisher
  starts.
- `Manipulation Planning (no task ground truth)`: robot model, optional 6-DoF
  grasp candidates, the live selected pregrasp/grasp markers from
  `/z_manip/debug/markers`, the live planned TCP path from
  `/z_manip/debug/arm_path`, executed TCP paths, perceived collision markers,
  and Octomap occupied cells. Missing-upstream items are explicitly named and
  disabled so a minimal navigation deployment stays clean. Candidate grasps,
  executed-path history, target-pose glyphs, and key-frame axes are opt-in;
  every `nav_msgs/Path` is rendered as a line without per-pose axes.
- `Manipulation Diagnostics`: live standard `visualization_msgs/Marker` text for
  `/piper/execution_status` and `/z_manip/perception/status` plus
  `/z_manip/perception/valid`.  Gray means unavailable/stale, amber means active
  or warning, green means ready/valid, and red means rejected/error.
- `TF (navigation + wrist camera)`: navigation and wrist optical frames.

RViz's standard `Image` display creates dock panes.  The RGB, depth, overlay,
and mask panes can be docked over one another as a perception tab set; RViz
persists the operator's dock arrangement.  The Displays tree is visible on
startup so inactive upstream products can be enabled without editing YAML.
The Views panel also contains
`Navigation Overview`, `Mobile Manipulation`, and `Wrist Camera TF` views.

## Topic contract

The single source of topic names is
`configs/rviz/manipulation_topics.json`.  The Z-Manip publishers should use the
same sim/real-neutral `/z_manip/*` names.  For a deployment that remaps topics,
the config builder accepts environment overrides such as:

```bash
GO2W_RVIZ_TARGET_CLOUD_TOPIC=/my/validated_target_cloud \
GO2W_RVIZ_SCENE_CLOUD_TOPIC=/my/perceived_scene \
GO2W_SCENE=office NAV_MODE=waypoint bash scripts/nav/bringup.sh up
```

Every override is listed in `scripts/nav/build_rviz_config.py`.  The status
bridge additionally exposes its input/output names, fixed frame, stale timeout,
and publish period as ROS parameters.  The generated runtime file is
`refs/Navigation-Physical-Experiment/go2w.rviz`.

## PiPER execution identity

`/piper/execution_status` remains a semicolon-delimited `std_msgs/String`; its
first field is still the trajectory state and the legacy `gripper` and measured
`aperture` fields remain present. Accepted commands now add executor-owned,
strictly increasing identities:

```text
active;owner=trajectory;command_id=4;segment=place_approach;
trajectory_contract_id=place-goal-7;executor_epoch=4f8c...;
trajectory_received_at=81.230000;gripper=accepted:0.0440;
gripper_command_id=2;gripper_command_aperture=0.0440;
gripper_received_at=81.110000;gripper_event_received_at=81.110000;
aperture=0.0452
```

`place_approach` and `place_retreat` commands must encode the same bounded
transaction ID in `JointTrajectory.header.frame_id` as
`<segment>|contract=<place_goal_id>`. The executor allocates a process-unique
`executor_epoch` and echoes both identities with its monotonic command ID and
source receive time. Release verification can therefore reject retained
status, executor restarts, cross-goal messages, and approach/retreat ID reuse.

The trajectory `segment` is read from `JointTrajectory.header.frame_id` and
must belong to the executor's explicit segment allowlist. Place approach and
retreat additionally require the contract suffix above. The executor assigns
`command_id` only after joint names, limits, timing, velocity, start state, sim
time, and segment all validate. The independent `gripper_command_id` is
allocated only after an aperture and its sim receive time validate. A rejected
message may report a rejection state, but it cannot increment or replace the
last accepted command ID, segment, aperture, or accepted receive time.

## Plugin policy

The configuration uses only `rviz_common/Group` and plugins shipped by
`rviz_default_plugins` on ROS 2 Jazzy.  It intentionally does not require
MoveIt RViz, vision-msgs RViz, or custom teleoperation plugins.  The overlay and
mask are ordinary `sensor_msgs/Image`; grasp and collision visualization uses
standard `visualization_msgs/Marker{,Array}`; TCP trajectories use
`nav_msgs/Path`.
