# RViz mobile-manipulation view

The normal `GO2W_SCENE=office NAV_MODE=waypoint` bringup now loads one combined
RViz configuration.  It keeps every upstream navigation display and adds two
live dock panes plus grouped 3-D manipulation diagnostics.  It is an observer:
none of these displays or the status bridge publish motion commands, and none
subscribe to `/ground_truth/*` or `/objects/*`.

## Default layout

- `Perception | Wrist RGB [LIVE]`: `/camera/color/image_raw`.
- `Perception | Aligned Depth [LIVE]`:
  `/camera/aligned_depth_to_color/image_raw`, normalized by RViz for inspection.
- `Perception 3D (measured RGB-D only)`: validated target cloud, scene collision
  cloud, and target pose.  These stay disabled until their upstream publisher
  starts.
- `Manipulation Planning (no task ground truth)`: robot model, 6-DoF grasp
  candidates, selected grasp/pregrasp, planned/executed TCP paths, perceived
  collision markers, and Octomap occupied cells.  Missing-upstream items are
  explicitly named and disabled so a minimal navigation deployment stays clean.
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

## Plugin policy

The configuration uses only `rviz_common/Group` and plugins shipped by
`rviz_default_plugins` on ROS 2 Jazzy.  It intentionally does not require
MoveIt RViz, vision-msgs RViz, or custom teleoperation plugins.  The overlay and
mask are ordinary `sensor_msgs/Image`; grasp and collision visualization uses
standard `visualization_msgs/Marker{,Array}`; TCP trajectories use
`nav_msgs/Path`.
