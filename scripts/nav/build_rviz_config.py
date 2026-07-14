#!/usr/bin/env python3
"""Build the combined navigation/manipulation RViz configuration.

The upstream navigation RViz file remains the source of its navigation
displays.  This builder removes the unsafe teleoperation panel structurally,
then adds standard Jazzy RViz displays for the measured manipulation contract.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
from typing import Any

import yaml


ENV_OVERRIDES = {
    "fixed_frame": "GO2W_RVIZ_FIXED_FRAME",
    "status_frame": "GO2W_RVIZ_STATUS_FRAME",
    "raw_lidar": "GO2W_RVIZ_RAW_LIDAR_TOPIC",
    "wrist_rgb": "GO2W_RVIZ_WRIST_RGB_TOPIC",
    "aligned_depth": "GO2W_RVIZ_ALIGNED_DEPTH_TOPIC",
    "camera_info": "GO2W_RVIZ_CAMERA_INFO_TOPIC",
    "detection_mask_overlay": "GO2W_RVIZ_OVERLAY_TOPIC",
    "target_mask": "GO2W_RVIZ_TARGET_MASK_TOPIC",
    "validated_target_cloud": "GO2W_RVIZ_TARGET_CLOUD_TOPIC",
    "scene_cloud": "GO2W_RVIZ_SCENE_CLOUD_TOPIC",
    "target_pose": "GO2W_RVIZ_TARGET_POSE_TOPIC",
    "candidate_grasps": "GO2W_RVIZ_GRASP_CANDIDATES_TOPIC",
    "selected_grasp": "GO2W_RVIZ_SELECTED_GRASP_TOPIC",
    "planned_tcp_path": "GO2W_RVIZ_PLANNED_TCP_PATH_TOPIC",
    "executed_tcp_path": "GO2W_RVIZ_EXECUTED_TCP_PATH_TOPIC",
    "collision_environment": "GO2W_RVIZ_COLLISION_MARKERS_TOPIC",
    "octomap_markers": "GO2W_RVIZ_OCTOMAP_MARKERS_TOPIC",
    "perception_diagnostics": "GO2W_RVIZ_PERCEPTION_STATUS_TOPIC",
    "perception_valid": "GO2W_RVIZ_PERCEPTION_VALID_TOPIC",
    "perception_status_marker": "GO2W_RVIZ_PERCEPTION_MARKER_TOPIC",
    "piper_execution_status": "GO2W_RVIZ_PIPER_STATUS_TOPIC",
    "piper_execution_status_marker": "GO2W_RVIZ_PIPER_MARKER_TOPIC",
    "robot_description": "GO2W_RVIZ_ROBOT_DESCRIPTION_TOPIC",
}


def load_contract(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("RViz topic contract must use schema_version 1")
    topics = data.get("topics")
    if not isinstance(topics, dict) or not topics:
        raise ValueError("RViz topic contract requires a non-empty topics object")
    resolved: dict[str, Any] = copy.deepcopy(data)
    for key, environment_name in ENV_OVERRIDES.items():
        value = os.environ.get(environment_name)
        if value:
            if key in ("fixed_frame", "status_frame"):
                resolved[key] = value
            else:
                resolved["topics"][key] = value
    for key, value in resolved["topics"].items():
        if not isinstance(value, str) or not value.startswith("/"):
            raise ValueError(f"RViz topic {key!r} must be an absolute ROS topic")
    return resolved


def _topic(value: str, *, reliable: bool = True) -> dict[str, Any]:
    return {
        "Depth": 5,
        "Durability Policy": "Volatile",
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable" if reliable else "Best Effort",
        "Value": value,
    }


def _image(name: str, topic: str, *, enabled: bool) -> dict[str, Any]:
    return {
        "Class": "rviz_default_plugins/Image",
        "Enabled": enabled,
        "Max Value": 1,
        "Median window": 5,
        "Min Value": 0,
        "Name": name,
        "Normalize Range": True,
        "Topic": _topic(topic, reliable=False),
        "Value": enabled,
    }


def _point_cloud(
    name: str,
    topic: str,
    *,
    color: str,
    enabled: bool = False,
    size_m: float = 0.006,
    decay_time: float = 0.0,
) -> dict[str, Any]:
    return {
        "Alpha": 0.85,
        "Autocompute Intensity Bounds": True,
        "Autocompute Value Bounds": {"Max Value": 10, "Min Value": -10, "Value": True},
        "Axis": "Z",
        "Channel Name": "intensity",
        "Class": "rviz_default_plugins/PointCloud2",
        "Color": color,
        "Color Transformer": "FlatColor",
        "Decay Time": decay_time,
        "Enabled": enabled,
        "Invert Rainbow": False,
        "Max Color": "255; 255; 255",
        "Max Intensity": 1,
        "Min Color": "0; 0; 0",
        "Min Intensity": 0,
        "Name": name,
        "Position Transformer": "XYZ",
        "Selectable": True,
        "Size (Pixels)": 3,
        "Size (m)": size_m,
        "Style": "Flat Squares",
        "Topic": _topic(topic, reliable=False),
        "Use Fixed Frame": True,
        "Use rainbow": False,
        "Value": enabled,
    }


def _marker(name: str, topic: str, *, enabled: bool = False) -> dict[str, Any]:
    return {
        "Class": "rviz_default_plugins/Marker",
        "Enabled": enabled,
        "Namespaces": {},
        "Topic": _topic(topic),
        "Value": enabled,
        "Name": name,
    }


def _marker_array(name: str, topic: str, *, enabled: bool = False) -> dict[str, Any]:
    return {
        "Class": "rviz_default_plugins/MarkerArray",
        "Enabled": enabled,
        "Namespaces": {},
        "Topic": _topic(topic),
        "Value": enabled,
        "Name": name,
    }


def _path(name: str, topic: str, *, color: str, enabled: bool = False) -> dict[str, Any]:
    # A Path can contain hundreds of poses.  Rendering every pose as an axes
    # glyph obscures both the trajectory line and the registered scan, so all
    # TCP paths are deliberately line-only.
    return {
        "Alpha": 1,
        "Buffer Length": 1,
        "Class": "rviz_default_plugins/Path",
        "Color": color,
        "Enabled": enabled,
        "Head Diameter": 0.08,
        "Head Length": 0.05,
        "Length": 0.08,
        "Line Style": "Billboards",
        "Line Width": 0.015,
        "Name": name,
        "Offset": {"X": 0, "Y": 0, "Z": 0},
        "Pose Color": color,
        "Pose Style": "None",
        "Radius": 0.01,
        "Shaft Diameter": 0.02,
        "Shaft Length": 0.05,
        "Topic": _topic(topic),
        "Value": enabled,
    }


def _axes(
    name: str,
    frame: str,
    *,
    length: float,
    radius: float,
    enabled: bool = True,
) -> dict[str, Any]:
    return {
        "Class": "rviz_default_plugins/Axes",
        "Enabled": enabled,
        "Length": length,
        "Name": name,
        "Radius": radius,
        "Reference Frame": frame,
        "Value": enabled,
    }


def _group(name: str, displays: list[dict[str, Any]], *, enabled: bool = True) -> dict[str, Any]:
    return {
        "Class": "rviz_common/Group",
        "Displays": displays,
        "Enabled": enabled,
        "Name": name,
        "Value": enabled,
    }


def augment_config(config: dict[str, Any], contract: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(config)
    panels = result.setdefault("Panels", [])
    # Keep the operator surface dense: the stock Selection/Tool Properties/
    # Views/Time docks are empty in this observer-only workflow and otherwise
    # squeeze the RGB-D perception docks into unreadable strips.
    panels[:] = [
        panel for panel in panels
        if panel.get("Class") == "rviz_common/Displays"
    ]

    manager = result.setdefault("Visualization Manager", {})
    # Manipulation is usable before SLAM has created map->base_link. Defaulting
    # RViz to map made a healthy local lidar cloud appear as a few remote points
    # (or disappear entirely). Navigation remains available as a saved view.
    manager.setdefault("Global Options", {})["Fixed Frame"] = contract["status_frame"]
    displays = manager.setdefault("Displays", [])
    displays[:] = [
        display for display in displays
        if display.get("Class") != "rviz_default_plugins/Image"
        and not str(display.get("Name", "")).startswith(("Perception |", "Manipulation |"))
    ]
    # Keep upstream navigation diagnostics available without letting large
    # frame axes or the dense free-path fan cover the registered map.
    for display in displays:
        if display.get("Name") in {"Vehicle", "FreePaths"}:
            display["Enabled"] = False
            display["Value"] = False
    topics = contract["topics"]

    image_displays = [
        _image("Perception | Wrist RGB [LIVE]", topics["wrist_rgb"], enabled=True),
        _image("Perception | Aligned Depth [LIVE]", topics["aligned_depth"], enabled=True),
        _image(
            "Perception | Detections + Mask [upstream required]",
            topics["detection_mask_overlay"],
            enabled=False,
        ),
        _image(
            "Perception | Target Mask [upstream required]",
            topics["target_mask"],
            enabled=False,
        ),
    ]
    local_lidar = _point_cloud(
        "Local LiDAR [LIVE, accumulated]",
        topics["raw_lidar"],
        color="255; 215; 0",
        # Raw measurements live in the physical mid360_raw frame. Keep this
        # diagnostic opt-in so it cannot visually contaminate level RegScan.
        enabled=False,
        size_m=0.012,
        # Mid-360 completes multiple sweeps in this window. Keeping one full
        # simulated second retained roughly 100 incremental clouds in RViz.
        decay_time=0.2,
    )
    local_lidar.update({"Alpha": 0.7, "Size (Pixels)": 2, "Style": "Points"})
    perception = _group(
        "Perception 3D (measured RGB-D only)",
        [
            {
                "Class": "rviz_default_plugins/CameraInfo",
                "Enabled": False,
                "Name": "Camera Model [upstream LIVE]",
                "Topic": _topic(topics["camera_info"], reliable=False),
                "Value": False,
            },
            _point_cloud(
                "Validated Target Cloud [upstream required]",
                topics["validated_target_cloud"],
                color="0; 255; 64",
                size_m=0.008,
            ),
            _point_cloud(
                "Scene Collision Cloud [upstream required]",
                topics["scene_cloud"],
                color="80; 160; 255",
                size_m=0.005,
            ),
            {
                "Alpha": 1,
                "Axes Length": 0.12,
                "Axes Radius": 0.012,
                "Class": "rviz_default_plugins/Pose",
                "Color": "0; 255; 64",
                "Enabled": False,
                "Head Length": 0.08,
                "Head Radius": 0.018,
                "Name": "Tracked Target Pose [upstream required]",
                "Shaft Length": 0.12,
                "Shaft Radius": 0.008,
                "Shape": "Axes",
                "Topic": _topic(topics["target_pose"]),
                "Value": False,
            },
        ],
    )
    manipulation = _group(
        "Manipulation Planning (no task ground truth)",
        [
            _group(
                "Key Frames",
                [
                    _axes("Platform | base_link", "base_link", length=0.24, radius=0.010),
                    _axes("Tool | piper_link8", "piper_link8", length=0.16, radius=0.007),
                    _axes(
                        "Sensor | camera_color_optical_frame",
                        "camera_color_optical_frame",
                        length=0.12,
                        radius=0.005,
                    ),
                ],
                enabled=False,
            ),
            {
                "Alpha": 0.75,
                "Class": "rviz_default_plugins/RobotModel",
                "Collision Enabled": False,
                "Description Source": "Topic",
                "Description Topic": _topic(topics["robot_description"]),
                "Enabled": False,
                "Links": {"All Links Enabled": True, "Expand Joint Details": False},
                "Mass Properties": {"Inertia": False, "Mass": False},
                "Name": "Robot Model [requires description + arm TF]",
                "TF Prefix": "",
                "Update Interval": 0,
                "Value": False,
                "Visual Enabled": True,
            },
            _marker_array(
                "6DoF Grasp Candidates [upstream required]",
                topics["candidate_grasps"],
            ),
            _marker_array(
                "Selected Pregrasp + Grasp [LIVE when planned]",
                topics["selected_grasp"],
                enabled=True,
            ),
            _path(
                "Planned TCP Trajectory [LIVE when planned]",
                topics["planned_tcp_path"],
                color="255; 210; 0",
                enabled=True,
            ),
            _path(
                "Executed TCP Trace [upstream required]",
                topics["executed_tcp_path"],
                color="0; 255; 255",
            ),
            _marker_array(
                "Perceived Collision Environment [upstream required]",
                topics["collision_environment"],
            ),
            _marker_array(
                "Octomap Occupied Cells [octomap_server required]",
                topics["octomap_markers"],
            ),
        ],
    )
    diagnostics = _group(
        "Manipulation Diagnostics",
        [
            _marker(
                "Perception Contract Status",
                topics["perception_status_marker"],
                enabled=False,
            ),
            _marker(
                "PiPER Execution Status",
                topics["piper_execution_status_marker"],
                enabled=False,
            ),
        ],
    )
    tf_display = {
        "Class": "rviz_default_plugins/TF",
        "Enabled": False,
        "Frame Timeout": 15,
        "Frames": {"All Enabled": False},
        "Marker Scale": 0.2,
        "Name": "TF Tree [diagnostic opt-in]",
        "Show Arrows": False,
        "Show Axes": False,
        "Show Names": False,
        "Tree": {},
        "Update Interval": 0,
        "Value": False,
    }
    displays[:0] = image_displays + [local_lidar]
    displays.extend([tf_display, perception, manipulation, diagnostics])

    views = manager.setdefault("Views", {})
    current = views.get("Current")
    if isinstance(current, dict):
        navigation = copy.deepcopy(current)
        navigation["Name"] = "Navigation Overview"
        manipulation_view = copy.deepcopy(current)
        manipulation_view.update(
            {
                "Distance": 3.0,
                "Focal Point": {"X": 0.45, "Y": 0.0, "Z": 0.75},
                "Name": "Mobile Manipulation",
                "Pitch": 0.65,
                "Target Frame": contract["status_frame"],
                "Yaw": 3.9,
            },
        )
        wrist_view = copy.deepcopy(current)
        wrist_view.update(
            {
                "Distance": 0.8,
                "Focal Point": {"X": 0.0, "Y": 0.0, "Z": 0.5},
                "Name": "Wrist Camera TF",
                "Pitch": 0.55,
                "Target Frame": "camera_color_optical_frame",
                "Yaw": 3.14,
            },
        )
        views["Saved"] = [navigation, manipulation_view, wrist_view]
        views["Current"] = copy.deepcopy(manipulation_view)

    geometry = result.setdefault("Window Geometry", {})
    geometry["Perception | Wrist RGB [LIVE]"] = {"collapsed": False}
    geometry["Perception | Aligned Depth [LIVE]"] = {"collapsed": False}
    # Optional mask docks are intentionally absent until their publishers are
    # live; RViz otherwise opens blank panels even when the displays are disabled.
    geometry.pop("Perception | Detections + Mask [upstream required]", None)
    geometry.pop("Perception | Target Mask [upstream required]", None)
    geometry["Displays"] = {"collapsed": False}
    geometry["Hide Left Dock"] = False
    # The upstream opaque Qt state hides the Displays tree.  Dropping it lets
    # RViz lay out the RGB/depth docks beside an immediately operable tree.
    geometry.pop("QMainWindow State", None)
    return result


def build(stock_path: Path, contract_path: Path, output_path: Path) -> None:
    stock = yaml.safe_load(stock_path.read_text(encoding="utf-8"))
    if not isinstance(stock, dict):
        raise ValueError("stock RViz configuration must be a mapping")
    result = augment_config(stock, load_contract(contract_path))
    output_path.write_text(
        yaml.safe_dump(result, sort_keys=False, width=120),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("stock", type=Path)
    parser.add_argument("contract", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    build(arguments.stock, arguments.contract, arguments.output)
    print(f"[rviz] combined navigation/manipulation config -> {arguments.output}")


if __name__ == "__main__":
    main()
