#!/usr/bin/env python3
"""Fail-closed validation for the shared simulation/real navigation profile."""

from __future__ import annotations

import argparse
import math
import re
from collections.abc import Mapping
from pathlib import Path, PurePosixPath

import yaml


MAX_SPEED_MPS = 1.0
MAX_COMMAND_RAMP_COEFFICIENT = 2.0
MAX_YAW_RATE_DEG_S = 60.0
MIN_GOAL_THRESHOLD_M = 0.02
MAX_GOAL_THRESHOLD_M = 0.20
MIN_OBSTACLE_HEIGHT_M = 0.05
MAX_OBSTACLE_HEIGHT_M = 0.20
CONFIG_COMPONENT_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")


def _required_parameters(data: Mapping, node_name: str) -> Mapping:
    block = data.get(node_name)
    if not isinstance(block, Mapping):
        raise ValueError(f"robot config requires a {node_name} mapping")
    parameters = block.get("ros__parameters")
    if not isinstance(parameters, Mapping):
        raise ValueError(
            f"robot config requires {node_name}.ros__parameters mapping"
        )
    return parameters


def _required_finite(parameters: Mapping, key: str, *, positive: bool = False) -> float:
    if key not in parameters:
        raise ValueError(f"robot config requires parameter {key}")
    try:
        value = float(parameters[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"robot config parameter {key} must be numeric") from exc
    if not math.isfinite(value) or (positive and value <= 0.0):
        qualifier = "finite and positive" if positive else "finite"
        raise ValueError(f"robot config parameter {key} must be {qualifier}")
    return value


def _validate_robot_schema(path: Path) -> None:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read robot config {path}: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ValueError("robot config must contain a YAML mapping")

    mounting = _required_parameters(data, "sensorMountingOffsets")
    for key in ("sensorOffsetX", "sensorOffsetY", "sensorOffsetZ"):
        _required_finite(mounting, key)

    local_planner = _required_parameters(data, "localPlanner")
    _required_finite(local_planner, "vehicleLength", positive=True)
    _required_finite(local_planner, "vehicleWidth", positive=True)
    _required_parameters(data, "pathFollower")


def _config_parts(config_name: str) -> tuple[str, ...]:
    relative = PurePosixPath(config_name)
    parts = relative.parts
    if (
        not config_name
        or config_name.endswith(".yaml")
        or relative.is_absolute()
        or not parts
        or any(
            part in {".", ".."} or not CONFIG_COMPONENT_PATTERN.fullmatch(part)
            for part in parts
        )
    ):
        raise ValueError(
            "robot config must be a safe relative path without a .yaml suffix"
        )
    return parts


def validate_profile(
    *,
    maximum_speed: float,
    autonomy_speed: float,
    bridge_speed: float,
    command_ramp_coefficient: float,
    yaw_rate_deg_s: float,
    goal_threshold: float,
    obstacle_height: float,
    robot_config: str,
    config_root: Path | None = None,
) -> None:
    values = (
        maximum_speed,
        autonomy_speed,
        bridge_speed,
        command_ramp_coefficient,
        yaw_rate_deg_s,
        goal_threshold,
        obstacle_height,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("navigation profile numeric values must be finite and positive")
    if maximum_speed > MAX_SPEED_MPS:
        raise ValueError(f"maximum speed exceeds {MAX_SPEED_MPS:.2f} m/s safety cap")
    if autonomy_speed > maximum_speed:
        raise ValueError("navigation cruise speed cannot exceed its hard cap")
    if not math.isclose(bridge_speed, autonomy_speed, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("NAV_SPEED must match NAV_AUTONOMY_SPEED")
    if command_ramp_coefficient > MAX_COMMAND_RAMP_COEFFICIENT:
        raise ValueError(
            "pathFollower command-ramp coefficient exceeds the 100 Hz safety cap"
        )
    if yaw_rate_deg_s > MAX_YAW_RATE_DEG_S:
        raise ValueError(
            f"yaw rate exceeds {MAX_YAW_RATE_DEG_S:.1f} deg/s safety cap"
        )
    if not MIN_GOAL_THRESHOLD_M <= goal_threshold < MAX_GOAL_THRESHOLD_M:
        raise ValueError(
            "arrival threshold must be in "
            f"[{MIN_GOAL_THRESHOLD_M:.2f}, {MAX_GOAL_THRESHOLD_M:.2f}) m"
        )
    if not MIN_OBSTACLE_HEIGHT_M <= obstacle_height <= MAX_OBSTACLE_HEIGHT_M:
        raise ValueError(
            "obstacle height threshold must be in "
            f"[{MIN_OBSTACLE_HEIGHT_M:.2f}, {MAX_OBSTACLE_HEIGHT_M:.2f}] m"
        )
    config_parts = _config_parts(robot_config)
    if config_root is not None:
        root = config_root.resolve()
        candidate = root.joinpath(
            *config_parts[:-1], f"{config_parts[-1]}.yaml"
        )
        # Do not resolve the file itself: colcon --symlink-install legitimately
        # points package-owned config entries back into src/.
        if not candidate.is_file():
            raise ValueError(f"robot config does not exist under {root}: {robot_config}")
        _validate_robot_schema(candidate)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-speed", type=float, required=True)
    parser.add_argument("--autonomy-speed", type=float, required=True)
    parser.add_argument("--bridge-speed", type=float, required=True)
    parser.add_argument("--command-ramp", type=float, required=True)
    parser.add_argument("--yaw-rate-deg-s", type=float, required=True)
    parser.add_argument("--goal-threshold", type=float, required=True)
    parser.add_argument("--obstacle-height", type=float, required=True)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--config-root", type=Path)
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        validate_profile(
            maximum_speed=args.max_speed,
            autonomy_speed=args.autonomy_speed,
            bridge_speed=args.bridge_speed,
            command_ramp_coefficient=args.command_ramp,
            yaw_rate_deg_s=args.yaw_rate_deg_s,
            goal_threshold=args.goal_threshold,
            obstacle_height=args.obstacle_height,
            robot_config=args.robot_config,
            config_root=args.config_root,
        )
    except ValueError as exc:
        raise SystemExit(f"invalid navigation profile: {exc}") from exc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
