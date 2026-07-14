"""Validated loader for configurable manipulation fixtures and objects.

This module deliberately has no Isaac or ROS dependency so scene descriptions can
be linted in CI.  Coordinates belong to the environment JSON, never to perception,
planning, or execution code.
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any


_SAFE_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
_PROXY_SHAPES = {"cuboid", "cylinder", "capsule"}
_PHYSICS_MODES = {"asset", "proxy"}


class SceneConfigError(ValueError):
    """Raised when a manipulation scene cannot be loaded safely."""


def _finite_vector(value: Any, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise SceneConfigError(f"{field} must be a JSON array of length {length}")
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise SceneConfigError(f"{field} contains a non-finite value")
    return result


def _positive_vector(value: Any, length: int, field: str) -> tuple[float, ...]:
    result = _finite_vector(value, length, field)
    if any(v <= 0.0 for v in result):
        raise SceneConfigError(f"{field} values must be positive")
    return result


def _unit_quaternion(value: Any, field: str) -> tuple[float, float, float, float]:
    quat = _finite_vector(value, 4, field)
    norm = math.sqrt(sum(v * v for v in quat))
    if norm < 1.0e-9:
        raise SceneConfigError(f"{field} must not be zero")
    if abs(norm - 1.0) > 1.0e-3:
        raise SceneConfigError(f"{field} must be normalized (norm={norm:.6f})")
    return tuple(v / norm for v in quat)


def _safe_name(value: Any, field: str) -> str:
    if not isinstance(value, str) or _SAFE_NAME.fullmatch(value) is None:
        raise SceneConfigError(f"{field} must match {_SAFE_NAME.pattern}")
    return value


def _resolve_asset(value: Any, asset_roots: dict[str, str], field: str) -> str:
    if not isinstance(value, str) or not value:
        raise SceneConfigError(f"{field} must be a non-empty string")
    result = value
    for key, root in asset_roots.items():
        result = result.replace("{" + key + "}", root.rstrip("/"))
    unresolved = re.findall(r"\{[A-Z0-9_]+\}", result)
    if unresolved:
        raise SceneConfigError(f"{field} has unresolved tokens: {unresolved}")
    return result


def _validate_proxy(raw: Any, prefix: str) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise SceneConfigError(f"{prefix} must be an object")
    shape = raw.get("shape")
    if shape not in _PROXY_SHAPES:
        raise SceneConfigError(f"{prefix}.shape must be one of {sorted(_PROXY_SHAPES)}")
    proxy: dict[str, Any] = {
        "shape": shape,
        "position": _finite_vector(raw.get("position", [0, 0, 0]), 3,
                                   f"{prefix}.position"),
        "orientation_wxyz": _unit_quaternion(
            raw.get("orientation_wxyz", [1, 0, 0, 0]),
            f"{prefix}.orientation_wxyz"),
    }
    if shape == "cuboid":
        proxy["size"] = _positive_vector(raw.get("size"), 3, f"{prefix}.size")
    else:
        radius = float(raw.get("radius", 0.0))
        height = float(raw.get("height", 0.0))
        if not math.isfinite(radius) or not math.isfinite(height) or radius <= 0 or height <= 0:
            raise SceneConfigError(f"{prefix}.radius and height must be positive")
        proxy.update(radius=radius, height=height)
    return proxy


def legacy_grasp_box_enabled(scene: dict[str, Any] | None) -> bool:
    """Keep the historical box unless a manipulation fixture disables it."""
    if scene is None:
        return True
    try:
        enabled = scene["fixtures"]["legacy_grasp_box"]["enabled"]
    except (KeyError, TypeError) as exc:
        raise SceneConfigError(
            "normalized scene is missing fixtures.legacy_grasp_box.enabled"
        ) from exc
    if not isinstance(enabled, bool):
        raise SceneConfigError("fixtures.legacy_grasp_box.enabled must be boolean")
    return enabled


def load_manip_scene(path: str | Path, asset_roots: dict[str, str]) -> dict[str, Any]:
    """Load and normalize a manipulation scene JSON document.

    The returned structure uses tuples for vectors and resolved asset URLs.  Unknown
    keys are tolerated for forward compatibility, while every physics-relevant field
    is checked explicitly.
    """
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SceneConfigError(f"cannot load {config_path}: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("version") != 1:
        raise SceneConfigError("scene config version must be 1")

    fixtures_raw = raw.get("fixtures", {})
    if not isinstance(fixtures_raw, dict):
        raise SceneConfigError("fixtures must be an object")
    legacy_box_raw = fixtures_raw.get("legacy_grasp_box", {"enabled": True})
    if not isinstance(legacy_box_raw, dict):
        raise SceneConfigError("fixtures.legacy_grasp_box must be an object")
    legacy_box_enabled = legacy_box_raw.get("enabled")
    if not isinstance(legacy_box_enabled, bool):
        raise SceneConfigError("fixtures.legacy_grasp_box.enabled must be boolean")

    shelf_raw = raw.get("shelf")
    if not isinstance(shelf_raw, dict) or not isinstance(shelf_raw.get("parts"), list):
        raise SceneConfigError("shelf.parts must be an array")
    shelf_parts: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, part_raw in enumerate(shelf_raw["parts"]):
        prefix = f"shelf.parts[{index}]"
        if not isinstance(part_raw, dict):
            raise SceneConfigError(f"{prefix} must be an object")
        name = _safe_name(part_raw.get("name"), f"{prefix}.name")
        if name in names:
            raise SceneConfigError(f"duplicate scene entity name: {name}")
        names.add(name)
        if part_raw.get("shape") != "cuboid":
            raise SceneConfigError(f"{prefix}.shape currently must be cuboid")
        if part_raw.get("static_collision") is not True:
            raise SceneConfigError(f"{prefix}.static_collision must be true")
        shelf_parts.append({
            "name": name,
            "shape": "cuboid",
            "size": _positive_vector(part_raw.get("size"), 3, f"{prefix}.size"),
            "position": _finite_vector(part_raw.get("position"), 3, f"{prefix}.position"),
            "orientation_wxyz": _unit_quaternion(
                part_raw.get("orientation_wxyz", [1, 0, 0, 0]),
                f"{prefix}.orientation_wxyz"),
            "color": _finite_vector(part_raw.get("color", [0.35, 0.38, 0.42]), 3,
                                    f"{prefix}.color"),
            "static_collision": True,
        })

    objects_raw = raw.get("objects")
    if not isinstance(objects_raw, list) or not objects_raw:
        raise SceneConfigError("objects must be a non-empty array")
    objects: list[dict[str, Any]] = []
    for index, object_raw in enumerate(objects_raw):
        prefix = f"objects[{index}]"
        if not isinstance(object_raw, dict):
            raise SceneConfigError(f"{prefix} must be an object")
        name = _safe_name(object_raw.get("name"), f"{prefix}.name")
        if name in names:
            raise SceneConfigError(f"duplicate scene entity name: {name}")
        names.add(name)
        family = _safe_name(object_raw.get("shape_family"), f"{prefix}.shape_family")
        mode = object_raw.get("physics_mode")
        if mode not in _PHYSICS_MODES:
            raise SceneConfigError(f"{prefix}.physics_mode must be one of {sorted(_PHYSICS_MODES)}")
        mass = float(object_raw.get("mass_kg", 0.0))
        if not math.isfinite(mass) or mass <= 0.0:
            raise SceneConfigError(f"{prefix}.mass_kg must be positive")
        proxies = [
            _validate_proxy(proxy, f"{prefix}.collision_proxies[{proxy_index}]")
            for proxy_index, proxy in enumerate(object_raw.get("collision_proxies", []))
        ]
        if mode == "proxy" and not proxies:
            raise SceneConfigError(f"{prefix} proxy physics requires collision_proxies")
        objects.append({
            "name": name,
            "shape_family": family,
            "asset": _resolve_asset(object_raw.get("asset"), asset_roots, f"{prefix}.asset"),
            "position": _finite_vector(object_raw.get("position"), 3, f"{prefix}.position"),
            "orientation_wxyz": _unit_quaternion(
                object_raw.get("orientation_wxyz", [1, 0, 0, 0]),
                f"{prefix}.orientation_wxyz"),
            "scale": (_positive_vector(object_raw["scale"], 3, f"{prefix}.scale")
                      if "scale" in object_raw else None),
            "mass_kg": mass,
            "physics_mode": mode,
            "collision_proxies": proxies,
        })

    return {
        "version": 1,
        "name": _safe_name(raw.get("name"), "name"),
        "source": str(config_path),
        "fixtures": {
            "legacy_grasp_box": {"enabled": legacy_box_enabled},
        },
        "shelf": {"parts": shelf_parts},
        "objects": objects,
    }
