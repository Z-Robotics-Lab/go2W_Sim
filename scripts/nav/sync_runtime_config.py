#!/usr/bin/env python3
"""Synchronize ARISE runtime configuration after validating the sim contract."""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path


CALIBRATION_RELATIVE = Path(
    "src/slam/arise_slam_mid360/config/livox/livox_mid360_calibration.yaml"
)
CALIBRATION_INSTALL_RELATIVE = Path(
    "install/arise_slam_mid360/share/arise_slam_mid360/config/livox/"
    "livox_mid360_calibration.yaml"
)


def read_rotation_offset(path: Path) -> tuple[float, float, float]:
    """Read the OpenCV-YAML IMU-to-lidar offset without requiring OpenCV."""
    text = path.read_text()
    match = re.search(
        r"imu_laser_rotation_offset:.*?\n(?:.*\n){0,4}\s*data:\s*\[([^]]+)\]",
        text,
        flags=re.DOTALL,
    )
    if match is None:
        raise RuntimeError(f"imu_laser_rotation_offset missing from {path}")
    values = tuple(float(value.strip()) for value in match.group(1).split(","))
    if len(values) != 3:
        raise RuntimeError(f"expected three rotation values in {path}, got {values}")
    return values


def sync_calibration(nav_root: Path) -> Path:
    """Copy the validated source calibration to the package's runtime share."""
    source = nav_root / CALIBRATION_RELATIVE
    destination = nav_root / CALIBRATION_INSTALL_RELATIVE
    offset = read_rotation_offset(source)
    if offset != (0.0, 0.0, 0.0):
        raise RuntimeError(
            "Isaac lidar and IMU are co-framed; refusing runtime offset "
            f"{offset} from {source}"
        )
    if not destination.parent.is_dir():
        raise RuntimeError(
            f"ARISE install tree missing at {destination.parent}; build the package first"
        )
    if not destination.exists() or source.resolve() != destination.resolve():
        shutil.copyfile(source, destination)
    if source.read_bytes() != destination.read_bytes():
        raise RuntimeError(f"runtime calibration did not match source: {destination}")
    return destination


def main() -> None:
    nav_root = Path(sys.argv[1]).resolve()
    destination = sync_calibration(nav_root)
    print(f"[sync] ARISE runtime calibration -> {destination}")


if __name__ == "__main__":
    main()
