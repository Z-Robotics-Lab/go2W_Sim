#!/usr/bin/env python3
"""Compatibility helpers for ROS ``diagnostic_msgs/DiagnosticStatus`` levels."""

from __future__ import annotations

import operator


def diagnostic_level_to_int(level: object) -> int:
    """Return a canonical uint8 value for Jazzy bytes or integer levels."""

    if isinstance(level, (bytes, bytearray, memoryview)):
        raw = bytes(level)
        if len(raw) != 1:
            raise ValueError("diagnostic level bytes must contain exactly one byte")
        return raw[0]

    try:
        value = operator.index(level)
    except TypeError as error:
        raise TypeError("diagnostic level must be an integer or one byte") from error
    if not 0 <= value <= 255:
        raise ValueError("diagnostic level must fit in uint8")
    return value
