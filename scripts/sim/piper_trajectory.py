"""Pure validation and sim-time interpolation for PiPER joint trajectories."""
from __future__ import annotations

from dataclasses import dataclass
import bisect
import math
from typing import Iterable, Sequence


ARM_JOINT_NAMES = tuple(f"piper_joint{i}" for i in range(1, 7))


class TrajectoryValidationError(ValueError):
    """A command was rejected before it could own the arm."""


@dataclass(frozen=True)
class TrajectorySample:
    positions: tuple[float, ...]
    done: bool


class JointTrajectoryBuffer:
    """Validated trajectory with deterministic interpolation against simulation time.

    The class is ROS-independent.  The bridge converts ``time_from_start`` to seconds
    and supplies plain sequences, allowing safety checks to run in normal unit tests.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        position_limits: dict[str, tuple[float, float]],
        velocity_limits: dict[str, float],
        *,
        start_tolerance: float = 0.12,
        velocity_margin: float = 1.05,
        max_duration: float = 120.0,
    ) -> None:
        self.joint_names = tuple(joint_names)
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("canonical joint names must be unique")
        if set(position_limits) != set(self.joint_names):
            raise ValueError("position_limits must cover exactly the canonical joints")
        if set(velocity_limits) != set(self.joint_names):
            raise ValueError("velocity_limits must cover exactly the canonical joints")
        self.position_limits = dict(position_limits)
        self.velocity_limits = dict(velocity_limits)
        self.start_tolerance = float(start_tolerance)
        self.velocity_margin = float(velocity_margin)
        self.max_duration = float(max_duration)
        self.status = "idle"
        self._times: tuple[float, ...] = ()
        self._positions: tuple[tuple[float, ...], ...] = ()
        self._started_at = 0.0
        self._hold: tuple[float, ...] | None = None

    @property
    def active(self) -> bool:
        return self.status == "active"

    @property
    def hold_positions(self) -> tuple[float, ...] | None:
        return self._hold

    def _validate_finite(self, values: Iterable[float], field: str) -> tuple[float, ...]:
        result = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in result):
            raise TrajectoryValidationError(f"{field} contains a non-finite value")
        return result

    def submit(
        self,
        command_joint_names: Sequence[str],
        point_positions: Sequence[Sequence[float]],
        time_from_start: Sequence[float],
        current_positions: Sequence[float],
        sim_time: float,
    ) -> None:
        names = tuple(command_joint_names)
        if len(names) != len(set(names)):
            raise TrajectoryValidationError("joint_names contains duplicates")
        if set(names) != set(self.joint_names):
            missing = sorted(set(self.joint_names) - set(names))
            extra = sorted(set(names) - set(self.joint_names))
            raise TrajectoryValidationError(f"joint_names mismatch missing={missing} extra={extra}")
        if not point_positions or len(point_positions) != len(time_from_start):
            raise TrajectoryValidationError("trajectory needs equally sized non-empty points and times")
        current = self._validate_finite(current_positions, "current_positions")
        if len(current) != len(self.joint_names):
            raise TrajectoryValidationError("current_positions length mismatch")
        times = self._validate_finite(time_from_start, "time_from_start")
        if times[0] < 0.0 or any(b <= a for a, b in zip(times, times[1:])):
            raise TrajectoryValidationError("time_from_start must be non-negative and strictly increasing")
        if times[-1] <= 0.0 or times[-1] > self.max_duration:
            raise TrajectoryValidationError(
                f"duration must be in (0, {self.max_duration:.3f}] seconds")

        command_index = {name: index for index, name in enumerate(names)}
        ordered: list[tuple[float, ...]] = []
        for point_index, point in enumerate(point_positions):
            values = self._validate_finite(point, f"points[{point_index}].positions")
            if len(values) != len(names):
                raise TrajectoryValidationError(f"points[{point_index}] positions length mismatch")
            q = tuple(values[command_index[name]] for name in self.joint_names)
            for joint_name, value in zip(self.joint_names, q):
                lower, upper = self.position_limits[joint_name]
                if value < lower or value > upper:
                    raise TrajectoryValidationError(
                        f"{joint_name}={value:.6f} outside [{lower:.6f}, {upper:.6f}]")
            ordered.append(q)

        if times[0] == 0.0:
            start_error = max(abs(a - b) for a, b in zip(current, ordered[0]))
            if start_error > self.start_tolerance:
                raise TrajectoryValidationError(
                    f"first point is {start_error:.4f} rad from measured state "
                    f"(limit {self.start_tolerance:.4f})")
            check_times = times
            check_positions = tuple(ordered)
        else:
            check_times = (0.0,) + times
            check_positions = (current,) + tuple(ordered)

        for segment_index, (t0, t1) in enumerate(zip(check_times, check_times[1:])):
            duration = t1 - t0
            for joint_index, joint_name in enumerate(self.joint_names):
                speed = abs(
                    check_positions[segment_index + 1][joint_index]
                    - check_positions[segment_index][joint_index]
                ) / duration
                allowed = self.velocity_limits[joint_name] * self.velocity_margin
                if speed > allowed:
                    raise TrajectoryValidationError(
                        f"segment {segment_index} {joint_name} speed {speed:.4f} "
                        f"exceeds {allowed:.4f} rad/s")

        self._times = tuple(check_times)
        self._positions = tuple(check_positions)
        self._started_at = float(sim_time)
        self._hold = current
        self.status = "active"

    def sample(self, sim_time: float) -> TrajectorySample:
        if not self._positions:
            if self._hold is None:
                raise RuntimeError("no trajectory or hold position is available")
            return TrajectorySample(self._hold, True)
        if not self.active:
            return TrajectorySample(self._hold or self._positions[-1], True)
        elapsed = max(0.0, float(sim_time) - self._started_at)
        if elapsed >= self._times[-1]:
            self._hold = self._positions[-1]
            self.status = "succeeded"
            return TrajectorySample(self._hold, True)
        right = bisect.bisect_right(self._times, elapsed)
        left = max(0, right - 1)
        if right >= len(self._times):
            right = len(self._times) - 1
            left = right
        if left == right:
            positions = self._positions[left]
        else:
            t0, t1 = self._times[left], self._times[right]
            alpha = (elapsed - t0) / (t1 - t0)
            positions = tuple(
                a + alpha * (b - a)
                for a, b in zip(self._positions[left], self._positions[right])
            )
        self._hold = positions
        return TrajectorySample(positions, False)

    def cancel(self, current_positions: Sequence[float], reason: str = "canceled") -> None:
        current = self._validate_finite(current_positions, "current_positions")
        if len(current) != len(self.joint_names):
            raise TrajectoryValidationError("current_positions length mismatch")
        self._times = ()
        self._positions = ()
        self._hold = current
        self.status = reason
