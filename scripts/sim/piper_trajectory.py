"""Pure PiPER trajectory, gripper-command, and execution-status contracts."""
from __future__ import annotations

from dataclasses import dataclass
import bisect
import math
from typing import Iterable, Mapping, Sequence


ARM_JOINT_NAMES = tuple(f"piper_joint{i}" for i in range(1, 7))
TRAJECTORY_SEGMENTS = frozenset((
    "transit",
    "approach",
    "lift",
    "carry",
    "place_transit",
    "place_approach",
    "place_retreat",
))
_TRAJECTORY_CONTRACT_SEPARATOR = "|contract="
_CONTRACTED_SEGMENTS = frozenset(("place_approach", "place_retreat"))


class TrajectoryValidationError(ValueError):
    """A command was rejected before it could own the arm."""


class GripperValidationError(ValueError):
    """A command was rejected before it could own the gripper."""


@dataclass(frozen=True)
class TrajectorySample:
    positions: tuple[float, ...]
    done: bool


@dataclass(frozen=True)
class EndpointConvergenceConfig:
    """Measured joint-state gate used before reporting trajectory success.

    The executor adapter supplies encoder positions, velocities and their
    observation time.  These bounds therefore apply unchanged to simulation
    and hardware and do not depend on task or object ground truth.
    """

    position_tolerance_rad: float = 0.04
    velocity_tolerance_rad_s: float = 0.10
    dwell_s: float = 0.20
    timeout_s: float = 2.0
    feedback_max_age_s: float = 0.20

    def __post_init__(self) -> None:
        for name, value in (
            ("position_tolerance_rad", self.position_tolerance_rad),
            ("velocity_tolerance_rad_s", self.velocity_tolerance_rad_s),
            ("dwell_s", self.dwell_s),
            ("timeout_s", self.timeout_s),
            ("feedback_max_age_s", self.feedback_max_age_s),
        ):
            if isinstance(value, bool):
                raise ValueError(f"{name} must be a real-valued SI quantity")
            parsed = float(value)
            if not math.isfinite(parsed) or parsed <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, parsed)
        if self.dwell_s > self.timeout_s:
            raise ValueError("dwell_s must not exceed timeout_s")

    @classmethod
    def from_environ(cls, environ: Mapping[str, str]) -> "EndpointConvergenceConfig":
        """Load optional SI-unit overrides without selecting a sim-only policy."""

        defaults = cls()
        return cls(
            position_tolerance_rad=float(environ.get(
                "GO2W_PIPER_ENDPOINT_POSITION_TOLERANCE_RAD",
                defaults.position_tolerance_rad,
            )),
            velocity_tolerance_rad_s=float(environ.get(
                "GO2W_PIPER_ENDPOINT_VELOCITY_TOLERANCE_RAD_S",
                defaults.velocity_tolerance_rad_s,
            )),
            dwell_s=float(environ.get(
                "GO2W_PIPER_ENDPOINT_DWELL_S", defaults.dwell_s)),
            timeout_s=float(environ.get(
                "GO2W_PIPER_ENDPOINT_TIMEOUT_S", defaults.timeout_s)),
            feedback_max_age_s=float(environ.get(
                "GO2W_PIPER_FEEDBACK_MAX_AGE_S", defaults.feedback_max_age_s)),
        )


@dataclass(frozen=True)
class GripperCommand:
    """One accepted aperture command with executor-assigned identity."""

    command_id: int
    aperture: float
    received_at: float


def _clean_status_token(value: object) -> str:
    """Keep the semicolon status protocol parseable and single-line."""

    text = str(value).strip().replace(";", ":").replace("=", ":")
    text = "_".join(text.split())
    return text or "unspecified"


def _identity_token(value: object, label: str) -> str:
    """Return one bounded token that cannot corrupt the status protocol."""
    if not isinstance(value, str):
        raise TrajectoryValidationError(f"{label} must be a string")
    token = value.strip()
    if (
        not token
        or token != value
        or len(token) > 128
        or any(ord(char) < 0x21 or ord(char) > 0x7e for char in token)
        or any(char in token for char in ";|=")
    ):
        raise TrajectoryValidationError(f"{label} is invalid")
    return token


def parse_trajectory_segment(value: object) -> tuple[str, str]:
    """Parse a segment and the mandatory place-transaction identity."""
    if not isinstance(value, str):
        raise TrajectoryValidationError("segment must be a string")
    if value.count(_TRAJECTORY_CONTRACT_SEPARATOR) > 1:
        raise TrajectoryValidationError("trajectory contract delimiter is repeated")
    if _TRAJECTORY_CONTRACT_SEPARATOR in value:
        segment, contract_id = value.split(_TRAJECTORY_CONTRACT_SEPARATOR, 1)
        contract_id = _identity_token(contract_id, "trajectory contract_id")
    else:
        segment, contract_id = value, "none"
    segment = segment.strip()
    if segment not in TRAJECTORY_SEGMENTS:
        raise TrajectoryValidationError(
            f"segment must be one of {sorted(TRAJECTORY_SEGMENTS)}, got {segment!r}",
        )
    if segment in _CONTRACTED_SEGMENTS and contract_id == "none":
        raise TrajectoryValidationError(
            f"{segment} requires a trajectory contract_id",
        )
    if segment not in _CONTRACTED_SEGMENTS and contract_id != "none":
        raise TrajectoryValidationError(
            f"{segment} cannot carry a place trajectory contract_id",
        )
    return segment, contract_id


def _format_optional(value: float | None, *, digits: int = 6) -> str:
    return "none" if value is None else f"{value:.{digits}f}"


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
        endpoint_convergence: EndpointConvergenceConfig | None = None,
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
        self.endpoint_convergence = endpoint_convergence or EndpointConvergenceConfig()
        self.status = "idle"
        self.owner = "none"
        self.phase = "idle"
        self.command_id = 0
        self.segment = "none"
        self.contract_id = "none"
        self.received_at: float | None = None
        self._times: tuple[float, ...] = ()
        self._positions: tuple[tuple[float, ...], ...] = ()
        self._started_at = 0.0
        self._hold: tuple[float, ...] | None = None
        self._last_sample_at: float | None = None
        self._last_feedback_at: float | None = None
        self._last_feedback_positions: tuple[float, ...] | None = None
        self._within_tolerance_since: float | None = None
        self.endpoint_position_error_rad: float | None = None
        self.endpoint_velocity_rad_s: float | None = None
        self.feedback_age_s: float | None = None
        self.settle_dwell_s = 0.0

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
        *,
        segment: str,
    ) -> int:
        segment_name, contract_id = parse_trajectory_segment(segment)
        received_at = float(sim_time)
        if not math.isfinite(received_at) or received_at < 0.0:
            raise TrajectoryValidationError("sim_time must be finite and non-negative")
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
        self._started_at = received_at
        self._hold = current
        self._last_sample_at = received_at
        self._last_feedback_at = None
        self._last_feedback_positions = current
        self._within_tolerance_since = None
        self.endpoint_position_error_rad = None
        self.endpoint_velocity_rad_s = None
        self.feedback_age_s = None
        self.settle_dwell_s = 0.0
        self.command_id += 1
        self.segment = segment_name
        self.contract_id = contract_id
        self.received_at = received_at
        self.owner = "trajectory"
        self.phase = "tracking"
        self.status = "active"
        return self.command_id

    def _fail_closed(self, reason: str) -> TrajectorySample:
        """Reject completion and stop at the last trustworthy measured state."""

        if self._last_feedback_positions is not None:
            self._hold = self._last_feedback_positions
        self.status = f"rejected:{_clean_status_token(reason)}"
        self.phase = "failed"
        return TrajectorySample(self._hold or self._positions[-1], True)

    def _measured_feedback(
        self,
        sim_time: float,
        measured_positions: Sequence[float] | None,
        measured_velocities: Sequence[float] | None,
        feedback_at: float | None,
    ) -> tuple[tuple[float, ...], tuple[float, ...], float] | TrajectorySample:
        """Validate one encoder sample and retain only monotonic fresh feedback."""

        if measured_positions is None or measured_velocities is None or feedback_at is None:
            return self._fail_closed("feedback_missing")
        try:
            positions = self._validate_finite(measured_positions, "measured_positions")
            velocities = self._validate_finite(measured_velocities, "measured_velocities")
            observed_at = float(feedback_at)
        except (TypeError, ValueError, TrajectoryValidationError):
            return self._fail_closed("feedback_invalid")
        if len(positions) != len(self.joint_names) or len(velocities) != len(self.joint_names):
            return self._fail_closed("feedback_length_mismatch")
        if not math.isfinite(observed_at) or observed_at < 0.0:
            return self._fail_closed("feedback_time_invalid")
        if observed_at > sim_time + 1.0e-9:
            return self._fail_closed("feedback_from_future")
        age = sim_time - observed_at
        self.feedback_age_s = age
        if age > self.endpoint_convergence.feedback_max_age_s:
            return self._fail_closed("feedback_stale")
        if (
            self._last_feedback_at is not None
            and observed_at < self._last_feedback_at - 1.0e-9
        ):
            return self._fail_closed("feedback_time_regressed")
        if (
            self._last_feedback_at is not None
            and observed_at > self._last_feedback_at
            and observed_at - self._last_feedback_at
            > self.endpoint_convergence.feedback_max_age_s
        ):
            self._within_tolerance_since = None
            self.settle_dwell_s = 0.0
        if self._last_feedback_at is None or observed_at > self._last_feedback_at:
            self._last_feedback_at = observed_at
            self._last_feedback_positions = positions
        return positions, velocities, observed_at

    def sample(
        self,
        sim_time: float,
        measured_positions: Sequence[float] | None = None,
        measured_velocities: Sequence[float] | None = None,
        *,
        feedback_at: float | None = None,
    ) -> TrajectorySample:
        if not self._positions:
            if self._hold is None:
                raise RuntimeError("no trajectory or hold position is available")
            return TrajectorySample(self._hold, True)
        if not self.active:
            return TrajectorySample(self._hold or self._positions[-1], True)
        try:
            now = float(sim_time)
        except (TypeError, ValueError):
            return self._fail_closed("execution_time_invalid")
        if not math.isfinite(now) or now < 0.0:
            return self._fail_closed("execution_time_invalid")
        if self._last_sample_at is not None and now < self._last_sample_at - 1.0e-9:
            return self._fail_closed("execution_time_regressed")
        self._last_sample_at = now
        feedback = self._measured_feedback(
            now, measured_positions, measured_velocities, feedback_at)
        if isinstance(feedback, TrajectorySample):
            return feedback
        positions, velocities, observed_at = feedback

        elapsed = max(0.0, now - self._started_at)
        if elapsed >= self._times[-1]:
            self._hold = self._positions[-1]
            self.phase = "settling"
            endpoint_at = self._started_at + self._times[-1]
            self.endpoint_position_error_rad = max(
                abs(measured - target)
                for measured, target in zip(positions, self._positions[-1])
            )
            self.endpoint_velocity_rad_s = max(abs(value) for value in velocities)
            converged = (
                self.endpoint_position_error_rad
                <= self.endpoint_convergence.position_tolerance_rad
                and self.endpoint_velocity_rad_s
                <= self.endpoint_convergence.velocity_tolerance_rad_s
            )
            # Dwell advances on encoder observation time, not executor polling
            # time. Reusing one stamped sample can therefore never prove settle.
            if converged and observed_at + 1.0e-9 >= endpoint_at:
                if self._within_tolerance_since is None:
                    self._within_tolerance_since = observed_at
                self.settle_dwell_s = max(
                    0.0, observed_at - self._within_tolerance_since)
                if self.settle_dwell_s + 1.0e-9 >= self.endpoint_convergence.dwell_s:
                    self.status = "succeeded"
                    self.phase = "converged"
                    return TrajectorySample(self._hold, True)
            else:
                self._within_tolerance_since = None
                self.settle_dwell_s = 0.0
            if now - endpoint_at + 1.0e-9 >= self.endpoint_convergence.timeout_s:
                return self._fail_closed("endpoint_not_converged")
            return TrajectorySample(self._hold, False)
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
        self.status = _clean_status_token(reason)
        self.phase = "terminal"
        self._within_tolerance_since = None
        self.settle_dwell_s = 0.0

    def status_fields(self, physical_owner: str) -> tuple[str, ...]:
        """Return the compatible status prefix plus strict command identity."""

        owner = str(physical_owner).strip()
        if owner in ("trajectory", "trajectory_hold"):
            owner = "trajectory"
        else:
            owner = _clean_status_token(owner or self.owner)
        return (
            self.status,
            f"owner={owner}",
            f"command_id={self.command_id}",
            f"segment={self.segment}",
            f"trajectory_contract_id={self.contract_id}",
            f"trajectory_received_at={_format_optional(self.received_at)}",
            f"trajectory_phase={self.phase}",
            f"endpoint_position_error={_format_optional(self.endpoint_position_error_rad)}",
            f"endpoint_velocity={_format_optional(self.endpoint_velocity_rad_s)}",
            f"feedback_age={_format_optional(self.feedback_age_s)}",
            f"settle_dwell={self.settle_dwell_s:.6f}",
        )


class GripperCommandBuffer:
    """Validate apertures and retain the last accepted command identity.

    Rejections are observable status events, but never increment or replace the
    ID, aperture, or receive time of the last accepted command.
    """

    def __init__(self, minimum_aperture: float, maximum_aperture: float) -> None:
        minimum = float(minimum_aperture)
        maximum = float(maximum_aperture)
        if (
            not math.isfinite(minimum)
            or not math.isfinite(maximum)
            or minimum < 0.0
            or maximum <= minimum
        ):
            raise ValueError("gripper aperture limits must be finite and increasing")
        self.minimum_aperture = minimum
        self.maximum_aperture = maximum
        self.command_id = 0
        self.aperture: float | None = None
        self.received_at: float | None = None
        self.event_received_at: float | None = None
        self.status = "idle"

    def submit(self, aperture: float, received_at: float) -> GripperCommand:
        """Accept one command transactionally and allocate its next ID."""

        value = float(aperture)
        stamp = float(received_at)
        if not math.isfinite(value):
            raise GripperValidationError("aperture is non-finite")
        if not self.minimum_aperture <= value <= self.maximum_aperture:
            raise GripperValidationError(
                f"aperture {value:.6f} outside "
                f"[{self.minimum_aperture:.6f}, {self.maximum_aperture:.6f}]",
            )
        if not math.isfinite(stamp) or stamp < 0.0:
            raise GripperValidationError("received_at must be finite and non-negative")
        self.command_id += 1
        self.aperture = value
        self.received_at = stamp
        self.event_received_at = stamp
        self.status = f"accepted:{value:.4f}"
        return GripperCommand(self.command_id, value, stamp)

    def reject(self, reason: object, received_at: float) -> None:
        """Report a rejection without changing the last accepted identity."""

        stamp = float(received_at)
        if not math.isfinite(stamp) or stamp < 0.0:
            raise GripperValidationError("received_at must be finite and non-negative")
        self.event_received_at = stamp
        self.status = f"rejected:{_clean_status_token(reason)}"

    def note_external_owner(self, state: object, received_at: float) -> None:
        """Expose named-pose ownership without allocating a gripper command ID."""

        stamp = float(received_at)
        if not math.isfinite(stamp) or stamp < 0.0:
            raise GripperValidationError("received_at must be finite and non-negative")
        self.event_received_at = stamp
        self.status = _clean_status_token(state)

    def status_fields(self) -> tuple[str, ...]:
        aperture = "none" if self.aperture is None else f"{self.aperture:.4f}"
        return (
            f"gripper={self.status}",
            f"gripper_command_id={self.command_id}",
            f"gripper_command_aperture={aperture}",
            f"gripper_received_at={_format_optional(self.received_at)}",
            f"gripper_event_received_at={_format_optional(self.event_received_at)}",
        )


def format_execution_status(
    trajectory: JointTrajectoryBuffer,
    gripper: GripperCommandBuffer,
    *,
    executor_epoch: str,
    physical_owner: str,
    measured_aperture: float,
) -> str:
    """Build the backward-compatible String payload with strict identities."""

    measured = float(measured_aperture)
    if not math.isfinite(measured) or measured < 0.0:
        raise ValueError("measured aperture must be finite and non-negative")
    epoch = _identity_token(executor_epoch, "executor_epoch")
    return ";".join((
        *trajectory.status_fields(physical_owner),
        f"executor_epoch={epoch}",
        *gripper.status_fields(),
        f"aperture={measured:.4f}",
    ))
