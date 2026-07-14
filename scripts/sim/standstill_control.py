"""Pure control contracts for the Go2W simulator parking brake.

The runtime adapter owns Isaac-specific gain and target writes.  This module
only defines the state transition and joint selection rules so they can be
tested without importing Isaac Sim, ROS, or torch.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


PARK_STIFFNESS_ENV = "GO2W_STANDSTILL_WHEEL_KP"
PARK_DAMPING_ENV = "GO2W_STANDSTILL_WHEEL_DAMPING"
PARK_TRANSITION_TICKS_ENV = "GO2W_STANDSTILL_GAIN_TICKS"
STANDSTILL_LINEAR_ENTER_ENV = "GO2W_STANDSTILL_LINEAR_ENTER_MPS"
STANDSTILL_LINEAR_EXIT_ENV = "GO2W_STANDSTILL_LINEAR_EXIT_MPS"
STANDSTILL_ANGULAR_ENTER_ENV = "GO2W_STANDSTILL_ANGULAR_ENTER_RPS"
STANDSTILL_ANGULAR_EXIT_ENV = "GO2W_STANDSTILL_ANGULAR_EXIT_RPS"
STANDSTILL_ENTER_DURATION_ENV = "GO2W_STANDSTILL_ENTER_DURATION_S"
STANDSTILL_EXIT_DURATION_ENV = "GO2W_STANDSTILL_EXIT_DURATION_S"


def _finite_nonnegative(value: float, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative, got {value!r}")
    return parsed


def _finite_positive(value: float, name: str) -> float:
    parsed = _finite_nonnegative(value, name)
    if parsed <= 0.0:
        raise ValueError(f"{name} must be positive")
    return parsed


def _positive_integer(value: int, name: str) -> int:
    if isinstance(value, bool) or int(value) != value or int(value) < 1:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


@dataclass(frozen=True)
class StandstillGateConfig:
    """SI-unit thresholds for distinguishing parking from motion intent.

    Linear and angular velocity are deliberately gated independently.  Adding
    metres per second to radians per second in one Euclidean norm has no
    physical meaning and can hide a valid low-rate manipulation yaw command.
    """

    linear_enter_mps: float = 0.005
    linear_exit_mps: float = 0.015
    angular_enter_rps: float = 0.005
    angular_exit_rps: float = 0.015
    enter_duration_s: float = 0.50
    exit_duration_s: float = 0.04

    def __post_init__(self) -> None:
        for field_name in (
            "linear_enter_mps",
            "linear_exit_mps",
            "angular_enter_rps",
            "angular_exit_rps",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite_nonnegative(getattr(self, field_name), field_name),
            )
        if self.linear_enter_mps >= self.linear_exit_mps:
            raise ValueError("linear standstill enter threshold must be below exit")
        if self.angular_enter_rps >= self.angular_exit_rps:
            raise ValueError("angular standstill enter threshold must be below exit")
        object.__setattr__(
            self,
            "enter_duration_s",
            _finite_positive(self.enter_duration_s, "enter_duration_s"),
        )
        object.__setattr__(
            self,
            "exit_duration_s",
            _finite_positive(self.exit_duration_s, "exit_duration_s"),
        )

    @classmethod
    def from_environ(cls, environ: Mapping[str, str]) -> "StandstillGateConfig":
        """Load deployment overrides while retaining explicit physical units."""
        return cls(
            linear_enter_mps=float(
                environ.get(STANDSTILL_LINEAR_ENTER_ENV, "0.005")
            ),
            linear_exit_mps=float(
                environ.get(STANDSTILL_LINEAR_EXIT_ENV, "0.015")
            ),
            angular_enter_rps=float(
                environ.get(STANDSTILL_ANGULAR_ENTER_ENV, "0.005")
            ),
            angular_exit_rps=float(
                environ.get(STANDSTILL_ANGULAR_EXIT_ENV, "0.015")
            ),
            enter_duration_s=float(
                environ.get(STANDSTILL_ENTER_DURATION_ENV, "0.50")
            ),
            exit_duration_s=float(
                environ.get(STANDSTILL_EXIT_DURATION_ENV, "0.04")
            ),
        )


@dataclass(frozen=True)
class StandstillDecision:
    """One command-gate transition evaluated at the policy control rate."""

    parked: bool
    engage: bool
    release: bool
    linear_speed_mps: float
    angular_speed_rps: float
    region: str


class StandstillCommandGate:
    """Debounced two-channel hysteresis for the simulator parking brake."""

    def __init__(self, config: StandstillGateConfig):
        self.config = config
        self._parked = False
        self._enter_elapsed_s = 0.0
        self._exit_elapsed_s = 0.0

    @property
    def parked(self) -> bool:
        return self._parked

    @property
    def enter_elapsed_s(self) -> float:
        return self._enter_elapsed_s

    @property
    def exit_elapsed_s(self) -> float:
        return self._exit_elapsed_s

    def reset(self) -> None:
        """Return to drive mode without retaining pre-reset debounce history."""
        self._parked = False
        self._enter_elapsed_s = 0.0
        self._exit_elapsed_s = 0.0

    def force_park(self) -> StandstillDecision:
        """Enter parking immediately for an invalid or stale command source."""
        engage = not self._parked
        self._parked = True
        self._enter_elapsed_s = self.config.enter_duration_s
        self._exit_elapsed_s = 0.0
        return StandstillDecision(
            parked=True,
            engage=engage,
            release=False,
            linear_speed_mps=0.0,
            angular_speed_rps=0.0,
            region="forced",
        )

    def update(
        self,
        vx_mps: float,
        vy_mps: float,
        wz_rps: float,
        *,
        dt_s: float,
    ) -> StandstillDecision:
        """Classify one SI ``cmd_vel`` sample and advance the hysteresis state."""
        values = tuple(float(value) for value in (vx_mps, vy_mps, wz_rps))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("standstill gate requires finite cmd_vel components")
        elapsed = _finite_positive(dt_s, "standstill gate dt_s")
        vx, vy, wz = values
        linear_speed = math.hypot(vx, vy)
        angular_speed = abs(wz)
        inside_enter = (
            linear_speed <= self.config.linear_enter_mps
            and angular_speed <= self.config.angular_enter_rps
        )
        outside_exit = (
            linear_speed >= self.config.linear_exit_mps
            or angular_speed >= self.config.angular_exit_rps
        )
        engage = False
        release = False
        if inside_enter:
            region = "enter"
            self._enter_elapsed_s = min(
                self.config.enter_duration_s, self._enter_elapsed_s + elapsed
            )
            self._exit_elapsed_s = 0.0
            if (
                not self._parked
                and self._enter_elapsed_s >= self.config.enter_duration_s
            ):
                self._parked = True
                engage = True
        elif outside_exit:
            region = "exit"
            self._exit_elapsed_s = min(
                self.config.exit_duration_s, self._exit_elapsed_s + elapsed
            )
            self._enter_elapsed_s = 0.0
            if (
                self._parked
                and self._exit_elapsed_s >= self.config.exit_duration_s
            ):
                self._parked = False
                release = True
        else:
            region = "hysteresis"
            self._enter_elapsed_s = 0.0
            self._exit_elapsed_s = 0.0
        return StandstillDecision(
            parked=self._parked,
            engage=engage,
            release=release,
            linear_speed_mps=linear_speed,
            angular_speed_rps=angular_speed,
            region=region,
        )


@dataclass(frozen=True)
class ParkingBrakeConfig:
    """Drive and parked gains for an encoder-based wheel position hold."""

    drive_stiffness: float
    drive_damping: float
    park_stiffness: float
    park_damping: float
    transition_ticks: int

    def __post_init__(self) -> None:
        for field_name in (
            "drive_stiffness",
            "drive_damping",
            "park_stiffness",
            "park_damping",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite_nonnegative(getattr(self, field_name), field_name),
            )
        if self.park_stiffness <= 0.0:
            raise ValueError("park_stiffness must be positive for position hold")
        object.__setattr__(
            self,
            "transition_ticks",
            _positive_integer(self.transition_ticks, "transition_ticks"),
        )

    @classmethod
    def from_environ(
        cls,
        environ: Mapping[str, str],
        *,
        drive_stiffness: float,
        drive_damping: float,
    ) -> "ParkingBrakeConfig":
        """Build a validated config while keeping deployment gains configurable."""
        try:
            transition_ticks = int(environ.get(PARK_TRANSITION_TICKS_ENV, "10"))
        except ValueError as exc:
            raise ValueError(
                f"{PARK_TRANSITION_TICKS_ENV} must be an integer"
            ) from exc
        return cls(
            drive_stiffness=drive_stiffness,
            drive_damping=drive_damping,
            park_stiffness=float(environ.get(PARK_STIFFNESS_ENV, "20.0")),
            park_damping=float(environ.get(PARK_DAMPING_ENV, "8.0")),
            transition_ticks=transition_ticks,
        )


@dataclass(frozen=True)
class ParkingBrakeCommand:
    """One control-tick command emitted by :class:`WheelParkingBrake`."""

    mode: str
    blend: float
    stiffness: float
    damping: float
    position_target: tuple[float, ...] | None


class WheelParkingBrake:
    """Latch wheel encoders and blend between velocity drive and parking hold.

    During release the position target follows the measured encoder positions
    while the position gain fades.  That prevents the remaining gain from
    pulling against the velocity command ramp.
    """

    def __init__(self, config: ParkingBrakeConfig):
        self.config = config
        self._desired_parked = False
        self._blend = 0.0
        self._anchor: tuple[float, ...] | None = None

    @property
    def desired_parked(self) -> bool:
        return self._desired_parked

    @property
    def blend(self) -> float:
        return self._blend

    @property
    def anchor(self) -> tuple[float, ...] | None:
        return self._anchor

    @staticmethod
    def _positions(values: Sequence[float]) -> tuple[float, ...]:
        positions = tuple(float(value) for value in values)
        if not positions:
            raise ValueError("wheel positions must not be empty")
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("wheel positions must all be finite")
        return positions

    def engage(self, wheel_positions: Sequence[float]) -> None:
        """Request parking and latch the encoders once for this engagement."""
        positions = self._positions(wheel_positions)
        if not self._desired_parked:
            self._anchor = positions
        elif self._anchor is not None and len(positions) != len(self._anchor):
            raise ValueError("wheel position count changed while parking")
        self._desired_parked = True

    def release(self) -> None:
        """Request velocity mode; gain removal occurs over transition ticks."""
        self._desired_parked = False

    def reset(self) -> None:
        """Return immediately to configured velocity-drive gains."""
        self._desired_parked = False
        self._blend = 0.0
        self._anchor = None

    def step(self, wheel_positions: Sequence[float]) -> ParkingBrakeCommand:
        """Advance one policy tick and return gains plus an optional hold target."""
        current = self._positions(wheel_positions)
        increment = 1.0 / self.config.transition_ticks

        if self._desired_parked:
            if self._anchor is None:
                raise RuntimeError("parking requested without a wheel encoder anchor")
            if len(current) != len(self._anchor):
                raise ValueError("wheel position count does not match parking anchor")
            self._blend = min(1.0, self._blend + increment)
            target = self._anchor
            mode = "parked" if self._blend >= 1.0 else "engaging"
        else:
            self._blend = max(0.0, self._blend - increment)
            if self._blend > 0.0:
                # Recenter during release so residual Kp cannot fight locomotion.
                target = current
                mode = "releasing"
            else:
                self._anchor = None
                target = None
                mode = "drive"

        stiffness = (
            self.config.drive_stiffness
            + self._blend * (self.config.park_stiffness - self.config.drive_stiffness)
        )
        damping = (
            self.config.drive_damping
            + self._blend * (self.config.park_damping - self.config.drive_damping)
        )
        return ParkingBrakeCommand(
            mode=mode,
            blend=self._blend,
            stiffness=stiffness,
            damping=damping,
            position_target=target,
        )


def select_joint_complement(
    joint_names: Sequence[str],
    excluded_indices: Sequence[int],
) -> tuple[tuple[int, ...], tuple[str, ...]]:
    """Select movable joints not owned by another controller.

    Isaac's ``robot.joint_names`` contains articulation DOFs only, so taking
    the complement of the discovered PiPER indices yields every platform DOF
    without maintaining a second hard-coded robot joint list.
    """
    names = tuple(str(name) for name in joint_names)
    if not names or any(not name for name in names):
        raise ValueError("joint_names must contain non-empty names")
    if len(set(names)) != len(names):
        raise ValueError("joint_names must be unique")
    excluded = {int(index) for index in excluded_indices}
    if any(index < 0 or index >= len(names) for index in excluded):
        raise ValueError("excluded joint index is out of range")
    selected = tuple(index for index in range(len(names)) if index not in excluded)
    if not selected:
        raise ValueError("platform joint selection is empty")
    return selected, tuple(names[index] for index in selected)
