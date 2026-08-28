from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


MOTION_CONFIRMATION_TOKEN = "RM65_CANFD_HIGH_FOLLOW_CONFIRMED"
NOT_ATTEMPTED = -999


@dataclass(frozen=True)
class CanfdSafetyConfig:
    control_dt_sec: float = 0.005
    max_command_joint_speed_deg_s: float = 5.0
    max_command_joint_step_deg: float = 0.025
    max_tracking_error_deg: float = 1.0
    max_command_interval_sec: float = 0.010
    max_ik_fk_position_error_m: float = 0.001
    max_ik_fk_orientation_error_deg: float = 0.2

    def validate(self) -> None:
        for name, value in (
            ("control_dt_sec", self.control_dt_sec),
            ("max_command_joint_speed_deg_s", self.max_command_joint_speed_deg_s),
            ("max_command_joint_step_deg", self.max_command_joint_step_deg),
            ("max_tracking_error_deg", self.max_tracking_error_deg),
            ("max_command_interval_sec", self.max_command_interval_sec),
            ("max_ik_fk_position_error_m", self.max_ik_fk_position_error_m),
            ("max_ik_fk_orientation_error_deg", self.max_ik_fk_orientation_error_deg),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")

        speed_step = self.max_command_joint_speed_deg_s * self.control_dt_sec
        if self.max_command_joint_step_deg > speed_step + 1e-12:
            raise ValueError(
                "max_command_joint_step_deg must not exceed "
                "max_command_joint_speed_deg_s * control_dt_sec"
            )
        if self.max_command_interval_sec <= self.control_dt_sec:
            raise ValueError("max_command_interval_sec must exceed control_dt_sec")


@dataclass(frozen=True)
class JointStepResult:
    command_deg: tuple[float, ...]
    step_deg: tuple[float, ...]
    max_abs_step_deg: float
    limited: bool


@dataclass(frozen=True)
class TrackingErrorResult:
    error_deg: tuple[float, ...]
    max_abs_error_deg: float


def motion_permission(allow_motion: bool, confirmation: str) -> tuple[bool, str]:
    if not allow_motion:
        return False, "allow_motion_false"
    if confirmation != MOTION_CONFIRMATION_TOKEN:
        return False, "motion_confirmation_invalid"
    return True, "ok"


def validate_joint_vector(values: Iterable[float], name: str = "joint vector") -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != 6:
        raise ValueError(f"{name} must contain exactly 6 values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def joint_delta_deg(target_deg: Iterable[float], reference_deg: Iterable[float]) -> tuple[float, ...]:
    target = validate_joint_vector(target_deg, "target joints")
    reference = validate_joint_vector(reference_deg, "reference joints")
    return tuple(target[index] - reference[index] for index in range(6))


def limit_joint_step(
    *,
    current_deg: Iterable[float],
    target_deg: Iterable[float],
    max_step_deg: float,
) -> JointStepResult:
    current = validate_joint_vector(current_deg, "current joints")
    target = validate_joint_vector(target_deg, "target joints")
    if not math.isfinite(max_step_deg) or max_step_deg <= 0.0:
        raise ValueError("max_step_deg must be finite and greater than zero")

    command = []
    step = []
    limited = False
    for current_value, target_value in zip(current, target):
        raw_step = target_value - current_value
        clipped_step = max(-max_step_deg, min(max_step_deg, raw_step))
        if abs(clipped_step - raw_step) > 1e-12:
            limited = True
        step.append(clipped_step)
        command.append(current_value + clipped_step)

    max_abs = max(abs(value) for value in step)
    return JointStepResult(tuple(command), tuple(step), max_abs, limited)


def tracking_error(
    *,
    actual_deg: Iterable[float],
    commanded_deg: Iterable[float],
) -> TrackingErrorResult:
    error = joint_delta_deg(commanded_deg, actual_deg)
    return TrackingErrorResult(error, max(abs(value) for value in error))


def command_interval_valid(
    *,
    now_monotonic: float,
    previous_send_monotonic: float | None,
    max_interval_sec: float,
) -> tuple[bool, float]:
    if not math.isfinite(now_monotonic):
        raise ValueError("now_monotonic must be finite")
    if not math.isfinite(max_interval_sec) or max_interval_sec <= 0.0:
        raise ValueError("max_interval_sec must be finite and greater than zero")
    if previous_send_monotonic is None:
        return True, -1.0
    interval = max(0.0, now_monotonic - previous_send_monotonic)
    return interval <= max_interval_sec, interval
