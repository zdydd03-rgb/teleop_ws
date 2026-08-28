from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from .target_math import (
    IDENTITY_POSE,
    Pose3D,
    Vector3,
    generate_raw_target,
    limit_pose_step,
    pose_within_workspace,
    quaternion_angular_distance,
    quaternion_slerp,
    validate_pose,
    vector_norm,
    vector_subtract,
    validate_workspace_bounds,
)


@dataclass(frozen=True)
class PicoInput:
    seq: int
    received_monotonic: float
    packet_valid: bool
    transport_alive: bool
    hand_teleop_enabled: bool
    palm_valid: bool
    reset_origin: bool
    palm_pose: Pose3D


@dataclass(frozen=True)
class FlangeInput:
    received_monotonic: float
    pose: Pose3D


@dataclass(frozen=True)
class TargetOutput:
    source_seq: int
    active: bool
    target_valid: bool
    state: str
    reason: str
    pico_age_sec: float
    flange_age_sec: float
    raw_pico_pose: Pose3D
    filtered_pico_pose: Pose3D
    input_motion_state: str
    still_hold_active: bool
    settling_active: bool
    settling_remaining_translation_m: float
    settling_remaining_rotation_deg: float
    motion_window_translation_m: float
    motion_window_rotation_deg: float
    raw_target_pose: Pose3D
    limited_target_pose: Pose3D


@dataclass(frozen=True)
class TargetGeneratorConfig:
    control_hz: float = 100.0
    pico_freshness_timeout_sec: float = 0.10
    flange_freshness_timeout_sec: float = 0.10
    position_scale: float = 0.25
    max_translation_speed_mps: float = 0.025
    max_rotation_speed_rad_s: float = math.radians(8.0)

    # Input stabilization. Motion is detected from the filtered PICO pose over
    # a sliding window. SETTLING completes the latched endpoint before STILL.
    motion_gate_enabled: bool = True
    input_filter_time_constant_sec: float = 0.060
    motion_detection_window_sec: float = 0.40
    motion_start_translation_m: float = 0.004
    motion_start_rotation_rad: float = math.radians(2.0)
    motion_stop_translation_m: float = 0.0035
    motion_stop_rotation_rad: float = math.radians(1.5)
    motion_stop_hold_sec: float = 0.30

    workspace_limit_enabled: bool = False
    workspace_min_xyz: Vector3 = (-10.0, -10.0, -10.0)
    workspace_max_xyz: Vector3 = (10.0, 10.0, 10.0)

    def validate(self) -> None:
        positive_values = {
            "control_hz": self.control_hz,
            "pico_freshness_timeout_sec": self.pico_freshness_timeout_sec,
            "flange_freshness_timeout_sec": self.flange_freshness_timeout_sec,
            "motion_detection_window_sec": self.motion_detection_window_sec,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")

        non_negative_values = {
            "position_scale": self.position_scale,
            "max_translation_speed_mps": self.max_translation_speed_mps,
            "max_rotation_speed_rad_s": self.max_rotation_speed_rad_s,
            "input_filter_time_constant_sec": self.input_filter_time_constant_sec,
            "motion_start_translation_m": self.motion_start_translation_m,
            "motion_start_rotation_rad": self.motion_start_rotation_rad,
            "motion_stop_translation_m": self.motion_stop_translation_m,
            "motion_stop_rotation_rad": self.motion_stop_rotation_rad,
            "motion_stop_hold_sec": self.motion_stop_hold_sec,
        }
        for name, value in non_negative_values.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

        if self.motion_start_translation_m < self.motion_stop_translation_m:
            raise ValueError(
                "motion_start_translation_m must be greater than or equal to "
                "motion_stop_translation_m"
            )
        if self.motion_start_rotation_rad < self.motion_stop_rotation_rad:
            raise ValueError(
                "motion_start_rotation_rad must be greater than or equal to "
                "motion_stop_rotation_rad"
            )

        if self.workspace_limit_enabled:
            validate_workspace_bounds(
                self.workspace_min_xyz,
                self.workspace_max_xyz,
            )


class TargetGeneratorCore:
    """Pure state machine and target math without ROS or robot SDK dependencies."""

    def __init__(self, config: TargetGeneratorConfig) -> None:
        config.validate()
        self.config = config

        self._pico: PicoInput | None = None
        self._flange: FlangeInput | None = None

        self._active = False
        self._seen_disabled = False
        self._last_enable = False
        self._pending_rising_edge = False

        self._pico_anchor: Pose3D | None = None
        self._flange_anchor: Pose3D | None = None
        self._limited_target: Pose3D | None = None

        self._filtered_pico_pose: Pose3D | None = None
        self._filter_received_monotonic: float | None = None
        self._motion_history: deque[tuple[float, Pose3D]] = deque()
        self._input_motion_state = "STILL"
        self._motion_stop_candidate_since: float | None = None
        self._motion_window_translation_m = 0.0
        self._motion_window_rotation_rad = 0.0
        self._settling_target: Pose3D | None = None
        self._settling_remaining_translation_m = 0.0
        self._settling_remaining_rotation_rad = 0.0

    @property
    def active(self) -> bool:
        return self._active

    def update_pico(self, value: PicoInput) -> None:
        if not 0 <= value.seq <= 0xFFFFFFFF:
            raise ValueError("PICO seq must be uint32")
        if not math.isfinite(value.received_monotonic):
            raise ValueError("PICO receive time must be finite")

        if value.palm_valid:
            validate_pose(value.palm_pose)

        enabled = bool(value.hand_teleop_enabled)
        if not enabled:
            self._seen_disabled = True
            self._pending_rising_edge = False
        elif not self._last_enable and self._seen_disabled:
            self._pending_rising_edge = True

        self._last_enable = enabled
        self._pico = value

        if enabled and value.palm_valid:
            self._update_filtered_pico(value)
        else:
            # A new teleoperation session must start from a fresh filter and
            # detector state. This also prevents old drift from carrying over.
            self._reset_input_processing(clear_filter=True)

    def update_flange(self, value: FlangeInput) -> None:
        if not math.isfinite(value.received_monotonic):
            raise ValueError("flange receive time must be finite")
        validate_pose(value.pose)
        self._flange = value

    def step(self, now_monotonic: float) -> TargetOutput:
        if not math.isfinite(now_monotonic):
            raise ValueError("current monotonic time must be finite")

        pico_age = self._age(
            now_monotonic,
            self._pico.received_monotonic if self._pico else None,
        )
        flange_age = self._age(
            now_monotonic,
            self._flange.received_monotonic if self._flange else None,
        )
        source_seq = self._pico.seq if self._pico is not None else 0

        if self._active:
            invalid_reason = self._active_invalid_reason(pico_age, flange_age)
            if invalid_reason is not None:
                return self._deactivate(
                    source_seq,
                    invalid_reason,
                    pico_age,
                    flange_age,
                )

            assert self._pico is not None
            assert self._pico_anchor is not None
            assert self._flange_anchor is not None
            assert self._limited_target is not None

            if self._pico.reset_origin:
                return self._deactivate(
                    source_seq,
                    "reset_origin",
                    pico_age,
                    flange_age,
                )

            pico_current = self._current_filtered_pico_pose()
            enter_settling = self._update_motion_gate_before_target(
                now_monotonic,
                pico_current,
            )

            if self.config.motion_gate_enabled and self._input_motion_state == "STILL":
                held_target = self._limited_target
                self._clear_settling_target()
                return self._active_output(
                    source_seq=source_seq,
                    reason="still_hold",
                    pico_age=pico_age,
                    flange_age=flange_age,
                    raw_target=held_target,
                    limited_target=held_target,
                )

            if self._input_motion_state == "SETTLING":
                if self._settling_target is None:
                    return self._deactivate(
                        source_seq,
                        "settling_target_missing",
                        pico_age,
                        flange_age,
                    )
                raw_target = self._settling_target
            else:
                try:
                    raw_target = generate_raw_target(
                        pico_anchor=self._pico_anchor,
                        pico_current=pico_current,
                        flange_anchor=self._flange_anchor,
                        position_scale=self.config.position_scale,
                    )
                except ValueError as exc:
                    return self._deactivate(
                        source_seq,
                        f"target_math_invalid:{exc}",
                        pico_age,
                        flange_age,
                    )

                if enter_settling:
                    # Lock the user's final requested pose.  Subsequent PICO
                    # jitter is ignored while the rate-limited target catches up.
                    self._input_motion_state = "SETTLING"
                    self._settling_target = validate_pose(raw_target)
                    self._motion_stop_candidate_since = None

            if self.config.workspace_limit_enabled and not pose_within_workspace(
                raw_target,
                self.config.workspace_min_xyz,
                self.config.workspace_max_xyz,
            ):
                return self._deactivate(
                    source_seq,
                    "workspace_violation",
                    pico_age,
                    flange_age,
                    raw_target=raw_target,
                )

            nominal_dt = 1.0 / self.config.control_hz
            limited_target = limit_pose_step(
                current=self._limited_target,
                target=raw_target,
                max_translation_distance=(
                    self.config.max_translation_speed_mps * nominal_dt
                ),
                max_rotation_angle_rad=(
                    self.config.max_rotation_speed_rad_s * nominal_dt
                ),
            )
            self._limited_target = limited_target

            if self._input_motion_state == "SETTLING":
                self._update_settling_remaining(limited_target)
                if self._settling_complete():
                    # limit_pose_step snaps exactly to the target when it is
                    # within one permitted step. Re-anchor only after the full
                    # latched target has been reached, then freeze.
                    assert self._settling_target is not None
                    self._limited_target = self._settling_target
                    self._reanchor_at_current_target(pico_current)
                    held_target = self._limited_target
                    self._input_motion_state = "STILL"
                    self._clear_settling_target()
                    return self._active_output(
                        source_seq=source_seq,
                        reason="still_hold",
                        pico_age=pico_age,
                        flange_age=flange_age,
                        raw_target=held_target,
                        limited_target=held_target,
                    )

                return self._active_output(
                    source_seq=source_seq,
                    reason="settling",
                    pico_age=pico_age,
                    flange_age=flange_age,
                    raw_target=raw_target,
                    limited_target=limited_target,
                )

            return self._active_output(
                source_seq=source_seq,
                reason="tracking",
                pico_age=pico_age,
                flange_age=flange_age,
                raw_target=raw_target,
                limited_target=limited_target,
            )

        return self._step_inactive(source_seq, pico_age, flange_age)

    def _step_inactive(
        self,
        source_seq: int,
        pico_age: float,
        flange_age: float,
    ) -> TargetOutput:
        if self._pico is None:
            return self._inactive_output(
                source_seq,
                "WAITING_FOR_INPUT",
                "no_pico_message",
                pico_age,
                flange_age,
            )

        if not self._pico.packet_valid:
            return self._inactive_output(
                source_seq,
                "WAITING_FOR_INPUT",
                "pico_packet_invalid",
                pico_age,
                flange_age,
            )

        if not self._pico.transport_alive:
            return self._inactive_output(
                source_seq,
                "WAITING_FOR_INPUT",
                "pico_transport_not_alive",
                pico_age,
                flange_age,
            )

        if pico_age > self.config.pico_freshness_timeout_sec:
            return self._inactive_output(
                source_seq,
                "WAITING_FOR_INPUT",
                "pico_stale",
                pico_age,
                flange_age,
            )

        if not self._pico.hand_teleop_enabled:
            return self._inactive_output(
                source_seq,
                "READY",
                "waiting_for_enable",
                pico_age,
                flange_age,
            )

        if not self._pending_rising_edge:
            reason = (
                "awaiting_enable_release"
                if not self._seen_disabled
                else "awaiting_new_enable_rising_edge"
            )
            return self._inactive_output(
                source_seq,
                "READY",
                reason,
                pico_age,
                flange_age,
            )

        activation_error = self._activation_invalid_reason(pico_age, flange_age)
        if activation_error is not None:
            self._pending_rising_edge = False
            return self._inactive_output(
                source_seq,
                "READY",
                f"activation_rejected:{activation_error}",
                pico_age,
                flange_age,
            )

        assert self._flange is not None
        pico_current = self._current_filtered_pico_pose()
        self._active = True
        self._pending_rising_edge = False
        self._pico_anchor = validate_pose(pico_current)
        self._flange_anchor = validate_pose(self._flange.pose)
        self._limited_target = self._flange_anchor
        self._input_motion_state = (
            "STILL" if self.config.motion_gate_enabled else "MOVING"
        )
        self._motion_stop_candidate_since = None
        self._clear_settling_target()

        return self._active_output(
            source_seq=source_seq,
            reason="activated",
            pico_age=pico_age,
            flange_age=flange_age,
            raw_target=self._flange_anchor,
            limited_target=self._flange_anchor,
        )

    def _activation_invalid_reason(
        self,
        pico_age: float,
        flange_age: float,
    ) -> str | None:
        assert self._pico is not None

        if not self._pico.palm_valid:
            return "palm_invalid"
        if self._pico.reset_origin:
            return "reset_origin"
        if pico_age > self.config.pico_freshness_timeout_sec:
            return "pico_stale"
        if self._flange is None:
            return "no_flange_pose"
        if flange_age > self.config.flange_freshness_timeout_sec:
            return "flange_stale"

        try:
            validate_pose(self._pico.palm_pose)
            self._current_filtered_pico_pose()
            validate_pose(self._flange.pose)
        except ValueError as exc:
            return f"pose_invalid:{exc}"

        if self.config.workspace_limit_enabled and not pose_within_workspace(
            self._flange.pose,
            self.config.workspace_min_xyz,
            self.config.workspace_max_xyz,
        ):
            return "flange_outside_workspace"

        return None

    def _active_invalid_reason(
        self,
        pico_age: float,
        flange_age: float,
    ) -> str | None:
        if self._pico is None:
            return "no_pico_message"
        if not self._pico.packet_valid:
            return "pico_packet_invalid"
        if not self._pico.transport_alive:
            return "pico_transport_not_alive"
        if not self._pico.hand_teleop_enabled:
            return "teleop_disabled"
        if not self._pico.palm_valid:
            return "palm_invalid"
        if pico_age > self.config.pico_freshness_timeout_sec:
            return "pico_stale"
        if self._flange is None:
            return "no_flange_pose"
        if flange_age > self.config.flange_freshness_timeout_sec:
            return "flange_stale"

        try:
            validate_pose(self._pico.palm_pose)
            self._current_filtered_pico_pose()
            validate_pose(self._flange.pose)
        except ValueError as exc:
            return f"pose_invalid:{exc}"

        return None

    def _update_filtered_pico(self, value: PicoInput) -> None:
        raw_pose = validate_pose(value.palm_pose)
        previous_time = self._filter_received_monotonic
        previous_pose = self._filtered_pico_pose

        reset_filter = (
            previous_time is None
            or previous_pose is None
            or value.received_monotonic <= previous_time
            or value.received_monotonic - previous_time
            > self.config.pico_freshness_timeout_sec
        )

        if reset_filter:
            filtered = raw_pose
            self._motion_history.clear()
        else:
            dt = value.received_monotonic - previous_time
            tau = self.config.input_filter_time_constant_sec
            alpha = 1.0 if tau <= 0.0 else 1.0 - math.exp(-dt / tau)
            filtered = Pose3D(
                position=tuple(
                    previous_pose.position[index]
                    + alpha * (raw_pose.position[index] - previous_pose.position[index])
                    for index in range(3)
                ),
                orientation=quaternion_slerp(
                    previous_pose.orientation,
                    raw_pose.orientation,
                    alpha,
                ),
            )
            filtered = validate_pose(filtered)

        self._filtered_pico_pose = filtered
        self._filter_received_monotonic = value.received_monotonic
        self._motion_history.append((value.received_monotonic, filtered))
        self._trim_motion_history(value.received_monotonic)
        self._update_motion_window_metrics()

    def _trim_motion_history(self, now: float) -> None:
        cutoff = now - self.config.motion_detection_window_sec
        # Retain one sample at or immediately before the cutoff when possible,
        # which gives a stable approximately-window-length displacement.
        while len(self._motion_history) >= 2 and self._motion_history[1][0] <= cutoff:
            self._motion_history.popleft()

    def _update_motion_window_metrics(self) -> None:
        if len(self._motion_history) < 2:
            self._motion_window_translation_m = 0.0
            self._motion_window_rotation_rad = 0.0
            return

        start_pose = self._motion_history[0][1]
        end_pose = self._motion_history[-1][1]
        delta = tuple(
            end_pose.position[index] - start_pose.position[index]
            for index in range(3)
        )
        self._motion_window_translation_m = math.sqrt(
            sum(component * component for component in delta)
        )
        self._motion_window_rotation_rad = quaternion_angular_distance(
            start_pose.orientation,
            end_pose.orientation,
        )

    def _update_motion_gate_before_target(
        self,
        now: float,
        pico_current: Pose3D,
    ) -> bool:
        """Update STILL/MOVING/SETTLING transitions before target generation.

        Returns True only when MOVING has satisfied the stop criteria long
        enough and the current live target should be latched for SETTLING.
        """
        if not self.config.motion_gate_enabled:
            self._input_motion_state = "MOVING"
            self._motion_stop_candidate_since = None
            self._clear_settling_target()
            return False

        moving_request = (
            self._motion_window_translation_m
            >= self.config.motion_start_translation_m
            or self._motion_window_rotation_rad
            >= self.config.motion_start_rotation_rad
        )
        stopping_request = (
            self._motion_window_translation_m
            <= self.config.motion_stop_translation_m
            and self._motion_window_rotation_rad
            <= self.config.motion_stop_rotation_rad
        )

        if self._input_motion_state == "STILL":
            if moving_request:
                self._input_motion_state = "MOVING"
                self._motion_stop_candidate_since = None
                self._clear_settling_target()
                self._reanchor_at_current_target(pico_current)
            return False

        if self._input_motion_state == "SETTLING":
            # A new deliberate motion cancels the old latched endpoint and
            # starts a fresh zero-jump mapping from the current robot target.
            if moving_request:
                self._input_motion_state = "MOVING"
                self._motion_stop_candidate_since = None
                self._clear_settling_target()
                self._reanchor_at_current_target(pico_current)
            return False

        if stopping_request:
            if self._motion_stop_candidate_since is None:
                self._motion_stop_candidate_since = now
            elif (
                now - self._motion_stop_candidate_since
                >= self.config.motion_stop_hold_sec
            ):
                return True
        else:
            self._motion_stop_candidate_since = None

        return False

    def _update_settling_remaining(self, limited_target: Pose3D) -> None:
        if self._settling_target is None:
            self._settling_remaining_translation_m = 0.0
            self._settling_remaining_rotation_rad = 0.0
            return
        self._settling_remaining_translation_m = vector_norm(
            vector_subtract(
                self._settling_target.position,
                limited_target.position,
            )
        )
        self._settling_remaining_rotation_rad = quaternion_angular_distance(
            limited_target.orientation,
            self._settling_target.orientation,
        )

    def _settling_complete(self) -> bool:
        return (
            self._settling_target is not None
            and self._settling_remaining_translation_m <= 1.0e-10
            and self._settling_remaining_rotation_rad <= 1.0e-10
        )

    def _clear_settling_target(self) -> None:
        self._settling_target = None
        self._settling_remaining_translation_m = 0.0
        self._settling_remaining_rotation_rad = 0.0

    def _reanchor_at_current_target(self, pico_current: Pose3D) -> None:
        assert self._limited_target is not None
        self._pico_anchor = validate_pose(pico_current)
        self._flange_anchor = validate_pose(self._limited_target)

    def _current_filtered_pico_pose(self) -> Pose3D:
        if self._filtered_pico_pose is not None:
            return validate_pose(self._filtered_pico_pose)
        if self._pico is not None and self._pico.palm_valid:
            return validate_pose(self._pico.palm_pose)
        raise ValueError("no valid filtered PICO pose")

    def _active_output(
        self,
        *,
        source_seq: int,
        reason: str,
        pico_age: float,
        flange_age: float,
        raw_target: Pose3D,
        limited_target: Pose3D,
    ) -> TargetOutput:
        raw_pico, filtered_pico = self._diagnostic_pico_poses()
        return TargetOutput(
            source_seq=source_seq,
            active=True,
            target_valid=True,
            state="ACTIVE",
            reason=reason,
            pico_age_sec=pico_age,
            flange_age_sec=flange_age,
            raw_pico_pose=raw_pico,
            filtered_pico_pose=filtered_pico,
            input_motion_state=self._input_motion_state,
            still_hold_active=(
                self.config.motion_gate_enabled
                and self._input_motion_state == "STILL"
            ),
            settling_active=(self._input_motion_state == "SETTLING"),
            settling_remaining_translation_m=(
                self._settling_remaining_translation_m
            ),
            settling_remaining_rotation_deg=math.degrees(
                self._settling_remaining_rotation_rad
            ),
            motion_window_translation_m=self._motion_window_translation_m,
            motion_window_rotation_deg=math.degrees(
                self._motion_window_rotation_rad
            ),
            raw_target_pose=validate_pose(raw_target),
            limited_target_pose=validate_pose(limited_target),
        )

    def _deactivate(
        self,
        source_seq: int,
        reason: str,
        pico_age: float,
        flange_age: float,
        *,
        raw_target: Pose3D = IDENTITY_POSE,
    ) -> TargetOutput:
        self._active = False
        self._pending_rising_edge = False
        self._pico_anchor = None
        self._flange_anchor = None
        self._limited_target = None
        self._input_motion_state = "STILL"
        self._motion_stop_candidate_since = None
        self._clear_settling_target()

        raw_pico, filtered_pico = self._diagnostic_pico_poses()
        return TargetOutput(
            source_seq=source_seq,
            active=False,
            target_valid=False,
            state="READY",
            reason=reason,
            pico_age_sec=pico_age,
            flange_age_sec=flange_age,
            raw_pico_pose=raw_pico,
            filtered_pico_pose=filtered_pico,
            input_motion_state=self._input_motion_state,
            still_hold_active=False,
            settling_active=False,
            settling_remaining_translation_m=0.0,
            settling_remaining_rotation_deg=0.0,
            motion_window_translation_m=self._motion_window_translation_m,
            motion_window_rotation_deg=math.degrees(
                self._motion_window_rotation_rad
            ),
            raw_target_pose=raw_target,
            limited_target_pose=IDENTITY_POSE,
        )

    def _reset_input_processing(self, *, clear_filter: bool) -> None:
        if clear_filter:
            self._filtered_pico_pose = None
            self._filter_received_monotonic = None
        self._motion_history.clear()
        self._input_motion_state = "STILL"
        self._motion_stop_candidate_since = None
        self._motion_window_translation_m = 0.0
        self._motion_window_rotation_rad = 0.0
        self._clear_settling_target()

    def _diagnostic_pico_poses(self) -> tuple[Pose3D, Pose3D]:
        raw_pose = IDENTITY_POSE
        if self._pico is not None and self._pico.palm_valid:
            raw_pose = validate_pose(self._pico.palm_pose)
        filtered_pose = (
            validate_pose(self._filtered_pico_pose)
            if self._filtered_pico_pose is not None
            else raw_pose
        )
        return raw_pose, filtered_pose

    @staticmethod
    def _age(now: float, received: float | None) -> float:
        if received is None:
            return math.inf
        return max(0.0, now - received)

    def _inactive_output(
        self,
        source_seq: int,
        state: str,
        reason: str,
        pico_age: float,
        flange_age: float,
    ) -> TargetOutput:
        raw_pico, filtered_pico = self._diagnostic_pico_poses()
        return TargetOutput(
            source_seq=source_seq,
            active=False,
            target_valid=False,
            state=state,
            reason=reason,
            pico_age_sec=pico_age,
            flange_age_sec=flange_age,
            raw_pico_pose=raw_pico,
            filtered_pico_pose=filtered_pico,
            input_motion_state=self._input_motion_state,
            still_hold_active=False,
            settling_active=False,
            settling_remaining_translation_m=0.0,
            settling_remaining_rotation_deg=0.0,
            motion_window_translation_m=self._motion_window_translation_m,
            motion_window_rotation_deg=math.degrees(
                self._motion_window_rotation_rad
            ),
            raw_target_pose=IDENTITY_POSE,
            limited_target_pose=IDENTITY_POSE,
        )
