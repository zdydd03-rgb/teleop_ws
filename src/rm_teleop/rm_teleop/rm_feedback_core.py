from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Iterable

from .target_math import Pose3D, quaternion_angular_distance, validate_pose


RM65_DOF = 6


@dataclass(frozen=True)
class RmFeedbackSnapshot:
    received_monotonic: float
    source_ip: str
    source_port: int
    parse_error_code: int
    joint_position_deg: tuple[float, ...]
    joint_speed_deg_s: tuple[float, ...]
    joint_enabled: tuple[bool, ...]
    joint_error_codes: tuple[int, ...]
    arm_error_codes: tuple[int, ...]
    waypoint: Pose3D
    arm_current_status: int


@dataclass(frozen=True)
class FeedbackStatistics:
    received: int
    rejected: int
    current_hz: float
    last_error: str


@dataclass(frozen=True)
class FkComparison:
    position_error_m: float
    orientation_error_rad: float


def _finite_tuple(values: Iterable[Any], length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)

    if len(result) < length:
        raise ValueError(f"{name} must contain at least {length} values")

    result = result[:length]

    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains a non-finite value")

    return result


def _bool_tuple(values: Iterable[Any], length: int, name: str) -> tuple[bool, ...]:
    result = tuple(bool(value) for value in values)

    if len(result) < length:
        raise ValueError(f"{name} must contain at least {length} values")

    return result[:length]


def _int_tuple(values: Iterable[Any], length: int, name: str) -> tuple[int, ...]:
    result = tuple(int(value) for value in values)

    if len(result) < length:
        raise ValueError(f"{name} must contain at least {length} values")

    return result[:length]


def _decode_ip(value: Any) -> str:
    if isinstance(value, bytes):
        return value.split(b"\0", 1)[0].decode("utf-8", errors="replace")

    if isinstance(value, str):
        return value

    try:
        raw = bytes(value)
        return raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")
    except (TypeError, ValueError):
        return str(value)


def _pose_from_api2(value: Any) -> Pose3D:
    position = value.position
    quaternion = value.quaternion

    return validate_pose(
        Pose3D(
            position=(
                float(position.x),
                float(position.y),
                float(position.z),
            ),
            # API2结构体顺序为w, x, y, z；项目内部统一x, y, z, w。
            orientation=(
                float(quaternion.x),
                float(quaternion.y),
                float(quaternion.z),
                float(quaternion.w),
            ),
        )
    )


def snapshot_from_api2_state(
    state: Any,
    received_monotonic: float | None = None,
    *,
    dof: int = RM65_DOF,
) -> RmFeedbackSnapshot:
    if dof <= 0:
        raise ValueError("dof must be greater than zero")

    receive_time = time.monotonic() if received_monotonic is None else float(received_monotonic)

    if not math.isfinite(receive_time):
        raise ValueError("received_monotonic must be finite")

    parse_error_code = int(state.errCode)

    if parse_error_code != 0:
        raise ValueError(f"API2 UDP parse error: {parse_error_code}")

    joint_status = state.joint_status
    err = state.err

    arm_error_count = int(err.err_len)
    raw_arm_errors = tuple(int(code) for code in err.err)

    if arm_error_count < 0 or arm_error_count > len(raw_arm_errors):
        raise ValueError("invalid arm error count")

    return RmFeedbackSnapshot(
        received_monotonic=receive_time,
        source_ip=_decode_ip(state.arm_ip),
        source_port=int(state.arm_port),
        parse_error_code=parse_error_code,
        joint_position_deg=_finite_tuple(
            joint_status.joint_position,
            dof,
            "joint_position",
        ),
        joint_speed_deg_s=_finite_tuple(
            joint_status.joint_speed,
            dof,
            "joint_speed",
        ),
        joint_enabled=_bool_tuple(
            joint_status.joint_en_flag,
            dof,
            "joint_en_flag",
        ),
        joint_error_codes=_int_tuple(
            joint_status.joint_err_code,
            dof,
            "joint_err_code",
        ),
        # Some controller/API2 versions report err_len=1 with err[0]=0
        # when there is no arm error. Keep only actual non-zero codes.
        arm_error_codes=tuple(
            code
            for code in raw_arm_errors[:arm_error_count]
            if code != 0
        ),
        waypoint=_pose_from_api2(state.waypoint),
        arm_current_status=int(state.arm_current_status),
    )


def pose_from_fk_result(values: Iterable[Any]) -> Pose3D:
    result = _finite_tuple(values, 7, "forward kinematics result")

    # API2 rm_algo_forward_kinematics(flag=0)返回[x, y, z, w, x, y, z]。
    return validate_pose(
        Pose3D(
            position=(result[0], result[1], result[2]),
            orientation=(result[4], result[5], result[6], result[3]),
        )
    )


def compare_poses(left: Pose3D, right: Pose3D) -> FkComparison:
    left = validate_pose(left)
    right = validate_pose(right)

    dx = left.position[0] - right.position[0]
    dy = left.position[1] - right.position[1]
    dz = left.position[2] - right.position[2]

    return FkComparison(
        position_error_m=math.sqrt(dx * dx + dy * dy + dz * dz),
        orientation_error_rad=quaternion_angular_distance(
            left.orientation,
            right.orientation,
        ),
    )


class FeedbackStore:
    def __init__(self, rate_window_size: int = 400) -> None:
        if rate_window_size < 2:
            raise ValueError("rate_window_size must be at least 2")

        self._lock = threading.Lock()
        self._snapshot: RmFeedbackSnapshot | None = None
        self._packet_times: deque[float] = deque(maxlen=rate_window_size)
        self._received = 0
        self._rejected = 0
        self._last_error = ""

    def update(self, snapshot: RmFeedbackSnapshot) -> None:
        with self._lock:
            self._snapshot = snapshot
            self._packet_times.append(snapshot.received_monotonic)
            self._received += 1
            self._last_error = ""

    def reject(self, error: str) -> None:
        with self._lock:
            self._rejected += 1
            self._last_error = str(error)

    def latest(self) -> RmFeedbackSnapshot | None:
        with self._lock:
            return self._snapshot

    def statistics(self) -> FeedbackStatistics:
        with self._lock:
            packet_times = tuple(self._packet_times)
            received = self._received
            rejected = self._rejected
            last_error = self._last_error

        hz = 0.0

        if len(packet_times) >= 2:
            elapsed = packet_times[-1] - packet_times[0]

            if elapsed > 0.0:
                hz = (len(packet_times) - 1) / elapsed

        return FeedbackStatistics(
            received=received,
            rejected=rejected,
            current_hz=hz,
            last_error=last_error,
        )
