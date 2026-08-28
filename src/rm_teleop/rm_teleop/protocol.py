from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

UINT32_MAX = 0xFFFFFFFF
EXPECTED_PROTOCOL_VERSION = 1


class ProtocolError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class ProtocolConfig:
    expected_version: int = EXPECTED_PROTOCOL_VERSION
    quaternion_norm_min: float = 0.5
    quaternion_norm_max: float = 1.5
    max_abs_position_m: float = 10.0
    max_target_hz: int = 1000


@dataclass(frozen=True)
class Vector3Data:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class QuaternionData:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class PoseData:
    valid: bool
    position: Vector3Data
    rotation: QuaternionData


@dataclass(frozen=True)
class TeleopPacket:
    protocol_version: int
    seq: int
    source_timestamp_sec: float
    target_hz: int
    deadman: bool
    input_mode: str
    teleop_armed: bool
    stop_reason: str
    reset_origin: bool
    record_toggle: bool
    left_controller_connected: bool
    right_hand_tracked: bool
    palm: PoseData
    wrist: PoseData
    hand_teleop_enabled: bool


def _reject_json_constant(value: str) -> None:
    raise ProtocolError("json_nonfinite", f"JSON contains invalid constant: {value}")


def _ensure_finite_tree(value: Any, path: str = "$") -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return

    if isinstance(value, int):
        return

    if isinstance(value, float):
        if not math.isfinite(value):
            raise ProtocolError("json_nonfinite", f"non-finite number at {path}")
        return

    if isinstance(value, list):
        for index, item in enumerate(value):
            _ensure_finite_tree(item, f"{path}[{index}]")
        return

    if isinstance(value, dict):
        for key, item in value.items():
            _ensure_finite_tree(item, f"{path}.{key}")
        return

    raise ProtocolError(
        "json_type_invalid",
        f"unsupported JSON value type at {path}: {type(value).__name__}",
    )


def _require_object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProtocolError("field_type", f"{path} must be an object")
    return value


def _require_bool(obj: dict[str, Any], key: str, path: str = "$") -> bool:
    if key not in obj:
        raise ProtocolError("field_missing", f"missing field {path}.{key}")
    value = obj[key]
    if type(value) is not bool:
        raise ProtocolError("field_type", f"{path}.{key} must be a boolean")
    return value


def _require_string(obj: dict[str, Any], key: str, path: str = "$") -> str:
    if key not in obj:
        raise ProtocolError("field_missing", f"missing field {path}.{key}")
    value = obj[key]
    if not isinstance(value, str):
        raise ProtocolError("field_type", f"{path}.{key} must be a string")
    return value


def _require_int(obj: dict[str, Any], key: str, path: str = "$") -> int:
    if key not in obj:
        raise ProtocolError("field_missing", f"missing field {path}.{key}")
    value = obj[key]
    if type(value) is not int:
        raise ProtocolError("field_type", f"{path}.{key} must be an integer")
    return value


def _require_number(obj: dict[str, Any], key: str, path: str) -> float:
    if key not in obj:
        raise ProtocolError("field_missing", f"missing field {path}.{key}")
    value = obj[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("field_type", f"{path}.{key} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError("field_nonfinite", f"{path}.{key} must be finite")
    return result


def _parse_vector3(
    value: Any,
    path: str,
    config: ProtocolConfig,
    enforce_range: bool,
) -> Vector3Data:
    obj = _require_object(value, path)
    vector = Vector3Data(
        x=_require_number(obj, "x", path),
        y=_require_number(obj, "y", path),
        z=_require_number(obj, "z", path),
    )

    if enforce_range:
        maximum = max(abs(vector.x), abs(vector.y), abs(vector.z))
        if maximum > config.max_abs_position_m:
            raise ProtocolError(
                "position_range",
                f"{path} exceeds max_abs_position_m={config.max_abs_position_m}",
            )

    return vector


def _parse_quaternion(
    value: Any,
    path: str,
    config: ProtocolConfig,
    normalize: bool,
) -> QuaternionData:
    obj = _require_object(value, path)
    quaternion = QuaternionData(
        x=_require_number(obj, "x", path),
        y=_require_number(obj, "y", path),
        z=_require_number(obj, "z", path),
        w=_require_number(obj, "w", path),
    )

    if not normalize:
        return quaternion

    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )

    if norm < config.quaternion_norm_min or norm > config.quaternion_norm_max:
        raise ProtocolError(
            "quaternion_norm",
            f"{path} norm={norm:.9g} outside "
            f"[{config.quaternion_norm_min}, {config.quaternion_norm_max}]",
        )

    inverse_norm = 1.0 / norm
    return QuaternionData(
        x=quaternion.x * inverse_norm,
        y=quaternion.y * inverse_norm,
        z=quaternion.z * inverse_norm,
        w=quaternion.w * inverse_norm,
    )


def _invalid_pose() -> PoseData:
    return PoseData(
        valid=False,
        position=Vector3Data(0.0, 0.0, 0.0),
        rotation=QuaternionData(0.0, 0.0, 0.0, 1.0),
    )


def _parse_pose(value: Any, path: str, config: ProtocolConfig) -> PoseData:
    obj = _require_object(value, path)
    valid = _require_bool(obj, "valid", path)
    if "position" not in obj:
        raise ProtocolError("field_missing", f"missing field {path}.position")
    if "rotation" not in obj:
        raise ProtocolError("field_missing", f"missing field {path}.rotation")

    # Even invalid poses must retain the expected schema and finite numbers.
    # Their values are deliberately discarded so stale APK values cannot leak
    # into downstream control messages.
    position = _parse_vector3(
        obj.get("position"),
        f"{path}.position",
        config,
        enforce_range=valid,
    )
    rotation = _parse_quaternion(
        obj.get("rotation"),
        f"{path}.rotation",
        config,
        normalize=valid,
    )

    if not valid:
        return _invalid_pose()

    return PoseData(valid=True, position=position, rotation=rotation)


def parse_datagram(data: bytes, config: ProtocolConfig | None = None) -> TeleopPacket:
    config = config or ProtocolConfig()

    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProtocolError("utf8", f"invalid UTF-8: {exc}") from exc

    try:
        root = json.loads(text, parse_constant=_reject_json_constant)
    except ProtocolError:
        raise
    except json.JSONDecodeError as exc:
        raise ProtocolError(
            "json",
            f"invalid JSON at line {exc.lineno}, column {exc.colno}: {exc.msg}",
        ) from exc

    _ensure_finite_tree(root)
    packet = _require_object(root, "$")

    version = _require_int(packet, "version")
    if version != config.expected_version:
        raise ProtocolError(
            "version",
            f"unsupported version {version}, expected {config.expected_version}",
        )

    seq = _require_int(packet, "seq")
    if seq < 0 or seq > UINT32_MAX:
        raise ProtocolError("seq_range", f"seq out of uint32 range: {seq}")

    source_timestamp_sec = _require_number(packet, "timestamp", "$")
    if source_timestamp_sec < 0.0:
        raise ProtocolError("timestamp_range", "timestamp must be non-negative")

    target_hz = _require_int(packet, "target_hz")
    if target_hz <= 0 or target_hz > config.max_target_hz:
        raise ProtocolError(
            "target_hz_range",
            f"target_hz must be in [1, {config.max_target_hz}]",
        )

    deadman = _require_bool(packet, "deadman")
    input_mode = _require_string(packet, "input_mode")
    teleop_armed = _require_bool(packet, "teleop_armed")
    stop_reason = _require_string(packet, "stop_reason")
    reset_origin = _require_bool(packet, "reset_origin")
    record_toggle = _require_bool(packet, "record_toggle")
    left_controller_connected = _require_bool(packet, "left_controller_connected")
    right_hand_tracked = _require_bool(packet, "right_hand_tracked")

    if "right_palm" not in packet:
        raise ProtocolError("field_missing", "missing field $.right_palm")
    if "right_wrist" not in packet:
        raise ProtocolError("field_missing", "missing field $.right_wrist")

    palm = _parse_pose(packet["right_palm"], "$.right_palm", config)
    wrist = _parse_pose(packet["right_wrist"], "$.right_wrist", config)

    # Tracking false is a hard validity gate even if the APK accidentally
    # leaves a pose object's valid flag true.
    if not right_hand_tracked:
        palm = _invalid_pose()
        wrist = _invalid_pose()

    hand_teleop_enabled = (
        deadman
        and teleop_armed
        and input_mode == "hand_teleop_active"
        and stop_reason == ""
        and not left_controller_connected
        and right_hand_tracked
        and palm.valid
    )

    return TeleopPacket(
        protocol_version=version,
        seq=seq,
        source_timestamp_sec=source_timestamp_sec,
        target_hz=target_hz,
        deadman=deadman,
        input_mode=input_mode,
        teleop_armed=teleop_armed,
        stop_reason=stop_reason,
        reset_origin=reset_origin,
        record_toggle=record_toggle,
        left_controller_connected=left_controller_connected,
        right_hand_tracked=right_hand_tracked,
        palm=palm,
        wrist=wrist,
        hand_teleop_enabled=hand_teleop_enabled,
    )
