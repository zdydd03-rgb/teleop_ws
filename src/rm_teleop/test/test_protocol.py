import json
import math

import pytest

from rm_teleop.protocol import ProtocolError, parse_datagram


def make_packet() -> dict:
    return {
        "version": 1,
        "seq": 10,
        "timestamp": 12.5,
        "target_hz": 30,
        "deadman": True,
        "input_mode": "hand_teleop_active",
        "teleop_armed": True,
        "stop_reason": "",
        "reset_origin": False,
        "record_toggle": False,
        "left_controller_connected": False,
        "right_hand_tracked": True,
        "right_palm": {
            "valid": True,
            "position": {"x": 0.1, "y": 1.2, "z": -0.3},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.2},
        },
        "right_wrist": {
            "valid": True,
            "position": {"x": 0.0, "y": 1.0, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "right_keypoints": [],
    }


def encode(packet: dict) -> bytes:
    return json.dumps(packet).encode("utf-8")


def test_valid_packet_is_parsed_and_quaternion_is_normalized() -> None:
    packet = parse_datagram(encode(make_packet()))

    assert packet.seq == 10
    assert packet.hand_teleop_enabled is True
    assert packet.palm.valid is True
    assert math.isclose(packet.palm.rotation.w, 1.0)


def test_invalid_pose_discards_stale_values() -> None:
    raw = make_packet()
    raw["deadman"] = False
    raw["right_hand_tracked"] = False
    raw["right_palm"]["valid"] = False
    raw["right_palm"]["position"] = {"x": 0.4, "y": 1.5, "z": -0.8}
    raw["right_palm"]["rotation"] = {
        "x": 0.1,
        "y": 0.2,
        "z": 0.3,
        "w": 0.9,
    }

    packet = parse_datagram(encode(raw))

    assert packet.palm.valid is False
    assert packet.palm.position.x == 0.0
    assert packet.palm.position.y == 0.0
    assert packet.palm.position.z == 0.0
    assert packet.palm.rotation.x == 0.0
    assert packet.palm.rotation.y == 0.0
    assert packet.palm.rotation.z == 0.0
    assert packet.palm.rotation.w == 1.0
    assert packet.hand_teleop_enabled is False


@pytest.mark.parametrize(
    "field,value",
    [
        ("deadman", False),
        ("teleop_armed", False),
        ("input_mode", "hand_lost"),
        ("stop_reason", "right_hand_lost"),
        ("left_controller_connected", True),
        ("right_hand_tracked", False),
    ],
)
def test_enable_requires_all_safety_fields(field: str, value: object) -> None:
    raw = make_packet()
    raw[field] = value

    packet = parse_datagram(encode(raw))

    assert packet.hand_teleop_enabled is False


def test_valid_zero_quaternion_is_rejected() -> None:
    raw = make_packet()
    raw["right_palm"]["rotation"] = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 0.0,
    }

    with pytest.raises(ProtocolError) as error:
        parse_datagram(encode(raw))

    assert error.value.code == "quaternion_norm"


def test_boolean_is_not_accepted_as_integer() -> None:
    raw = make_packet()
    raw["seq"] = True

    with pytest.raises(ProtocolError) as error:
        parse_datagram(encode(raw))

    assert error.value.code == "field_type"


def test_nonfinite_number_is_rejected_anywhere_in_json() -> None:
    raw = make_packet()
    raw["right_keypoints"] = [{"debug_value": float("nan")}]
    data = json.dumps(raw, allow_nan=True).encode("utf-8")

    with pytest.raises(ProtocolError) as error:
        parse_datagram(data)

    assert error.value.code == "json_nonfinite"
