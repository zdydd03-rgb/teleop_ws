import json

from rm_teleop.receiver_core import ReceiverCore, SequenceStatus


def make_data(seq: int, timestamp: float = 10.0, palm_valid: bool = True) -> bytes:
    packet = {
        "version": 1,
        "seq": seq,
        "timestamp": timestamp,
        "target_hz": 30,
        "deadman": palm_valid,
        "input_mode": "hand_teleop_active" if palm_valid else "hand_lost",
        "teleop_armed": True,
        "stop_reason": "" if palm_valid else "right_hand_lost",
        "reset_origin": False,
        "record_toggle": False,
        "left_controller_connected": False,
        "right_hand_tracked": palm_valid,
        "right_palm": {
            "valid": palm_valid,
            "position": {"x": 0.1, "y": 1.0, "z": 0.2},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "right_wrist": {
            "valid": palm_valid,
            "position": {"x": 0.1, "y": 0.9, "z": 0.2},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "right_keypoints": [],
    }
    return json.dumps(packet).encode("utf-8")


def test_duplicate_does_not_refresh_last_accepted_time() -> None:
    core = ReceiverCore()
    core.process_datagram(make_data(1), 1.0)

    result = core.process_datagram(make_data(1), 1.1)

    assert result.accepted is False
    assert core.last_datagram_monotonic == 1.1
    assert core.last_accepted_monotonic == 1.0


def test_invalid_packet_does_not_refresh_last_accepted_time() -> None:
    core = ReceiverCore()
    core.process_datagram(make_data(1), 1.0)

    result = core.process_datagram(b"not-json", 1.1)

    assert result.accepted is False
    assert core.last_datagram_monotonic == 1.1
    assert core.last_accepted_monotonic == 1.0


def test_invalid_palm_does_not_refresh_valid_palm_time() -> None:
    core = ReceiverCore()
    core.process_datagram(make_data(1, palm_valid=True), 1.0)
    core.process_datagram(make_data(2, palm_valid=False), 1.1)

    assert core.last_accepted_monotonic == 1.1
    assert core.last_valid_palm_monotonic == 1.0


def test_sequence_resets_after_long_gap() -> None:
    core = ReceiverCore(sequence_reset_after_sec=1.0)
    core.process_datagram(make_data(100, timestamp=10.0), 1.0)

    result = core.process_datagram(make_data(5, timestamp=11.5), 2.1)

    assert result.accepted is True
    assert result.sequence_status == SequenceStatus.RESET_AFTER_GAP
    assert core.counters.sequence_resets == 1


def test_source_timestamp_rollback_recovers_app_restart() -> None:
    core = ReceiverCore(source_restart_rollback_sec=0.5)
    core.process_datagram(make_data(100, timestamp=20.0), 1.0)

    result = core.process_datagram(make_data(1, timestamp=0.1), 1.1)

    assert result.accepted is True
    assert result.sequence_status == SequenceStatus.SOURCE_RESTART
    assert core.counters.source_restarts == 1


def test_timeout_is_counted_once_per_transition() -> None:
    core = ReceiverCore()
    core.process_datagram(make_data(1), 1.0)

    assert core.latch_timeout_if_new(1.2, 0.15) is True
    assert core.latch_timeout_if_new(1.3, 0.15) is False
    assert core.counters.timeouts == 1

    core.process_datagram(make_data(2, timestamp=10.1), 1.4)
    assert core.latch_timeout_if_new(1.6, 0.15) is True
    assert core.counters.timeouts == 2
