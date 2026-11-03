"""No real SDK construction, connection, or hardware IO in these tests."""
import csv
import json
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from rm_teleop.o7_single_session_hil import (
    SingleSessionHil, StreamConfig, run_position_stream, POSITION_A, POSITION_B, ACTUAL_MOTION,
)
from rm_teleop.rm_api2_motion_adapter import RmApi2MotionAdapter
from rm_teleop.hil_monitor_data import Capture


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def wait(self, seconds):
        self.now += seconds


CONFIRMED = StreamConfig(open_pose_confirmed=True)


def make_adapter():
    adapter = object.__new__(RmApi2MotionAdapter)
    adapter.module = Mock()
    adapter.module.rm_peripheral_read_write_params_t.side_effect = lambda **kw: NS(**kw)
    adapter.arm = Mock()
    adapter.handle = NS(id=123)
    adapter.arm.rm_read_multiple_input_registers.side_effect = AssertionError("O7 reads forbidden")
    adapter.arm.rm_read_input_registers.side_effect = AssertionError("O7 reads forbidden")
    adapter.arm.rm_write_registers.return_value = 0
    return adapter


def ready(hil):
    now = time.monotonic()
    for i in range(201):
        hil.record(now - 1 + i * .005, True)


def run(adapter, *, config=CONFIRMED, guard=lambda: '', clock=None):
    clock = clock or Clock()
    events = []
    run_position_stream(adapter, config, events.append, guard, clock.wait, clock=clock)
    return events


@pytest.mark.parametrize('change', [dict(hz=0), dict(hz=.5), dict(hz=5), dict(hz=10),
    dict(hz=float('nan')), dict(hz=float('inf')), dict(hz=True), dict(duration_sec=1),
    dict(duration_sec=11), dict(duration_sec=float('nan')), dict(joint=0), dict(joint=2.0),
    dict(open_pose_confirmed=False), dict(open_pose_confirmed=1)])
def test_parameter_rejection_before_sdk(change):
    config = StreamConfig(**dict(dict(open_pose_confirmed=True), **change))
    adapter = make_adapter()
    hil = SingleSessionHil(Mock())
    ready(hil)
    accepted, reason = hil.trigger(True, adapter, stream_config=config, safety=lambda: '')
    assert not accepted and 'o7_stream_' in reason
    assert not hil.used
    adapter.arm.rm_read_multiple_input_registers.assert_not_called()
    adapter.arm.rm_write_registers.assert_not_called()


def test_write_only_fixed_targets_and_existing_handle():
    adapter = make_adapter()
    original_arm, original_handle = adapter.arm, adapter.handle
    events = run(adapter)
    starts = [e for e in events if e['event'] == 'POSITION_WRITE_START']
    assert len(starts) == 10
    assert [e['elapsed_time'] for e in starts] == pytest.approx(list(range(10)))
    for i, call in enumerate(adapter.arm.rm_write_registers.call_args_list):
        params, data = call.args
        expected = POSITION_A if i % 2 == 0 else POSITION_B
        assert vars(params) == dict(port=1, device=39, address=0, num=7)
        assert data == [byte for v in expected for byte in (0, v)]
        assert starts[i]['target_positions'] == list(expected)
        assert all(expected[j] == 255 for j in (0, 1, 3, 4, 5, 6))
    assert events[-1]['schedule_completed'] and events[-1]['position_write_count'] == 10
    assert events[-1]['elapsed_time'] == pytest.approx(10)
    assert events[-1]['last_target_positions'] == list(POSITION_B)
    assert events[-1]['sdk_acknowledged_success_count'] == 10
    assert events[-1]['sdk_unconfirmed_command_count'] == 0
    assert events[-1]['actual_motion'] == ACTUAL_MOTION
    assert 'success' not in events[-1] and 'restored_confirmed' not in events[-1]
    adapter.arm.rm_read_multiple_input_registers.assert_not_called()
    adapter.arm.rm_read_input_registers.assert_not_called()
    assert not any('READ' in e['event'] for e in events)
    assert adapter.arm is original_arm and adapter.handle is original_handle
    adapter.module.RoboticArm.assert_not_called()
    adapter.arm.rm_create_robot_arm.assert_not_called()
    assert not hasattr(RmApi2MotionAdapter, 'read_o7_hil_positions')


@pytest.mark.parametrize('code', [1, -1, -2, -3, -4])
def test_nonzero_results_never_stop_or_retry(code):
    adapter = make_adapter()
    adapter.arm.rm_write_registers.return_value = code
    events = run(adapter)
    assert adapter.arm.rm_write_registers.call_count == 10
    finishes = [e for e in events if e['event'] == 'POSITION_WRITE_FINISH']
    assert [e['return_code'] for e in finishes] == [code] * 10
    assert [e['modbus_return_code'] for e in finishes] == [code] * 10
    assert [e['consecutive_failures'] for e in finishes] == list(range(1, 11))
    assert not any(e['sdk_acknowledged'] for e in finishes)
    assert [e['target_position'] for e in finishes] == [220, 255] * 5
    assert events[-1]['schedule_completed'] and events[-1]['elapsed_time'] == pytest.approx(10)
    assert events[-1]['sdk_unconfirmed_command_count'] == 10
    assert events[-1]['sdk_acknowledged_success_count'] == events[-1]['sdk_exception_count'] == 0
    assert events[-1]['actual_motion'] == 'NOT AUTOMATICALLY VERIFIED'


def test_mixed_codes_are_reported_separately_without_stopping():
    adapter = make_adapter()
    codes = [1, 0, -2, 0, 1, 1, 1, 1, 0, 1]
    adapter.arm.rm_write_registers.side_effect = codes
    events = run(adapter)
    results = [e for e in events if e['event'] == 'POSITION_WRITE_FINISH']
    assert [e['return_code'] for e in results] == codes
    assert [e['consecutive_failures'] for e in results][:5] == [1, 0, 1, 0, 1]
    assert events[-1]['sdk_unconfirmed_command_count'] == 7
    assert events[-1]['sdk_acknowledged_success_count'] == 3
    assert events[-1]['schedule_completed']


def test_native_exception_aborts_after_first_call():
    adapter = make_adapter()
    adapter.arm.rm_write_registers.side_effect = RuntimeError('mock exception')
    events = run(adapter)
    adapter.arm.rm_write_registers.assert_called_once()
    failed = next(e for e in events if e['event'] == 'POSITION_WRITE_FINISH')
    assert failed['modbus_return_code'] is None
    assert failed['consecutive_failures'] == 1
    assert 'mock exception' in failed['error']
    assert any(e['event'] == 'STREAM_EXCEPTION' for e in events)
    assert events[-1]['sdk_exception_count'] == 1


@pytest.mark.parametrize('reason', ['FAULT_LATCHED', 'robot_feedback_stale', 'robot_disconnected',
                                   'control_not_ACTIVE_sending'])
def test_mainline_failure_prevents_next_command_and_restore(reason):
    adapter = make_adapter()
    fault = ['']
    def write(*args):
        fault[0] = reason
        return 0
    adapter.arm.rm_write_registers.side_effect = write
    events = run(adapter, guard=lambda: fault[0])
    adapter.arm.rm_write_registers.assert_called_once()
    assert events[-1]['reason'] == reason and not events[-1]['schedule_completed']


def test_guard_checked_while_waiting_not_only_at_send():
    adapter = make_adapter()
    clock = Clock()
    events = run(adapter, clock=clock, guard=lambda: 'unsafe' if clock.now >= 1000.2 else '')
    adapter.arm.rm_write_registers.assert_called_once()
    assert events[-1]['elapsed_time'] < .3
    assert events[-1]['reason'] == 'unsafe'


def test_slow_native_calls_skip_slots_no_catchup_queue():
    adapter = make_adapter()
    clock = Clock()
    def slow(*args):
        clock.now += 1.4
        return 0
    adapter.arm.rm_write_registers.side_effect = slow
    events = run(adapter, clock=clock)
    starts = [e for e in events if e['event'] == 'POSITION_WRITE_START']
    assert all(b['elapsed_time'] - a['elapsed_time'] >= 1.99 for a, b in zip(starts, starts[1:]))
    assert all(e['elapsed_time'] < 10 for e in starts)
    assert events[-1]['schedule_completed']
    assert adapter.arm.rm_write_registers.call_count == 5


@pytest.mark.parametrize('duration', [2.0, 3.5, 10.0])
def test_deadline_no_extra_restore_write(duration):
    adapter = make_adapter()
    events = run(adapter, config=StreamConfig(duration_sec=duration, open_pose_confirmed=True))
    starts = [e for e in events if e['event'] == 'POSITION_WRITE_START']
    assert all(e['elapsed_time'] < duration for e in starts)
    assert len(starts) == len([t for t in range(10) if t < duration])
    assert events[-1]['elapsed_time'] == pytest.approx(duration)
    assert events[-1]['last_target_positions'] == starts[-1]['target_positions']


def test_disabled_and_mutual_exclusion_with_legacy_hil():
    hil = SingleSessionHil(Mock())
    ready(hil)
    adapter = make_adapter()
    assert not hil.trigger(False, adapter, stream_config=CONFIRMED, safety=lambda: '')[0]
    assert hil.trigger(True, adapter)[0]
    hil.wait_closed(Mock())
    assert not hil.trigger(True, adapter, stream_config=CONFIRMED, safety=lambda: '')[0]
    adapter.arm.rm_read_multiple_input_registers.assert_not_called()


def test_stream_thread_cancel_latches_and_disconnect_waits():
    events = []
    adapter = make_adapter()
    hil = SingleSessionHil(events.append)
    ready(hil)
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    main_thread = threading.get_ident()
    def write(params, data):
        assert threading.get_ident() != main_thread
        entered.set()
        assert release.wait(2)
        return 1
    arm = adapter.arm
    arm.rm_write_registers.side_effect = write
    adapter.restore_udp_config_on_shutdown = False
    arm.rm_movej_canfd.return_value = 0
    assert hil.trigger(True, adapter, stream_config=CONFIRMED, safety=lambda: '')[0]
    assert entered.wait(1)
    # A transient non-ACTIVE cycle remains latched even if the arm recovers before write returns.
    hil.record(time.monotonic(), False)
    hil.record(time.monotonic(), True)
    assert hil.stream_cancel.is_set()
    assert not hil.trigger(True, adapter)[0]
    assert adapter.send_joint_canfd((0,) * 6, follow=True, trajectory_mode=0, radio=0).return_code == 0
    def close():
        hil.begin_close()
        hil.wait_closed(Mock())
        adapter.disconnect()
        closed.set()
    closer = threading.Thread(target=close)
    closer.start()
    assert not closed.wait(.02)
    arm.rm_delete_robot_arm.assert_not_called()
    release.set()
    closer.join(1)
    assert closed.is_set()
    arm.rm_write_registers.assert_called_once()
    arm.rm_read_multiple_input_registers.assert_not_called()
    arm.rm_delete_robot_arm.assert_called_once()
    assert events[-1]['event'] == 'STREAM_FINISH' and not events[-1]['schedule_completed']


def test_stream_events_csv_and_summary(tmp_path):
    adapter = make_adapter()
    adapter.arm.rm_write_registers.return_value = 1
    events = run(adapter)
    cap = Capture()
    for event in events:
        cap.event(event, event['timestamp_ns'] + 100)
    summary = cap.save(tmp_path)
    assert cap.start == events[0]['timestamp_ns'] and cap.end == events[-1]['timestamp_ns']
    assert summary['stream']['write_results'] == 10
    assert summary['stream']['return_codes'] == {'1': 10}
    assert summary['stream']['max_consecutive_failures'] == 10
    assert summary['stream']['completion']['schedule_completed']
    assert summary['stream']['sdk_acknowledged_success_count'] == 0
    assert summary['stream']['sdk_exception_count'] == 0
    assert summary['stream']['actual_motion'] == ACTUAL_MOTION
    rows = list(csv.DictReader((tmp_path / 'events.csv').open()))
    writes = [r for r in rows if r['event'] == 'POSITION_WRITE_FINISH']
    assert [int(r['command_sequence']) for r in writes] == list(range(1, 11))
    assert all(int(r['target_position']) in (220, 255) for r in writes)
    assert all(float(r['modbus_duration_ms']) >= 0 for r in writes)
    assert json.loads((tmp_path / 'summary.json').read_text())['stream']['sdk_unconfirmed_command_count'] == 10
    samples = list(csv.DictReader((tmp_path / 'samples.csv').open()))
    assert all(r['phase'] == 'INJECTION' for r in samples)


def test_node_stream_service_reuses_guard_and_shared_owner():
    from rm_teleop.rm65_canfd_high_follow_node import Rm65CanfdHighFollowNode, ControlSummary
    node = object.__new__(Rm65CanfdHighFollowNode)
    node._adapter_lock = threading.Lock()
    node._adapter = make_adapter()
    node._connected = node._ik_initialized = True
    node._fault_latched = False
    node._fault_reason = ''
    node.publish_hz = 200.0
    node.motion_permitted = True
    node.enable_o7_stream_hil = False
    node.enable_o7_single_session_hil = False
    node.o7_stream_config = CONFIRMED
    node._last_summary = ControlSummary(state='ACTIVE', command_sent=True)
    node._stop_event = threading.Event()
    node.feedback_freshness_timeout_sec = .05
    node.feedback_disconnect_timeout_sec = .2
    node.allowed_arm_current_statuses = (0, 5)
    snapshot = NS(received_monotonic=time.monotonic(), joint_enabled=[True]*6,
                  joint_error_codes=[0]*6, arm_error_codes=[], arm_current_status=5)
    node.store = NS(latest=lambda: snapshot)
    node._hil = SingleSessionHil(Mock())
    ready(node._hil)
    response = node._start_stream_hil(None, NS())
    assert not response.success and response.message == 'enable_o7_stream_hil=false'
    node.enable_o7_stream_hil = True
    snapshot.received_monotonic -= .1
    response = node._start_stream_hil(None, NS())
    assert not response.success and 'robot_feedback_stale' in response.message
    snapshot.received_monotonic = time.monotonic()
    node._fault_latched = True
    assert 'FAULT_LATCHED' in node._start_stream_hil(None, NS()).message
    node._adapter.arm.rm_read_multiple_input_registers.assert_not_called()
    node._adapter.arm.rm_write_registers.assert_not_called()


def test_configuration_and_monitor_wiring():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    node = (root / 'rm_teleop/rm65_canfd_high_follow_node.py').read_text()
    config = (root / 'config/high_follow.yaml').read_text()
    launch = (root / 'launch/teleop_high_follow.launch.py').read_text()
    monitor = (root / 'rm_teleop/single_session_hil_monitor.py').read_text()
    assert 'self.declare_parameter("enable_o7_stream_hil", False)' in node
    assert 'enable_o7_stream_hil: false' in config
    assert 'DeclareLaunchArgument("enable_o7_stream_hil", default_value="false")' in launch
    assert "--execute-stream" in monitor and 'add_mutually_exclusive_group' in monitor
    assert 'rm_api2' not in monitor and 'RoboticArm' not in monitor
    assert '.rm_write_registers(' not in node and '.rm_read_multiple_input_registers(' not in node


def test_fault_during_start_publication_cancels_before_native_call():
    adapter = make_adapter()
    fault = ['']
    events = []
    clock = Clock()
    def emit(event):
        events.append(event)
        if event['event'] == 'POSITION_WRITE_START':
            fault[0] = 'FAULT_LATCHED'
    run_position_stream(adapter, CONFIRMED, emit, lambda: fault[0], clock.wait, clock=clock)
    adapter.arm.rm_write_registers.assert_not_called()
    assert any(e['event'] == 'POSITION_WRITE_CANCELLED' for e in events)
    assert events[-1]['position_write_count'] == 0
    assert events[-1]['reason'] == 'FAULT_LATCHED'


def test_blocking_call_overrun_never_starts_another_command():
    adapter = make_adapter()
    clock = Clock()
    def write(params, data):
        clock.now += 11
        return 1
    adapter.arm.rm_write_registers.side_effect = write
    events = run(adapter, clock=clock)
    adapter.arm.rm_write_registers.assert_called_once()
    assert events[-1]['sdk_unconfirmed_command_count'] == 1
    assert events[-1]['last_target_positions'] == list(POSITION_A)
    assert events[-1]['position_write_count'] == 1


def test_open_pose_confirmation_required_before_any_sdk_call():
    adapter = make_adapter()
    hil = SingleSessionHil(Mock())
    ready(hil)
    accepted, reason = hil.trigger(True, adapter, stream_config=StreamConfig(), safety=lambda: '')
    assert not accepted and 'o7_stream_open_pose_confirmed=true required' in reason
    adapter.arm.rm_write_registers.assert_not_called()


def test_summary_keeps_exceptions_separate_from_nonzero_results(tmp_path):
    adapter = make_adapter()
    adapter.arm.rm_write_registers.side_effect = [1, 0, RuntimeError('native failure')]
    events = run(adapter)
    capture = Capture()
    for e in events:
        capture.event(e, e['timestamp_ns'])
    summary = capture.save(tmp_path)
    stream = summary['stream']
    assert stream['sdk_acknowledged_success_count'] == 1
    assert stream['sdk_unconfirmed_command_count'] == 1
    assert stream['sdk_exception_count'] == 1
    assert stream['write_results'] == 3
    assert stream['actual_motion'] == 'NOT AUTOMATICALLY VERIFIED'
    assert not stream['completion']['schedule_completed']
    rows = list(csv.DictReader((tmp_path / 'summary.csv').open()))
    assert all(r['stream_total_sdk_exception_count'] == '1' for r in rows)
    assert all(r['actual_motion'] == 'NOT AUTOMATICALLY VERIFIED' for r in rows)
    finished = [e for e in events if e['event'] == 'POSITION_WRITE_FINISH']
    assert [e['return_code'] for e in finished] == [1, 0, None]
    assert all(e['end_timestamp'] >= e['start_timestamp'] for e in finished)
    assert all(e['duration_ms'] == e['modbus_duration_ms'] >= 0 for e in finished)
    assert all(e['sequence'] == e['command_sequence'] for e in finished)


def test_late_wakeup_selects_latest_target_not_old_slot():
    adapter = make_adapter()
    clock = Clock()
    events = []
    delayed = False
    def wait(seconds):
        nonlocal delayed
        clock.wait(seconds)
        if not delayed and clock.now > 1000:
            clock.now = 1002.2
            delayed = True
    run_position_stream(adapter, CONFIRMED, events.append, lambda: '', wait, clock=clock)
    starts = [e for e in events if e['event'] == 'POSITION_WRITE_START']
    assert starts[1]['elapsed_time'] == pytest.approx(2.2)
    assert starts[1]['target_position'] == 220  # slot 2, not stale slot 1 (255)
    assert starts[2]['elapsed_time'] == pytest.approx(3)
    assert len(starts) == 9


def test_complete_stream_has_only_write_sdk_calls():
    adapter = make_adapter()
    run(adapter)
    assert {call[0] for call in adapter.arm.mock_calls} == {'rm_write_registers'}
