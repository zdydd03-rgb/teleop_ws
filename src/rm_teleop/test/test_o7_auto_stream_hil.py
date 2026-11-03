"""Automatic HIL tests use mock SDK objects only; no real robot initialization."""
import csv
import json
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from rm_teleop.o7_auto_stream_hil import (
    AutoStreamConfig, AutoStreamRunner, LatestTargetMailbox, triangle_positions,
)
from rm_teleop.o7_single_session_hil import SingleSessionHil, StreamConfig
from rm_teleop.rm_api2_motion_adapter import RmApi2MotionAdapter
from rm_teleop.hil_monitor_data import Capture


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class SimulatedHalt:
    def __init__(self, clock):
        self.clock = clock
        self.stopped = False

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, seconds):
        self.clock.now += seconds
        return self.stopped


def make_adapter():
    adapter = object.__new__(RmApi2MotionAdapter)
    adapter.module = Mock()
    adapter.module.rm_peripheral_read_write_params_t.side_effect = lambda **kw: NS(**kw)
    adapter.arm = Mock()
    adapter.handle = NS(id=123)
    adapter.arm.rm_write_registers.return_value = 1
    adapter.arm.rm_movej_canfd.return_value = 0
    adapter.restore_udp_config_on_shutdown = False
    return adapter


def ready(hil):
    hil.history.clear()  # Replace the synthetic history; never insert old timestamps after new ones.
    now = time.monotonic()
    for i in range(201):
        hil.record(now - 1 + i * .005, True)


def make_node(monkeypatch):
    from rm_teleop.rm65_canfd_high_follow_node import Rm65CanfdHighFollowNode, ControlSummary
    node = object.__new__(Rm65CanfdHighFollowNode)
    node._hil = SingleSessionHil(Mock())
    node._adapter_lock = threading.Lock()
    node._adapter = make_adapter()
    node._connected = node._ik_initialized = node.motion_permitted = True
    node._fault_latched = False
    node._fault_reason = ''
    node._last_summary = ControlSummary(state='ACTIVE', command_sent=True)
    node._stop_event = threading.Event()
    node.publish_hz = 200.0
    node.feedback_freshness_timeout_sec = .05
    node.feedback_disconnect_timeout_sec = .2
    node.allowed_arm_current_statuses = (0, 5)
    node.enable_o7_auto_stream_hil = True
    node.o7_auto_stream_config = AutoStreamConfig(open_pose_confirmed=True)
    node._auto_hil_last_gate = None
    node.get_logger = Mock(return_value=Mock())
    snapshot = NS(received_monotonic=time.monotonic(), joint_enabled=[True]*6,
                  joint_error_codes=[0]*6, arm_error_codes=[], arm_current_status=5)
    node.store = NS(latest=lambda: snapshot)
    worker = Mock()
    monkeypatch.setattr(node._hil, '_run_auto', worker)
    return node, snapshot, worker


def test_stable_active_auto_admission_then_no_restart(monkeypatch):
    node, snapshot, worker = make_node(monkeypatch)
    now = time.monotonic()
    for i in range(20):
        node._hil.record(now - .095 + i * .005, True)
    node._maybe_start_auto_hil()
    worker.assert_not_called()
    assert not node._hil.used
    ready(node._hil)
    node._maybe_start_auto_hil()
    node._hil.wait_closed(Mock())
    assert node._hil.used
    worker.assert_called_once()
    assert worker.call_args.args[0] is node._adapter
    from rm_teleop.rm65_canfd_high_follow_node import ControlSummary
    node._last_summary = ControlSummary(state='READY', command_sent=False)
    node._maybe_start_auto_hil()
    node._last_summary = ControlSummary(state='ACTIVE', command_sent=True)
    snapshot.received_monotonic = time.monotonic()
    ready(node._hil)
    node._maybe_start_auto_hil()
    worker.assert_called_once()
    node._adapter.arm.rm_create_robot_arm.assert_not_called()


@pytest.mark.parametrize('case', ['disabled', 'unconfirmed', 'motion_locked', 'disconnected',
                                  'fault', 'feedback_stale', 'not_active', 'ik_uninitialized'])
def test_auto_admission_requires_all_authorizations(monkeypatch, case):
    node, snapshot, worker = make_node(monkeypatch)
    ready(node._hil)
    if case == 'disabled':
        node.enable_o7_auto_stream_hil = False
    elif case == 'unconfirmed':
        node.o7_auto_stream_config = AutoStreamConfig()
    elif case == 'motion_locked':
        node.motion_permitted = False
    elif case == 'disconnected':
        node._connected = False
    elif case == 'fault':
        node._fault_latched = True
    elif case == 'feedback_stale':
        snapshot.received_monotonic -= .1
    elif case == 'ik_uninitialized':
        node._ik_initialized = False
    else:
        from rm_teleop.rm65_canfd_high_follow_node import ControlSummary
        node._last_summary = ControlSummary(state='READY', command_sent=False)
    node._maybe_start_auto_hil()
    assert not node._hil.used
    worker.assert_not_called()
    node._adapter.arm.rm_write_registers.assert_not_called()


@pytest.mark.parametrize('config', [AutoStreamConfig(hz=5, open_pose_confirmed=True),
    AutoStreamConfig(hz=float('nan'), open_pose_confirmed=True),
    AutoStreamConfig(timeout_sec=61, open_pose_confirmed=True),
    AutoStreamConfig(timeout_sec=0, open_pose_confirmed=True),
    AutoStreamConfig(timeout_sec=float('inf'), open_pose_confirmed=True), AutoStreamConfig()])
def test_invalid_auto_config(config):
    with pytest.raises(ValueError):
        config.validate()


def test_triangle_timing_and_other_six_targets():
    assert triangle_positions(0)[2] == 255
    assert triangle_positions(.5)[2] == 238
    assert triangle_positions(1)[2] == 220
    assert triangle_positions(1.5)[2] == 238
    assert triangle_positions(2)[2] == 255
    for i in range(600):
        target = triangle_positions(i / 10)
        assert 220 <= target[2] <= 255
        assert all(target[j] == 255 for j in (0, 1, 3, 4, 5, 6))


def test_generation_is_10hz_and_stops_at_60s_without_sdk():
    clock = Clock()
    events = []
    adapter = make_adapter()
    runner = AutoStreamRunner(adapter, AutoStreamConfig(open_pose_confirmed=True),
                              events.append, lambda: '', clock=clock)
    runner.halt = SimulatedHalt(clock)
    runner.first_selected.set()
    runner.generate()
    generated = [e for e in events if e['event'] == 'TARGET_GENERATED']
    assert len(generated) == 600
    assert generated[0]['target_position'] == 255
    assert all(e['elapsed_time'] < 60 for e in generated)
    metrics = runner.mailbox.snapshot()
    assert metrics['target_generation_hz'] == pytest.approx(10)
    assert metrics['actual_modbus_write_hz'] is None
    assert metrics['overwritten_target_count'] == 599
    assert metrics['discarded_pending_target_count'] == 1
    assert runner.mailbox.reason == 'timeout_60s_limit'
    assert clock.now == pytest.approx(60)
    adapter.arm.rm_write_registers.assert_not_called()


def test_mailbox_overwrites_never_queues_or_reopens():
    box = LatestTargetMailbox()
    box.put(triangle_positions(0), 0)
    box.put(triangle_positions(.1), .1)
    latest = box.put(triangle_positions(.2), .2)
    assert box.take(timeout=0) == latest
    assert box.take(timeout=0) is None
    assert box.snapshot()['overwritten_target_count'] == 2
    box.close('fault')
    assert box.put((255,) * 7, .3) is None
    assert not box.claim_write(latest, .3, 60)


def run_blocking_integration():
    adapter = make_adapter()
    entered = threading.Event()
    events = []
    sdk_threads = set()
    active = 0
    maximum_active = 0
    def write(params, data):
        nonlocal active, maximum_active
        active += 1
        maximum_active = max(maximum_active, active)
        sdk_threads.add(threading.get_ident())
        assert vars(params) == dict(port=1, device=39, address=0, num=7)
        entered.set()
        threading.Event().wait(.22)
        active -= 1
        return 1
    adapter.arm.rm_write_registers.side_effect = write
    runner = AutoStreamRunner(adapter, AutoStreamConfig(timeout_sec=.68, open_pose_confirmed=True),
                              events.append, lambda: '')
    thread = threading.Thread(target=runner.run)
    thread.start()
    assert entered.wait(1)
    # The main CANFD path remains usable while this mock native call is blocked.
    assert adapter.send_joint_canfd((0,) * 6, follow=True, trajectory_mode=0, radio=0).return_code == 0
    thread.join(3)
    assert not thread.is_alive()
    assert not runner.generator.is_alive()
    return adapter, runner, events, maximum_active, sdk_threads


def test_220ms_sdk_single_writer_and_nonzero_continues():
    adapter, runner, events, maximum_active, sdk_threads = run_blocking_integration()
    generated = [e for e in events if e['event'] == 'TARGET_GENERATED']
    writes = [e for e in events if e['event'] == 'POSITION_WRITE_FINISH']
    assert maximum_active == 1 and len(sdk_threads) == 1
    assert len(writes) >= 2 and len(generated) > len(writes)
    assert writes[0]['target_positions'] == [255] * 7
    assert all(e['return_code'] == e['modbus_return_code'] == 1 for e in writes)
    assert all(e['modbus_duration_ms'] >= 200 for e in writes)
    assert any(b['generation_sequence'] > a['generation_sequence'] + 1 for a, b in zip(writes, writes[1:]))
    final = events[-1]
    assert final['event'] == 'AUTO_STREAM_FINISH'
    assert final['actual_motion'] == 'NOT AUTOMATICALLY VERIFIED'
    assert final['sdk_acknowledged_success_count'] == 0
    assert final['sdk_unconfirmed_command_count'] == len(writes)
    assert final['overwritten_target_count'] > 0
    assert final['target_generation_hz'] > final['actual_modbus_write_hz']
    stop = next(e for e in events if e['event'] == 'AUTO_STREAM_STOP')
    assert all(e['start_timestamp'] <= stop['timestamp_ns'] for e in writes)
    assert runner.mailbox.closed
    adapter.module.RoboticArm.assert_not_called()
    adapter.arm.rm_create_robot_arm.assert_not_called()
    adapter.arm.rm_read_multiple_input_registers.assert_not_called()


@pytest.mark.parametrize('reason', ['control_not_ACTIVE_sending', 'FAULT_LATCHED:test',
                                   'robot_feedback_stale', 'robot_disconnected', 'node_closing'])
def test_inflight_fault_closes_mailbox_no_next_write(reason):
    adapter = make_adapter()
    entered, release = threading.Event(), threading.Event()
    unsafe = ['']
    events = []
    def write(*args):
        entered.set()
        assert release.wait(2)
        return 1
    adapter.arm.rm_write_registers.side_effect = write
    runner = AutoStreamRunner(adapter, AutoStreamConfig(open_pose_confirmed=True),
                              events.append, lambda: unsafe[0])
    worker = threading.Thread(target=runner.run)
    worker.start()
    assert entered.wait(1)
    unsafe[0] = reason
    assert runner.halt.wait(1)
    assert worker.is_alive()  # Safe cancellation never pretends native IO was terminated.
    assert runner.mailbox.closed and runner.mailbox.reason == reason
    release.set()
    worker.join(1)
    assert not worker.is_alive()
    adapter.arm.rm_write_registers.assert_called_once()
    assert events[-1]['reason'] == reason


def test_sdk_exception_aborts_without_retry():
    adapter = make_adapter()
    adapter.arm.rm_write_registers.side_effect = RuntimeError('mock native exception')
    events = []
    runner = AutoStreamRunner(adapter, AutoStreamConfig(open_pose_confirmed=True), events.append, lambda: '')
    runner.run()
    adapter.arm.rm_write_registers.assert_called_once()
    assert events[-1]['reason'] == 'sdk_exception'
    assert events[-1]['sdk_exception_count'] == 1
    assert events[-1]['sdk_unconfirmed_command_count'] == 0
    assert not runner.generator.is_alive()


def test_timeout_prohibits_following_write_while_native_call_is_blocked():
    clock = Clock()
    entered, release = threading.Event(), threading.Event()
    adapter = make_adapter()
    events = []
    def write(*args):
        entered.set()
        assert release.wait(2)
        return 1
    adapter.arm.rm_write_registers.side_effect = write
    runner = AutoStreamRunner(adapter, AutoStreamConfig(open_pose_confirmed=True),
                              events.append, lambda: '', clock=clock)
    worker = threading.Thread(target=runner.run)
    worker.start()
    assert entered.wait(1)
    clock.now = 60
    assert runner.halt.wait(1)
    assert runner.mailbox.reason == 'timeout_60s_limit'
    release.set()
    worker.join(1)
    assert not worker.is_alive()
    adapter.arm.rm_write_registers.assert_called_once()
    assert events[-1]['position_write_count'] == 1


def test_auto_and_legacy_modes_share_one_acceptance(monkeypatch):
    for auto_first in (True, False):
        hil = SingleSessionHil(Mock())
        ready(hil)
        monkeypatch.setattr(hil, '_run_auto', Mock())
        monkeypatch.setattr(hil, '_run_stream', Mock())
        args = dict(auto_config=AutoStreamConfig(open_pose_confirmed=True))
        legacy = dict(stream_config=StreamConfig(open_pose_confirmed=True))
        assert hil.trigger(True, make_adapter(), safety=lambda: '', **(args if auto_first else legacy))[0]
        hil.wait_closed(Mock())
        assert not hil.trigger(True, make_adapter(), safety=lambda: '', **(legacy if auto_first else args))[0]


def test_auto_lifecycle_waits_for_writer_and_generator_before_disconnect():
    hil = SingleSessionHil(Mock())
    ready(hil)
    adapter = make_adapter()
    arm = adapter.arm
    entered, release, disconnected = threading.Event(), threading.Event(), threading.Event()
    def write(*args):
        entered.set()
        assert release.wait(2)
        return 1
    arm.rm_write_registers.side_effect = write
    def healthy():
        hil.record(time.monotonic(), True)
        return ''
    assert hil.trigger(True, adapter, auto_config=AutoStreamConfig(open_pose_confirmed=True), safety=healthy)[0]
    assert entered.wait(1)
    hil.record(time.monotonic(), False)
    hil.record(time.monotonic(), True)  # Cancellation remains latched after re-entering ACTIVE.
    def close():
        hil.begin_close()
        hil.wait_closed(Mock())
        adapter.disconnect()
        disconnected.set()
    closer = threading.Thread(target=close)
    closer.start()
    assert not disconnected.wait(.02)
    arm.rm_delete_robot_arm.assert_not_called()
    release.set()
    closer.join(1)
    assert disconnected.is_set()
    arm.rm_write_registers.assert_called_once()
    arm.rm_delete_robot_arm.assert_called_once()
    assert hil.used


def test_auto_events_csv_and_reported_rates(tmp_path):
    _, _, events, _, _ = run_blocking_integration()
    capture = Capture()
    for e in events:
        capture.event(e, e['timestamp_ns'])
    summary = capture.save(tmp_path)
    auto = summary['auto_stream']
    assert auto['start']['event'] == 'AUTO_STREAM_START'
    assert auto['stop']['event'] == 'AUTO_STREAM_STOP'
    assert auto['finish']['event'] == 'AUTO_STREAM_FINISH'
    assert auto['node_reported_metrics']['target_generation_hz'] > auto['node_reported_metrics']['actual_modbus_write_hz']
    assert summary['stream']['completion']['event'] == 'AUTO_STREAM_FINISH'
    assert capture.end == events[-1]['timestamp_ns']
    rows = list(csv.DictReader((tmp_path / 'events.csv').open()))
    assert any(r['event'] == 'TARGET_GENERATED' and r['generation_sequence'] for r in rows)
    assert 'auto_stream' in json.loads((tmp_path / 'summary.json').read_text())
    csv_summary = list(csv.DictReader((tmp_path / 'summary.csv').open()))
    assert all(r['auto_stream_total_configured_target_generation_hz'] == '10.0' for r in csv_summary)


def test_auto_canfd_gap_latches_cancel_without_changing_manual_policy():
    for automatic in (True, False):
        hil = SingleSessionHil(Mock())
        hil.streaming = True
        hil.auto_streaming = automatic
        now = time.monotonic()
        hil.record(now - .02, True)
        hil.record(now, True)
        assert hil.stream_cancel.is_set() == automatic


def test_delayed_generation_skips_slots_and_reports_observed_rate():
    clock = Clock()
    events = []
    def emit(event):
        events.append(event)
        if event['event'] == 'TARGET_GENERATED' and event['generation_sequence'] == 2:
            clock.now += .35
    runner = AutoStreamRunner(make_adapter(), AutoStreamConfig(timeout_sec=1, open_pose_confirmed=True),
                              emit, lambda: '', clock=clock)
    runner.halt = SimulatedHalt(clock)
    runner.first_selected.set()
    runner.generate()
    metrics = runner.mailbox.snapshot()
    assert metrics['skipped_generation_slots'] == 2
    assert metrics['target_generation_count'] == 8
    assert metrics['target_generation_hz'] < 10
    targets = [e for e in events if e['event'] == 'TARGET_GENERATED']
    assert targets[2]['generated_monotonic'] == pytest.approx(.45)
    assert targets[2]['target_positions'] == list(triangle_positions(.45))


def test_auto_defaults_and_passive_monitor_wiring():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    config = (root / 'config/high_follow.yaml').read_text()
    launch = (root / 'launch/teleop_high_follow.launch.py').read_text()
    node = (root / 'rm_teleop/rm65_canfd_high_follow_node.py').read_text()
    monitor = (root / 'rm_teleop/single_session_hil_monitor.py').read_text()
    assert 'enable_o7_auto_stream_hil: false' in config
    assert 'o7_auto_stream_open_pose_confirmed: false' in config
    assert 'o7_auto_stream_hz: 10.0' in config
    assert 'o7_auto_stream_timeout_sec: 60.0' in config
    assert 'DeclareLaunchArgument("enable_o7_auto_stream_hil", default_value="false")' in launch
    assert 'if self.enable_o7_auto_stream_hil:' in node
    assert 'self.create_timer(.05, self._maybe_start_auto_hil)' in node
    assert '--observe-auto-stream' in monitor
    assert 'execute = options.execute_injection or options.execute_stream' in monitor
    assert 'rm_api2' not in monitor
