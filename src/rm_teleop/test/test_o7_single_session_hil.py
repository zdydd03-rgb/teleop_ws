import csv
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from rm_teleop.o7_single_session_hil import SingleSessionHil
from rm_teleop.rm_api2_motion_adapter import RmApi2MotionAdapter
from rm_teleop.hil_monitor_data import Capture, CONTROL_FIELDS, diagnostic_level, distribution


def ready(hil):
    now = time.monotonic()
    for i in range(201):
        hil.record(now - 1 + i * .005, True)


def adapter_mock(code=0):
    adapter = object.__new__(RmApi2MotionAdapter)
    adapter.arm = Mock()
    adapter.handle = NS(id=42)
    adapter.module = NS(rm_peripheral_read_write_params_t=lambda **kw: NS(**kw))
    adapter.arm.rm_write_registers.return_value = code
    return adapter


@pytest.mark.parametrize('enabled,reason', [(False, ''), (True, 'api2_not_connected'),
    (True, 'control_not_ACTIVE_sending'), (True, 'robot_feedback_stale'),
    (True, 'FAULT_LATCHED:test'), (True, 'ik_not_initialized'), (True, 'motion_not_permitted')])
def test_rejections(enabled, reason):
    hil = SingleSessionHil(Mock())
    ready(hil)
    adapter = adapter_mock()
    assert not hil.trigger(enabled, adapter, reason)[0]
    adapter.arm.rm_write_registers.assert_not_called()


def test_requires_sustained_stream():
    hil = SingleSessionHil(Mock())
    assert not hil.trigger(True, adapter_mock())[0]
    ready(hil)
    hil.record(time.monotonic(), False)
    assert not hil.trigger(True, adapter_mock())[0]


@pytest.mark.parametrize('code', [0, 1, -2])
def test_existing_arm_worker_once_and_concurrent_canfd(code):
    events = []
    entered, release = threading.Event(), threading.Event()
    adapter = adapter_mock(code)
    main_id = threading.get_ident()
    def write(params, data):
        assert threading.get_ident() != main_id
        assert vars(params) == dict(port=1, address=14, device=39, num=7)
        assert data == [0, 25] * 7
        entered.set()
        assert release.wait(2)
        return code
    adapter.arm.rm_write_registers.side_effect = write
    adapter.arm.rm_movej_canfd.return_value = 0
    hil = SingleSessionHil(events.append)
    ready(hil)
    assert hil.trigger(True, adapter)[0]
    assert entered.wait(1)
    results = []
    threads = [threading.Thread(target=lambda: results.append(hil.trigger(True, adapter)[0])) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert results == [False] * 8
    assert adapter.send_joint_canfd((0,) * 6, follow=True, trajectory_mode=0, radio=0).return_code == 0
    # close cannot free the handle while the SDK call is running.
    hil.begin_close()
    closed = threading.Event()
    def close():
        hil.wait_closed(Mock())
        adapter.disconnect()
        closed.set()
    adapter.restore_udp_config_on_shutdown = False
    closer = threading.Thread(target=close)
    closer.start()
    assert not closed.wait(.02)
    adapter.arm.rm_delete_robot_arm.assert_not_called()
    arm = adapter.arm
    release.set()
    closer.join(1)
    assert closed.is_set()
    arm.rm_delete_robot_arm.assert_called_once()
    arm.rm_create_robot_arm.assert_not_called()
    assert [e['event'] for e in events] == ['MODBUS_START', 'MODBUS_FINISH']
    assert events[-1]['return_code'] == code
    assert events[-1]['duration_ms'] >= 20
    assert events[-1]['modbus_end_timestamp'] >= events[-1]['modbus_start_timestamp']
    assert not hil.trigger(True, adapter)[0]


def test_exception_consumes_attempt_and_reports():
    events = []
    adapter = adapter_mock()
    adapter.arm.rm_write_registers.side_effect = RuntimeError('mock failure')
    hil = SingleSessionHil(events.append)
    ready(hil)
    assert hil.trigger(True, adapter)[0]
    hil.wait_closed(Mock())
    assert events[-1]['event'] == 'MODBUS_EXCEPTION'
    assert 'mock failure' in events[-1]['error']
    assert events[-1]['return_code'] is None
    assert not hil.trigger(True, adapter)[0]


def test_node_gate_uses_real_safety_state():
    # Construct without __init__: never construct a real adapter or connect a robot.
    from rm_teleop.rm65_canfd_high_follow_node import Rm65CanfdHighFollowNode, ControlSummary
    node = object.__new__(Rm65CanfdHighFollowNode)
    node._adapter_lock = threading.Lock()
    node._adapter = adapter_mock()
    node._connected = True
    node._ik_initialized = True
    node._fault_latched = False
    node._fault_reason = ''
    node.publish_hz = 200.0
    node.motion_permitted = True
    node.enable_o7_single_session_hil = True
    node._last_summary = ControlSummary(state='ACTIVE', command_sent=True)
    node._stop_event = threading.Event()
    node.feedback_freshness_timeout_sec = .05
    node.feedback_disconnect_timeout_sec = .2
    node.allowed_arm_current_statuses = (0, 5)
    node._hil = SingleSessionHil(Mock())
    snapshot = NS(received_monotonic=time.monotonic(), joint_enabled=[True]*6,
                  joint_error_codes=[0]*6, arm_error_codes=[], arm_current_status=5)
    node.store = NS(latest=lambda: snapshot)
    cases = [('_connected', False, 'robot_disconnected'),
             ('_ik_initialized', False, 'ik_not_initialized'),
             ('motion_permitted', False, 'motion_not_permitted'),
             ('_fault_latched', True, 'FAULT_LATCHED'),
             ('_last_summary', ControlSummary(state='READY'), 'control_not_ACTIVE')]
    for attr, value, reason in cases:
        before = getattr(node, attr)
        setattr(node, attr, value)
        response = node._trigger_hil_once(None, NS())
        assert not response.success and reason in response.message
        setattr(node, attr, before)
    snapshot.received_monotonic -= .1
    assert 'robot_feedback_stale' in node._trigger_hil_once(None, NS()).message
    node._adapter.arm.rm_write_registers.assert_not_called()


def test_csv_phases_failures_and_statistics(tmp_path):
    cap = Capture()
    msg = NS(**{k: 0 for k in CONTROL_FIELDS})
    msg.state, msg.reason = 'ACTIVE', 'command_sent'
    msg.command_sent = msg.command_attempted = True
    msg.command_interval_sec = .005
    msg.command_duration_ms = .2
    msg.feedback_age_sec = .001
    start = 20_000_000_000
    end = start + 80_000_000
    cap.event(dict(event='MODBUS_START', timestamp_ns=start), start)
    cap.event(dict(event='MODBUS_FINISH', timestamp_ns=end, return_code=0, duration_ms=80), end)
    for stamp in [start - 10_000_000_000, start - 1, start, end, end + 10_000_000_000]:
        cap.control(msg, stamp, stamp + 123)
    msg.command_sent = False
    msg.command_interval_sec = 999
    msg.fault_latched = True
    msg.feedback_age_sec = .06
    msg.reason = 'robot_feedback_stale'
    cap.control(msg, start + 1, start + 1)
    summary = cap.save(tmp_path)
    assert summary['complete_window']
    injection = summary['phases']['INJECTION']
    assert injection['sent'] == 2 and injection['send_failures'] == 1
    assert injection['canfd_interval_sec']['max'] == .005
    assert injection['not_sent_reasons'] == {'robot_feedback_stale': 1}
    assert injection['feedback_invalid_samples'] == 1
    rows = list(csv.DictReader((tmp_path / 'samples.csv').open()))
    assert {r['phase'] for r in rows} == {'BASELINE', 'INJECTION', 'POST'}
    assert len(rows) == 8
    assert json.loads((tmp_path / 'summary.json').read_text())['events'][1]['duration_ms'] == 80
    assert diagnostic_level(b'\x00') == 0
    assert diagnostic_level(b'\x02') == 2
    assert diagnostic_level(1) == 1
    assert distribution([1, 2, 3, 4])['p95'] == pytest.approx(3.85)


def test_single_session_source_invariants():
    root = Path(__file__).resolve().parents[1] / 'rm_teleop'
    node = (root / 'rm65_canfd_high_follow_node.py').read_text()
    adapter = (root / 'rm_api2_motion_adapter.py').read_text()
    assert '.rm_write_registers(' not in node
    assert 'RoboticArm(' not in adapter and 'rm_create_robot_arm(' not in adapter
    assert 'self.declare_parameter("enable_o7_single_session_hil", False)' in node
    assert node.index('self._hil.wait_closed', node.index('def destroy_node')) < node.index('adapter.disconnect()', node.index('def destroy_node'))
    assert 'RM_TRIPLE_MODE_E' in (root / 'rm_api2_adapter.py').read_text()


def test_shutdown_stops_before_wait_then_disconnect(monkeypatch):
    from rclpy.node import Node
    from rm_teleop.rm65_canfd_high_follow_node import Rm65CanfdHighFollowNode
    monkeypatch.setattr(Node, 'destroy_node', lambda self: True)
    node = object.__new__(Rm65CanfdHighFollowNode)
    node._stop_event = threading.Event()
    node._adapter_lock = threading.Lock()
    node._adapter = adapter_mock()
    node._connected = node._ik_initialized = True
    node.stop_on_shutdown = node._motion_active = True
    node.get_logger = lambda: Mock()
    node.csv_logger = Mock()
    node._connection_thread = NS(is_alive=lambda: False)
    entered, release = threading.Event(), threading.Event()
    arm = node._adapter.arm
    node._adapter.restore_udp_config_on_shutdown = False
    arm.rm_write_registers.side_effect = lambda *a: (entered.set(), release.wait(2), 0)[-1]
    node._hil = SingleSessionHil(Mock())
    ready(node._hil)
    assert node._hil.trigger(True, node._adapter)[0]
    assert entered.wait(1)
    shutdown = threading.Thread(target=node.destroy_node)
    shutdown.start()
    deadline = time.monotonic() + 1
    while not arm.rm_set_arm_stop.called and time.monotonic() < deadline:
        time.sleep(.001)
    arm.rm_set_arm_stop.assert_called_once()
    assert shutdown.is_alive()
    arm.rm_delete_robot_arm.assert_not_called()
    release.set()
    shutdown.join(1)
    assert not shutdown.is_alive()
    arm.rm_delete_robot_arm.assert_called_once()
    node.csv_logger.close.assert_called_once()


def test_blocked_worker_warning_does_not_force_free():
    hil = SingleSessionHil(Mock())
    hil.worker = Mock()
    hil.worker.is_alive.return_value = True
    warn = Mock()
    hil.wait_closed(warn)
    warn.assert_called_once()
    assert 'retaining API2 handle' in warn.call_args.args[0]
    assert hil.worker.join.call_args_list[0].kwargs == {'timeout': 2.0}
    assert hil.worker.join.call_args_list[1].args == ()


def test_history_gap_and_shutdown_reject():
    hil = SingleSessionHil(Mock())
    ready(hil)
    hil.history[30] = (hil.history[30][0] - .03, True)
    assert not hil.trigger(True, adapter_mock())[0]
    ready(hil)
    hil.begin_close()
    assert 'node_closing' in hil.trigger(True, adapter_mock())[1]


def test_missing_events_remain_incomplete(tmp_path):
    cap = Capture()
    cap.event(dict(event='MODBUS_START', timestamp_ns=10), 11)
    result = cap.save(tmp_path)
    assert not result['complete_window']
    assert cap.phase(20) == 'INJECTION'
    assert (tmp_path / 'summary.csv').is_file()
