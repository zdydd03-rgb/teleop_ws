"""Offline guards/worker regression. No ROS node or hardware SDK is constructed."""
import ast
import importlib.util
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

WORKSPACE=Path(__file__).resolve().parents[1]
PROJECT = WORKSPACE/'src/pico_o7'
CANDIDATE = Path(os.environ.get('O7_REVIEW_SOURCE', str(PROJECT/'pico_o7')))
sys.path[:0] = [str(PROJECT), str(WORKSPACE/'src/rm_teleop')]
from pico_o7.o7_stage.mapping import Mapper

spec = importlib.util.spec_from_file_location('pico_o7.o7_stage.review_realtime', CANDIDATE/'o7_stage/realtime.py')
realtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(realtime)


def method(path, name):
    tree = ast.parse(path.read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    env = {'time': time, 'math': math}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), env)
    return env[name]


class CombinedGuards(unittest.TestCase):
    def setUp(self):
        self.guard = method(CANDIDATE/'shared_session_node.py', '_hand_rejection')
        self.base = method(WORKSPACE/'src/rm_teleop/rm_teleop/rm65_canfd_high_follow_node.py', '_hil_rejection')
        self.node = SimpleNamespace(
            _stop_event=threading.Event(), _fault_latched=False, _fault_reason='',
            _adapter_lock=threading.Lock(), _connected=True, _ik_initialized=True,
            _adapter=SimpleNamespace(arm=object(), handle=object()),
            motion_permitted=True, publish_hz=200.,
            _last_summary=SimpleNamespace(state='ACTIVE', command_sent=True),
            _last_command_send_monotonic=100.,
            store=SimpleNamespace(latest=lambda: SimpleNamespace(received_monotonic=100.)),
            _critical_feedback_reason=lambda **kw: 'robot_feedback_stale' if kw['feedback_age'] > .05 else '')
        self.node._hil_rejection = lambda: self.base(self.node)

    def reject(self):
        with patch.object(time, 'monotonic', return_value=100.005):
            return self.guard(self.node)

    def test_locked_hand_stays_independent_of_active(self):
        self.node.motion_permitted = False
        self.node._last_summary.state = 'MOTION_LOCKED'
        self.node._last_command_send_monotonic = None
        self.node._hil_rejection = Mock(side_effect=AssertionError('ARM gate called while locked'))
        self.assertEqual(self.reject(), '')

    def test_combined_active_fresh_sending_allowed(self):
        self.assertEqual(self.reject(), '')

    def test_arm_ready_does_not_reject_hand(self):
        self.node._last_summary.state = 'READY'
        self.assertEqual(self.reject(), '')

    def test_arm_unsent_does_not_reject_hand(self):
        self.node._last_summary.command_sent = False
        self.assertEqual(self.reject(), '')

    def test_arm_feedback_does_not_reject_hand(self):
        self.node.store.latest = lambda: SimpleNamespace(received_monotonic=99.)
        self.assertEqual(self.reject(), '')

    def test_canfd_gap_does_not_reject_hand(self):
        self.node._last_command_send_monotonic = 99.98
        self.assertEqual(self.reject(), '')

    def test_arm_fault_ignored_but_shared_disconnect_and_shutdown_rejected(self):
        self.node._fault_latched = True
        self.assertEqual(self.reject(), '')
        self.node._fault_latched = False
        self.node._connected = False
        self.assertIn('not connected', self.reject())
        self.node._stop_event.set()
        self.assertEqual(self.reject(), 'node_closing')


class CombinedWorker(unittest.TestCase):
    def controller(self, guard, emit=lambda event: None, write=None):
        adapter=SimpleNamespace(write_o7_hil_positions=write)
        with patch.object(realtime.HandController,'_solve_loop',lambda self:None):
            controller=realtime.HandController(Mapper(),lambda:adapter,guard,emit,
                                               initializer=lambda *a:dict(success=True))
        controller.solver_thread.join(1)
        controller.state.enable()
        return controller

    def wait_for(self, predicate):
        deadline = time.monotonic() + 1
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(predicate())

    def test_empty_mailbox_shared_disconnect_pauses_and_resumes_without_cached_write(self):
        reason = ['existing API2 session not connected']
        controller = self.controller(lambda: reason[0])
        thread = threading.Thread(target=controller.write_loop)
        thread.start()
        try:
            self.wait_for(lambda: not controller.state.enabled)
            self.assertEqual(controller.state.reason, reason[0])
            self.assertTrue(controller.state.resume_pending)
            reason[0] = ''
            self.wait_for(lambda:controller.state.enabled)
            self.assertFalse(controller.state.resume_pending)
            self.assertEqual(controller.state.counts['writes'], 0)
        finally:
            controller.stop.set(); thread.join(1)
        self.assertFalse(thread.is_alive())

    def test_blocked_write_returns_then_stops_without_overlap_or_retry(self):
        reason = ['']; entered = threading.Event(); release = threading.Event(); calls = []
        def write(target):
            calls.append((list(target), threading.current_thread().name))
            entered.set(); release.wait(1); return 1
        events = []
        controller = self.controller(lambda: reason[0], events.append, write)
        now = time.monotonic()
        q = {n: (lo+hi)/2 for n, (lo, hi) in controller.state.mapper.bounds.items()}
        with controller.state.lock:
            controller.state.last_received = now
            controller.state.pending = (controller.state.epoch, now, dict(valid=True, q_by_name=q, seq=1))
        thread = threading.Thread(target=controller.write_loop, name='mock_o7_worker')
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            reason[0] = 'node_closing'
            release.set()
            self.wait_for(lambda: not controller.state.enabled)
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0][1], 'mock_o7_worker')
            self.assertEqual(controller.state.return_codes, {'1': 1})
            finish=next(e for e in events if e['event']=='O7_HAND_WRITE_FINISH')
            self.assertEqual(finish['return_code'], 1)
            self.wait_for(lambda:controller.state.closed)
            self.assertEqual(controller.state.reason, 'node_closing')
            self.assertFalse(controller.state.startup_pending)
        finally:
            release.set(); controller.stop.set(); thread.join(1)
        self.assertFalse(thread.is_alive())


class DecoupledRuntime(unittest.TestCase):
    def test_arm_transitions_do_not_pause_or_reinitialize_hand(self):
        guard=method(CANDIDATE/'shared_session_node.py','_hand_rejection')
        node=SimpleNamespace(_stop_event=threading.Event(),_adapter_lock=threading.Lock(),
                             _connected=True,_adapter=SimpleNamespace(arm=object()),
                             _hil_rejection=Mock(side_effect=AssertionError('ARM guard consulted')))
        calls=[];inits=[]
        adapter=SimpleNamespace(write_o7_hil_positions=lambda v:(calls.append(list(v)),0)[1])
        with patch.object(realtime.HandController,'_solve_loop',lambda self:None):
            c=realtime.HandController(Mapper(),lambda:adapter,lambda:guard(node),lambda e:None,
                start_from_parameter=True,initializer=lambda *a:(inits.append(a),dict(success=True))[1])
        thread=threading.Thread(target=c.write_loop);thread.start()
        def wait(predicate):
            deadline=time.monotonic()+2
            while not predicate() and time.monotonic()<deadline:time.sleep(.005)
            self.assertTrue(predicate())
        try:
            wait(lambda:c.state.enabled);epoch=c.state.epoch
            for i,state in enumerate(('ACTIVE','READY','WAITING_ROBOT','FAULT_LATCHED')):
                node._last_summary=SimpleNamespace(state=state,command_sent=False)
                node._last_command_send_monotonic=time.monotonic()-.025
                node._fault_latched=state=='FAULT_LATCHED'
                now=time.monotonic()
                q={n:lo+(hi-lo)*(.3+.15*i) for n,(lo,hi) in c.state.mapper.bounds.items()}
                with c.state.lock:
                    c.state.last_received=now
                    c.state.pending=(epoch,now,dict(valid=True,seq=i,q_by_name=q))
                wait(lambda:len(calls)==i+1 and not c.state.inflight)
                self.assertTrue(c.state.enabled);self.assertEqual(c.state.epoch,epoch)
            self.assertEqual(len(inits),1)
            node._hil_rejection.assert_not_called()
        finally:c.shutdown();thread.join(2)
        self.assertFalse(thread.is_alive())


if __name__ == '__main__':
    unittest.main(verbosity=2)
