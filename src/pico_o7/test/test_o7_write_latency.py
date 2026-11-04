"""Offline latency preparation tests; all native SDK calls replaced with mocks."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'rm_teleop')]
from pico_o7.o7_stage.mapping import Mapper
from pico_o7.o7_stage.plan import Index2Plan
from pico_o7.o7_stage.realtime import HandController,LatestHand
from pico_o7.o7_stage.write_timing import call_position_write,session_lease
from pico_o7.standalone_write import run
from pico_o7.write_latency_report import extract,stats
from test_o7_shared_stage import plan_data

class LatencyTests(unittest.TestCase):
    def controller(self):
        c=object.__new__(HandController)
        c.state=LatestHand(Mapper());c.stop=threading.Event();c.diagnostic=None;c.diagnostic_used=False
        self.events=[];self.adapter=Mock();self.adapter.write_o7_hil_positions.return_value=1
        c.adapter_getter=lambda:self.adapter;c.emit=self.events.append
        return c
    def test_identical_timing_scope_raw_codes_only(self):
        for code in (0,1,-3,999):
            adapter=Mock();adapter.write_o7_hil_positions.return_value=code
            result=call_position_write(adapter,[255,255,245,255,255,255,255])
            self.assertEqual(result['return_code'],code)
            self.assertGreaterEqual(result['call_end_timestamp_ns'],result['call_start_timestamp_ns'])
            self.assertAlmostEqual(result['duration_ms'],(result['call_end_monotonic_ns']-result['call_start_monotonic_ns'])/1e6)
            self.assertEqual(len(adapter.method_calls),1)
            self.assertNotIn('success',result)
        adapter=Mock();adapter.write_o7_hil_positions.side_effect=KeyboardInterrupt()
        result=call_position_write(adapter,[255]*7)
        self.assertIn('KeyboardInterrupt',result['error'])
        self.assertIsNotNone(result['call_end_timestamp_ns'])
    def test_latency_request_disabled_follow_only_consumes_once(self):
        c=self.controller();plan=Index2Plan(plan_data())
        c.state.enable();self.assertFalse(c.schedule_latency_once(plan,'r',lambda:'')[0])
        c.state.clear('operator_disabled',disable=True)
        self.assertTrue(c.schedule_latency_once(plan,'r',lambda:'')[0])
        self.assertFalse(c.state.enable()[0])
        c._run_latency_once(c.diagnostic)
        self.adapter.write_o7_hil_positions.assert_called_once_with(plan.target)
        self.assertTrue(c.state.closed)
        self.assertFalse(c.schedule_latency_once(plan,'r2',lambda:'')[0])
        self.assertEqual(self.events[-1]['calls_returned'],1)
    def test_disabled_again_cancels_queued_diagnostic(self):
        c=self.controller();plan=Index2Plan(plan_data())
        self.assertTrue(c.schedule_latency_once(plan,'r',lambda:'')[0])
        c.state.clear('operator_disabled_again',disable=True)
        c._run_latency_once(c.diagnostic)
        self.adapter.write_o7_hil_positions.assert_not_called()
        self.assertEqual(self.events[-1]['position_write_count'],0)
    def test_standalone_only_connect_position_delete_no_other_sdk_calls(self):
        arm=Mock(spec=['rm_create_robot_arm','rm_write_registers','rm_delete_robot_arm'])
        arm.rm_create_robot_arm.return_value=SimpleNamespace(id=7)
        arm.rm_write_registers.return_value=1;arm.rm_delete_robot_arm.return_value=0
        module=SimpleNamespace(RoboticArm=Mock(return_value=arm),
            rm_thread_mode_e=SimpleNamespace(RM_TRIPLE_MODE_E=3,RM_DUAL_MODE_E=2),
            rm_peripheral_read_write_params_t=lambda **kw:SimpleNamespace(**kw))
        with tempfile.TemporaryDirectory()as folder,contextlib.redirect_stdout(io.StringIO()):
            out=Path(folder)/'test_only'
            self.assertEqual(run(Index2Plan(plan_data()),'TRIPLE',out,module=module),0)
            rows=extract(out/'events.jsonl')
        self.assertEqual([c[0]for c in arm.method_calls],['rm_create_robot_arm','rm_write_registers','rm_delete_robot_arm'])
        params,data=arm.rm_write_registers.call_args.args
        self.assertEqual(vars(params),dict(port=1,address=0,device=39,num=7))
        self.assertEqual(data,[0,255,0,255,0,245,0,255,0,255,0,255,0,255])
        self.assertEqual(stats(rows)['raw_return_code_counts'],{'1':1})
        self.assertEqual(rows[0]['actual_motion'],'')
        module.RoboticArm.assert_called_once_with(3)
    def test_mutually_exclusive_project_session_lease(self):
        with session_lease():
            with self.assertRaisesRegex(RuntimeError,'Another project API2'):
                with session_lease():self.fail('second client must not be admitted')
    def test_plan_and_guard_reject_before_native(self):
        c=self.controller();plan=Index2Plan(plan_data())
        self.assertFalse(c.schedule_latency_once(plan,'r',lambda:'ARM not locked')[0])
        self.adapter.write_o7_hil_positions.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
