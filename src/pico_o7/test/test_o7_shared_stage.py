"""Mock-only verification; never constructs a real SDK/ROS node or contacts hardware."""
import ast
from dataclasses import dataclass
from datetime import datetime,timedelta,timezone
import json
from pathlib import Path
import sys
import threading
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'rm_teleop')]
from pico_o7.o7_stage.protocol import interpret_read,read_existing_session
from pico_o7.o7_stage.plan import Index2Plan
from pico_o7.o7_stage.worker import StageHil,run_index_once


def plan_data():
    data=json.loads((ROOT/'calibration/index2_plan.example.json').read_text())
    data.update(hold_confirmed_current_session=[True]*7,hold_evidence=['MOCK_ONLY']*7,
                local_window_confirmed_current_session=True,operator='MOCK_ONLY',
                confirmation_utc=datetime.now(timezone.utc).isoformat())
    return data


class SharedStageTests(unittest.TestCase):
    def test_nonzero_read_keeps_raw_but_never_positions(self):
        for code in [1,-1,-2,-3,-4,999]:
            row=interpret_read(code,[0]*14)
            self.assertFalse(row['valid']);self.assertIsNone(row['positions']);self.assertEqual(row['raw'],[0]*14)

    def test_full_read_byte_order_signed_bytes_and_bad_data(self):
        raw=[0,255,0,254,0,245,0,127,0,0,0,1,0,200]
        self.assertEqual(interpret_read(0,raw)['positions'],[255,254,245,127,0,1,200])
        self.assertEqual(interpret_read(0,[0,-1]*7)['positions'],[255]*7)
        for bad in [[0]*7,[0]*13,[0]*15,[True]*14,[300]*14,[1,0]*7]:
            self.assertFalse(interpret_read(0,bad)['valid'])
        self.assertEqual(interpret_read(0,245,index=2)['positions'],{'2':245})
        self.assertFalse(interpret_read(-2,0,index=2)['valid'])

    def test_read_calls_only_existing_arm_and_exact_ir_parameters(self):
        arm=Mock();arm.rm_read_multiple_input_registers.return_value=(0,[0,255]*7)
        arm.rm_read_input_registers.return_value=(0,245)
        adapter=SimpleNamespace(_require_arm=lambda:arm,module=SimpleNamespace(rm_peripheral_read_write_params_t=lambda **kw:SimpleNamespace(**kw)))
        self.assertTrue(read_existing_session(adapter)['valid'])
        self.assertEqual(vars(arm.rm_read_multiple_input_registers.call_args.args[0]),dict(port=1,address=0,device=39,num=7))
        self.assertTrue(read_existing_session(adapter,index=2)['valid'])
        self.assertEqual(vars(arm.rm_read_input_registers.call_args.args[0]),dict(port=1,address=2,device=39,num=1))
        self.assertEqual([c[0]for c in arm.method_calls],['rm_read_multiple_input_registers','rm_read_input_registers'])

    def test_other_six_values_reject_before_any_sdk_call(self):
        for index in [0,1,3,4,5,6]:
            data=plan_data();data['target_positions'][index]=254
            with self.assertRaisesRegex(ValueError,'non-selected'):Index2Plan(data)

    def test_dry_run_unconfirmed_is_not_executable(self):
        plan=Index2Plan.load(ROOT/'calibration/index2_plan.example.json')
        dry=plan.dry_run();self.assertEqual(dry['sdk_calls'],0)
        self.assertTrue(dry['structurally_valid']);self.assertFalse(dry['field_confirmations_complete'])
        adapter=Mock()
        with self.assertRaises(ValueError):run_index_once(adapter,plan,lambda _:None,lambda:'')
        adapter.write_o7_hil_positions.assert_not_called()

    def test_all_hold_confirmations_and_fresh_evidence_required(self):
        for index in range(7):
            data=plan_data();data['hold_confirmed_current_session'][index]=False
            with self.assertRaises(ValueError):Index2Plan(data).validate()
        data=plan_data();data['confirmation_utc']=(datetime.now(timezone.utc)-timedelta(hours=2)).isoformat()
        with self.assertRaisesRegex(ValueError,'stale'):Index2Plan(data).validate()

    def test_domain_index_and_local_window_validation(self):
        for change in [dict(joint_index=1),dict(joint_index=True),dict(hold_positions=[0]*6),
                       dict(target_positions=[False]*7),dict(approved_index_range=[0,255]),
                       dict(approved_index_range=[250,255])]:
            data=plan_data();data.update(change)
            with self.assertRaises(ValueError):Index2Plan(data)

    def test_once_preserves_return1_no_retry_return_or_reads(self):
        adapter=Mock();adapter.write_o7_hil_positions.return_value=1
        events=[];run_index_once(adapter,Index2Plan(plan_data()),events.append,lambda:'')
        adapter.write_o7_hil_positions.assert_called_once_with((255,255,245,255,255,255,255))
        self.assertEqual(len(adapter.method_calls),1)
        self.assertEqual(events[-1]['position_write_count'],1)
        self.assertEqual(events[-1]['calls_returned'],1)
        self.assertEqual(events[-1]['last_return_code'],1)
        self.assertEqual(events[-1]['physical_write_result'],'requires_field_observation')

    def test_guard_rechecked_after_start_before_submission(self):
        adapter=Mock();events=[]
        def emit(e):events.append(e)
        guard=Mock(side_effect=['','stopped'])
        run_index_once(adapter,Index2Plan(plan_data()),emit,guard)
        adapter.write_o7_hil_positions.assert_not_called()
        self.assertEqual(events[-1]['position_write_count'],0)

    def test_existing_monitor_can_capture_new_single_write(self):
        from rm_teleop.hil_monitor_data import Capture
        adapter=Mock();adapter.write_o7_hil_positions.return_value=1
        capture=Capture()
        run_index_once(adapter,Index2Plan(plan_data()),lambda e:capture.event(e,e['timestamp_ns']),lambda:'')
        with tempfile.TemporaryDirectory()as folder:
            summary=capture.save(folder)
        self.assertEqual(summary['stream']['return_codes'],{'1':1})
        self.assertEqual(summary['stream']['last_target_positions'],[255,255,245,255,255,255,255])

    def test_once_all_return_codes_are_only_raw_diagnostics(self):
        for code in (0,1,-1,-2,-3,-4,999):
            adapter=Mock();adapter.write_o7_hil_positions.return_value=code
            events=[];run_index_once(adapter,Index2Plan(plan_data()),events.append,lambda:'')
            self.assertEqual(events[-1]['calls_returned'],1)
            self.assertEqual(events[-1]['last_return_code'],code)
            self.assertEqual(events[-1]['physical_write_result'],'requires_field_observation')
            self.assertNotIn('sdk_acknowledged_success_count',events[-1])
            adapter.write_o7_hil_positions.assert_called_once()

    def test_worker_exception_consumes_attempt_and_never_restores(self):
        adapter=Mock();adapter.write_o7_hil_positions.side_effect=RuntimeError('MOCK_ONLY failure')
        events=[];owner=StageHil(events.append)
        now=time.monotonic()
        for i in range(201):owner.record(now-1+i*.005,True)
        accepted,_=owner.trigger(True,adapter,stream_config=Index2Plan(plan_data()),safety=lambda:'')
        self.assertTrue(accepted);owner.wait_closed(lambda _:None)
        self.assertTrue(owner.used);adapter.write_o7_hil_positions.assert_called_once()
        accepted,_=owner.trigger(True,adapter,stream_config=Index2Plan(plan_data()),safety=lambda:'')
        self.assertFalse(accepted)
        self.assertEqual(events[-1]['sdk_exception_count'],1)

    def test_read_and_write_cannot_overlap_and_close_waits(self):
        entered=threading.Event();release=threading.Event();events=[]
        arm=SimpleNamespace()
        def read(params):entered.set();release.wait(2);return(0,[0,255]*7)
        arm.rm_read_multiple_input_registers=read
        adapter=SimpleNamespace(_require_arm=lambda:arm,module=SimpleNamespace(rm_peripheral_read_write_params_t=lambda **kw:SimpleNamespace(**kw)))
        owner=StageHil(events.append)
        accepted,request_id=owner.trigger_read(adapter,lambda:'')
        self.assertTrue(accepted);self.assertTrue(entered.wait(.5))
        try:
            self.assertFalse(owner.trigger_read(adapter,lambda:'')[0])
            self.assertFalse(owner.trigger(True,adapter,stream_config=Index2Plan(plan_data()),safety=lambda:'')[0])
            owner.begin_close()
            self.assertFalse(owner.trigger_read(adapter,lambda:'')[0])
        finally:release.set();owner.wait_closed(lambda _:None)
        self.assertEqual(events[-1]['request_id'],request_id)
        self.assertFalse(owner.used)  # Diagnostic reads do not spend the future calibration attempt.

    def test_node_read_gate_rejects_motion_enabled(self):
        tree=ast.parse((ROOT/'pico_o7/shared_session_node.py').read_text())
        method=next(n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name=='_read_rejection')
        env={};exec(compile(ast.Module(body=[method],type_ignores=[]),'node_read_gate','exec'),env)
        node=SimpleNamespace(enable_o7_read_diag=True,motion_permitted=True)
        self.assertIn('allow_motion=false',env['_read_rejection'](node))

    def test_migrated_stage_import_has_no_sdk_side_effect(self):
        import pico_o7.shared_session_node
        self.assertTrue((ROOT/'config/o7_device_linear.json').is_file())
        self.assertFalse(any(n.startswith('Robotic_Arm')for n in sys.modules))


if __name__=='__main__':unittest.main(verbosity=2)
