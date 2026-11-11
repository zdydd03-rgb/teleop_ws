"""Offline checks: real input/solver, mocked native writes, NO hardware connection."""
import copy
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'rm_teleop')]
from pico_o7.o7_stage.mapping import Mapper,JOINTS
from pico_o7.o7_stage.hand_input import input_valid,ForwardCore
from pico_o7.o7_stage.realtime import LatestHand,HandController
from pico_o7.o7_stage.worker import StageHil
from pico_o7.o7_stage.plan import Index2Plan
from rm_teleop.receiver_core import ReceiverCore
from rm_teleop.rm_api2_motion_adapter import RmApi2MotionAdapter
from test_o7_shared_stage import plan_data

RAW=ROOT/'test/fixtures/right_hand_frame.jsonl'
def payload():
    with RAW.open()as stream:return json.loads(json.loads(next(stream))['raw_text'])

class RealtimeTests(unittest.TestCase):
    def setUp(self):
        self.mapper=Mapper()
        # Unit-test fixture only; do not depend on the operator's on-disk Hz/step tuning.
        self.mapper.config['write_hz']=2.
        for axis in self.mapper.axes:axis['max_step']=8
        self.now=[100.];self.state=LatestHand(self.mapper,lambda:self.now[0])
        self.state.enable();self.p=payload()
    def q(self,fraction):return{n:lo+fraction*(hi-lo)for n,(lo,hi)in self.mapper.bounds.items()}
    def offer(self,q=None):
        self.state.receive(dict(received_monotonic=self.now[0],valid=True,payload=self.p))
        epoch,received,_=self.state.take_raw()
        self.state.offer(epoch,received,dict(valid=True,seq=self.p['seq'],q_by_name=q or self.q(.5)))
    def test_named_reorder_independent_of_dictionary_or_solver_order(self):
        q={n:(self.mapper.bounds[n][1]*((i+1)/8))for i,n in enumerate(JOINTS)}
        scrambled=dict(reversed(list(q.items())))
        self.assertEqual(self.mapper.convert(scrambled)[0],[223,191,159,128,96,64,32])
    def test_direction_flip_and_per_axis_range(self):
        self.mapper.axes[1].update(direction='increasing',device_min=100,device_max=200)
        result,_=self.mapper.convert(self.q(.25))
        self.assertEqual(result,[191,125,191,191,191,191,191])
    def test_clamp_both_sides_and_single_step(self):
        self.assertEqual(self.mapper.convert(self.q(-5))[0],[255]*7)
        self.assertEqual(self.mapper.convert(self.q(5))[0],[0]*7)
        self.assertEqual(self.mapper.convert(self.q(1),[255]*7)[0],[247]*7)
        self.assertEqual(self.mapper.convert(self.q(0),[120]*7)[0],[128]*7)
    def test_nan_inf_bool_missing_rejected_before_target(self):
        for bad in (float('nan'),float('inf'),True):
            q=self.q(.5);q[JOINTS[2]]=bad
            with self.assertRaises(ValueError):self.mapper.convert(q)
            self.offer(q);self.assertIsNone(self.state.claim())
        q=self.q(.5);del q[JOINTS[2]]
        with self.assertRaises(KeyError):self.mapper.convert(q)
    def test_invalid_keypoint_tracking_and_no_finite_data(self):
        self.offer()
        for change in ('tracked','valid','nan','count'):
            p=payload()
            if change=='tracked':p['right_hand_tracked']=False
            if change=='valid':p['right_keypoints'][8]['valid']=False
            if change=='nan':p['right_keypoints'][8]['position']['x']=float('nan')
            if change=='count':p['right_keypoints'].pop()
            self.assertFalse(input_valid(p))
            self.state.receive(dict(received_monotonic=self.now[0],valid=True,payload=p))
            self.assertIsNone(self.state.claim())
    def test_stale_input_solution_and_pending_discard(self):
        self.offer();self.now[0]+=.26;self.assertIsNone(self.state.claim())
        self.assertIsNone(self.state.pending)
        self.state.receive(dict(received_monotonic=90.,valid=True,payload=self.p))
        self.assertIsNone(self.state.take_raw())
        self.state.receive(dict(received_monotonic=self.now[0],valid=True,payload=self.p))
        epoch,received,_=self.state.take_raw();self.now[0]+=.26
        self.state.offer(epoch,received,dict(valid=True,q_by_name=self.q(.5)))
        self.assertIsNone(self.state.claim())
    def test_stop_reenable_rejects_old_solver_and_cached_targets(self):
        self.offer();old_epoch=self.state.epoch;old_time=self.now[0]
        self.state.clear('operator_disabled',disable=True);self.now[0]+=.001;self.state.enable()
        # A ROS message buffered before enable is still young, but is NOT new input.
        self.state.receive(dict(received_monotonic=old_time,valid=True,payload=self.p))
        self.assertIsNone(self.state.take_raw())
        self.assertEqual(self.state.counts['pre_enable_dropped'],1)
        self.assertIsNone(self.state.claim())
        self.state.offer(old_epoch,old_time,dict(valid=True,q_by_name=self.q(.5)))
        self.assertIsNone(self.state.claim())
        self.offer();self.assertIsNotNone(self.state.claim())
    def test_cancel_after_claim_and_no_hold_anchor_advance(self):
        self.offer();claimed=self.state.claim()
        self.state.clear('operator_disabled',disable=True)
        self.assertFalse(self.state.authorize_call(claimed))
        self.assertEqual(self.state.last_target,[255]*7);self.assertEqual(self.state.counts['writes'],0)
    def test_return_codes_disable_failed_session_without_retry(self):
        for code in (0,1,-1,-2,-3,-4,999):
            self.setUp();self.offer(self.q(0));first=self.state.claim()
            self.assertTrue(self.state.authorize_call(first));self.state.finish(code,2.)
            self.assertEqual(self.state.enabled,code == 0)
            self.assertIsNone(self.state.pending)

    def test_slow_call_only_duration_based(self):
        self.offer();self.state.claim();self.state.finish(0,751.)
        self.assertFalse(self.state.enabled);self.assertTrue(self.state.recovery)
    def test_latest_overwrites_and_low_frequency(self):
        for i in range(10):self.state.receive(dict(received_monotonic=self.now[0],valid=True,payload=self.p))
        self.assertEqual(self.state.counts['raw_overwritten'],9)
        self.offer(self.q(.5));self.offer(self.q(.7))
        self.assertEqual(self.state.counts['target_overwritten'],1)
        self.state.claim();self.state.finish(0,1.)
        self.now[0]+=.1;self.offer();self.assertIsNone(self.state.claim())
        self.now[0]+=.41;self.offer();self.assertIsNotNone(self.state.claim())
    def test_original_receiver_path_retains_21_points_without_extra_socket(self):
        events=[];core=ForwardCore(ReceiverCore(),events.append)
        p=payload();raw=json.dumps(p).encode()
        result=core.process_datagram(raw,100.)
        self.assertTrue(result.accepted);self.assertTrue(events[-1]['valid'])
        self.assertEqual(events[-1]['payload']['right_keypoints'][8]['position'],p['right_keypoints'][8]['position'])
        self.assertEqual(len(events),1)
        self.assertFalse(core.process_datagram(raw,100.01).accepted)
        self.assertEqual(len(events),1)
        p['seq']+=1;p['right_keypoints'][8]['valid']=False
        self.assertTrue(core.process_datagram(json.dumps(p).encode(),100.02).accepted)
        self.assertFalse(events[-1]['valid'])
    def test_existing_adapter_exact_slave39_fc16_bytes(self):
        arm=Mock();arm.rm_write_registers.return_value=1
        adapter=object.__new__(RmApi2MotionAdapter)
        adapter.arm=arm;adapter.module=SimpleNamespace(rm_peripheral_read_write_params_t=lambda **k:SimpleNamespace(**k))
        self.assertEqual(adapter.write_o7_hil_positions([255,254,245,127,0,1,200]),1)
        params,data=arm.rm_write_registers.call_args.args
        self.assertEqual(vars(params),dict(port=1,address=0,device=39,num=7))
        self.assertEqual(data,[0,255,0,254,0,245,0,127,0,0,0,1,0,200])
        self.assertEqual([c[0]for c in arm.method_calls],['rm_write_registers'])
    def test_new_locked_once_does_not_weaken_old_active_gate(self):
        plan=Index2Plan(plan_data());adapter=Mock();adapter.write_o7_hil_positions.return_value=1
        owner=StageHil(lambda _:None)
        self.assertFalse(owner.trigger(True,adapter,stream_config=plan,safety=lambda:'')[0])
        self.assertTrue(owner.trigger_index_locked(adapter,plan,lambda:'')[0])
        owner.wait_closed(lambda _:None)
        adapter.write_o7_hil_positions.assert_called_once_with(plan.target)
    def test_actual_solver_subprocess_and_shared_worker_mock_native(self):
        events=[];done=threading.Event();calls=[]
        def write(v):calls.append(list(v));done.set();return 1
        controller=HandController(Mapper(),lambda:SimpleNamespace(write_o7_hil_positions=write),lambda:'',events.append)
        owner=StageHil(events.append)
        try:
            self.assertTrue(owner.start_hand_worker(controller)[0])
            self.assertFalse(controller.state.enabled)
            self.assertTrue(controller.state.enable()[0])
            deadline=time.monotonic()+5
            while not done.is_set()and time.monotonic()<deadline:
                controller.state.receive(dict(received_monotonic=time.monotonic(),valid=True,payload=payload()))
                time.sleep(.04)
            self.assertTrue(done.is_set(),'real solver process did not produce a fresh target')
            # Disable, drain the one native mock call, re-enable WITHOUT new data.
            controller.state.clear('operator_disabled',disable=True)
            time.sleep(.03)
            before=len(calls);self.assertTrue(controller.state.enable()[0]);time.sleep(.6)
            self.assertEqual(len(calls),before)
            self.assertTrue(any(e['event']=='O7_HAND_WRITE_FINISH'and e['return_code']==1 for e in events))
            self.assertTrue(controller.state.enabled) # code1 did not close the gate.
            self.assertTrue(all(len(v)==7 and all(type(x)is int and 0<=x<=255 for x in v)for v in calls))
        finally:
            controller.shutdown();owner.begin_close();owner.wait_closed(lambda _:None)
        self.assertFalse(owner.worker.is_alive())
        self.assertFalse(controller.solver_thread.is_alive())

if __name__=='__main__':unittest.main(verbosity=2)
