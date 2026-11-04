"""Regression for the recorded tracking loss → ARM recovery → hand stuck case."""
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

from pico_o7.o7_stage.mapping import Mapper
from pico_o7.o7_stage.realtime import HandController, LatestHand


def wait_for(predicate):
    deadline=time.monotonic()+1
    while not predicate() and time.monotonic()<deadline:time.sleep(.005)
    assert predicate()


def offer(controller,fraction):
    state=controller.state;now=time.monotonic()
    with state.lock:
        state.last_received=now
        q={n:lo+(hi-lo)*fraction for n,(lo,hi) in state.mapper.bounds.items()}
        state.pending=(state.epoch,now,dict(valid=True,seq=1,q_by_name=q))


def controller(guard,write,events):
    with patch.object(HandController,'_solve_loop',lambda self:None):
        c=HandController(Mapper(),lambda:SimpleNamespace(write_o7_hil_positions=write),guard,events.append,start_from_parameter=True)
    thread=threading.Thread(target=c.write_loop);thread.start()
    return c,thread


def test_tracking_exit_active_returns_to_fresh_following_and_keeps_step_anchor():
    reason=[''];calls=[];events=[]
    c,thread=controller(lambda:reason[0],lambda v:(calls.append(list(v)),1)[1],events)
    try:
        wait_for(lambda:c.state.enabled);offer(c,.6)
        wait_for(lambda:len(calls)==1 and not c.state.inflight)
        anchor=list(c.state.last_target);old_epoch=c.state.epoch
        reason[0]='control_not_ACTIVE_sending'
        wait_for(lambda:c.state.resume_pending)
        assert not c.state.enabled and c.state.pending is None
        assert c.state.last_target==anchor
        # A late solver result from before the tracking loss must never revive.
        c.state.offer(old_epoch,time.monotonic(),dict(valid=True,q_by_name={}))
        assert c.state.pending is None
        reason[0]='';wait_for(lambda:c.state.enabled)
        time.sleep(.27)
        assert len(calls)==1  # Restoring ACTIVE alone submits no cached/hold target.
        offer(c,.9);wait_for(lambda:len(calls)==2)
        assert all(abs(a-b)<=16 for a,b in zip(calls[0],calls[1]))
        assert any(e['event']=='O7_HAND_RESUME' for e in events)
        assert c.state.return_codes=={'1':2}
    finally:c.shutdown();thread.join(1)
    assert not thread.is_alive()


def test_canfd_temporary_gap_is_pause_not_permanent_disable():
    s=LatestHand(Mapper());s.enable();s.reject_control('CANFD_not_continuously_sending')
    assert not s.enabled and s.resume_pending and s.startup_pending
    assert s.raw is None and s.pending is None


def test_hard_fault_disconnect_and_feedback_stale_do_not_auto_resume():
    for reason in ('ARM_fault_latched','FAULT_LATCHED:joint_error','robot_feedback_stale',
                   'existing API2 session not connected','node_closing'):
        s=LatestHand(Mapper());s.enable();s.reject_control(reason)
        assert not s.enabled and not s.startup_pending and not s.resume_pending


def test_fault_while_waiting_to_resume_cancels_recovery():
    reason=[''];events=[]
    c,thread=controller(lambda:reason[0],lambda v:1,events)
    try:
        wait_for(lambda:c.state.enabled)
        reason[0]='control_not_ACTIVE_sending';wait_for(lambda:c.state.resume_pending)
        reason[0]='ARM_fault_latched';wait_for(lambda:not c.state.startup_pending)
        reason[0]='';time.sleep(.03)
        assert not c.state.enabled
    finally:c.shutdown();thread.join(1)


def test_inflight_write_drains_serially_before_recovery_and_no_old_target_replay():
    entered=threading.Event();release=threading.Event();reason=[''];events=[];calls=[]
    def write(v):
        calls.append(list(v));entered.set();release.wait(1);return 1
    c,thread=controller(lambda:reason[0],write,events)
    try:
        wait_for(lambda:c.state.enabled);offer(c,.6);assert entered.wait(1)
        reason[0]='control_not_ACTIVE_sending'
        assert c.state.inflight
        release.set();wait_for(lambda:c.state.resume_pending)
        assert len(calls)==1
        reason[0]='';wait_for(lambda:c.state.enabled)
        time.sleep(.27);assert len(calls)==1
        offer(c,.9);wait_for(lambda:len(calls)==2)
    finally:release.set();c.shutdown();thread.join(1)


def test_sdk_exception_and_slow_call_still_require_restart():
    for duration,error in [(12.,'mock native exception'),(751.,None)]:
        s=LatestHand(Mapper());s.enable();s.finish(1,duration,error)
        assert not s.enabled and not s.startup_pending
        assert 'requires_restart' in s.reason


def test_tracking_invalid_and_timeout_only_clear_mailboxes():
    s=LatestHand(Mapper());s.enable()
    frame=json.loads(json.loads((Path(__file__).parent/'fixtures/right_hand_frame.jsonl').read_text())['raw_text'])
    frame['right_hand_tracked']=False
    s.receive(dict(valid=True,received_monotonic=time.monotonic(),payload=frame))
    assert s.enabled and s.raw is None and s.pending is None
    assert s.reason=='tracking_invalid'
