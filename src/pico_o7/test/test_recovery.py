"""O7 transport/input recovery regressions, independent of ARM motion state."""
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

from pico_o7.o7_stage.mapping import Mapper
from pico_o7.o7_stage.realtime import HandController, LatestHand


def wait_for(predicate):
    deadline=time.monotonic()+5
    while not predicate() and time.monotonic()<deadline:time.sleep(.005)
    assert predicate()


def offer(controller,fraction):
    state=controller.state;now=state.clock()
    with state.lock:
        state.last_received=now
        q={n:lo+(hi-lo)*fraction for n,(lo,hi) in state.mapper.bounds.items()}
        state.pending=(state.epoch,now,dict(valid=True,seq=1,q_by_name=q))


def controller(guard,write,events):
    with patch.object(HandController,'_solve_loop',lambda self:None):
        adapter=SimpleNamespace(write_o7_hil_positions=write)
        c=HandController(Mapper(),lambda:adapter,guard,events.append,start_from_parameter=True,initializer=lambda *a:dict(success=True))
    thread=threading.Thread(target=c.write_loop);thread.start()
    return c,thread


def test_shared_connection_recovery_requires_fresh_input_and_keeps_step_anchor():
    reason=[''];calls=[];events=[]
    c,thread=controller(lambda:reason[0],lambda v:(calls.append(list(v)),0)[1],events)
    try:
        wait_for(lambda:c.state.enabled);offer(c,.6)
        wait_for(lambda:len(calls)==1 and not c.state.inflight)
        anchor=list(c.state.last_target);old_epoch=c.state.epoch
        reason[0]='existing API2 session not connected'
        wait_for(lambda:c.state.resume_pending)
        assert not c.state.enabled and c.state.pending is None
        assert c.state.last_target==anchor
        # A late solver result from before the tracking loss must never revive.
        c.state.offer(old_epoch,time.monotonic(),dict(valid=True,q_by_name={}))
        assert c.state.pending is None
        reason[0]='';wait_for(lambda:c.state.enabled)
        time.sleep(.27)
        assert len(calls)==1  # Restoring transport alone submits no cached/hold target.
        offer(c,.9);wait_for(lambda:len(calls)==2)
        assert all(abs(a-b)<=16 for a,b in zip(calls[0],calls[1]))
        assert any(e['event']=='O7_HAND_RESUME' for e in events)
        assert c.state.return_codes=={'0':2}
    finally:c.shutdown();thread.join(1)
    assert not thread.is_alive()


def test_shared_connection_loss_is_pause_not_permanent_disable():
    s=LatestHand(Mapper());s.enable();s.reject_control('existing API2 session not connected')
    assert not s.enabled and s.resume_pending and s.startup_pending
    assert s.raw is None and s.pending is None


def test_hand_local_fatal_rejections_do_not_auto_resume():
    for reason in ('node_closing','solver_unavailable'):
        s=LatestHand(Mapper());s.enable();s.reject_control(reason)
        assert not s.enabled and not s.startup_pending and not s.resume_pending


def test_fault_while_waiting_to_resume_cancels_recovery():
    reason=[''];events=[]
    c,thread=controller(lambda:reason[0],lambda v:1,events)
    try:
        wait_for(lambda:c.state.enabled)
        reason[0]='existing API2 session not connected';wait_for(lambda:c.state.resume_pending)
        reason[0]='node_closing';wait_for(lambda:not c.state.startup_pending)
        reason[0]='';time.sleep(.03)
        assert not c.state.enabled
    finally:c.shutdown();thread.join(1)


def test_inflight_write_drains_serially_before_recovery_and_no_old_target_replay():
    entered=threading.Event();release=threading.Event();reason=[''];events=[];calls=[]
    def write(v):
        calls.append(list(v));entered.set();release.wait(1);return 0
    c,thread=controller(lambda:reason[0],write,events)
    try:
        wait_for(lambda:c.state.enabled);offer(c,.6);assert entered.wait(1)
        reason[0]='existing API2 session not connected'
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
        assert not s.enabled and s.startup_pending
        assert s.recovery


def test_tracking_invalid_and_timeout_only_clear_mailboxes():
    s=LatestHand(Mapper());s.enable()
    frame=json.loads(json.loads((Path(__file__).parent/'fixtures/right_hand_frame.jsonl').read_text())['raw_text'])
    frame['right_hand_tracked']=False
    s.receive(dict(valid=True,received_monotonic=time.monotonic(),payload=frame))
    assert s.enabled and s.raw is None and s.pending is None
    assert s.reason=='tracking_invalid'


def test_modbus_failed_init_retries_with_backoff_then_latches():
    events=[];calls=[];inits=[];now=[100.]
    adapter=SimpleNamespace(write_o7_hil_positions=lambda v:(calls.append(v),1)[1])
    def initialize(*args):inits.append(now[0]);return dict(success=len(inits)==1)
    with patch.object(HandController,'_solve_loop',lambda self:None):
        c=HandController(Mapper(),lambda:adapter,lambda:'',events.append,start_from_parameter=True,initializer=initialize)
    c.state.clock=lambda:now[0]
    thread=threading.Thread(target=c.write_loop);thread.start()
    try:
        wait_for(lambda:c.state.enabled);assert len(inits)==1
        offer(c,.6);wait_for(lambda:len(calls)==1)
        time.sleep(.03);assert len(inits)==1
        for attempt,delay in enumerate((1.,2.,4.),start=2):
            now[0]+=delay;wait_for(lambda:len(inits)==attempt)
        wait_for(lambda:c.state.fault_latched)
        now[0]+=120;time.sleep(.03)
        assert len(inits)==4 and len(calls)==1 and not c.state.enabled
        assert c.state.pending is None and c.state.raw is None
        assert not c.state.enable()[0]
    finally:c.shutdown();thread.join(1)


def test_reconnect_and_stop_start_require_initialization():
    events=[];inits=[];current=[SimpleNamespace(write_o7_hil_positions=lambda v:0)]
    with patch.object(HandController,'_solve_loop',lambda self:None):
        c=HandController(Mapper(),lambda:current[0],lambda:'',events.append,start_from_parameter=True,
                         initializer=lambda *a:(inits.append(a),dict(success=True))[1])
    thread=threading.Thread(target=c.write_loop);thread.start()
    try:
        wait_for(lambda:c.state.enabled);assert len(inits)==1
        c.state.clear('operator_disabled',disable=True);c.state.modbus_initialized=False
        c.state.request_startup();wait_for(lambda:len(inits)==2 and c.state.enabled)
        current[0]=SimpleNamespace(write_o7_hil_positions=lambda v:0)
        wait_for(lambda:len(inits)==3 and c.state.enabled)
        assert c.state.pending is None
    finally:c.shutdown();thread.join(1)


def test_ten_hz_limit_and_recent_statistics():
    now=[10.]
    s=LatestHand(Mapper(),clock=lambda:now[0]);s.enable()
    assert s.mapper.config['write_hz']==10
    def target(fraction):
        q={n:lo+(hi-lo)*fraction for n,(lo,hi) in s.mapper.bounds.items()}
        s.last_received=now[0];s.pending=(s.epoch,now[0],dict(valid=True,q_by_name=q))
    target(.5);assert s.claim() is not None;s.finish(0,68)
    now[0]+=.099;target(.8);assert s.claim() is None
    now[0]+=.002;assert s.claim() is not None;s.finish(0,68)
    assert s.status()['recent_acknowledged_hz']==.4
    now[0]+=6;assert s.status()['recent_write_hz']==0


def test_adapter_initialization_verifies_existing_handle_only():
    from rm_teleop.rm_api2_motion_adapter import RmApi2MotionAdapter
    from unittest.mock import Mock
    adapter=object.__new__(RmApi2MotionAdapter)
    arm=Mock(spec=['rm_set_modbus_mode','rm_get_tool_rs485_mode'])
    adapter.arm=arm
    arm.rm_set_modbus_mode.return_value=0
    arm.rm_get_tool_rs485_mode.return_value=dict(return_code=0,mode=1,baudrate=115200,timeout=1)
    assert adapter.initialize_o7_modbus()['success']
    arm.rm_set_modbus_mode.assert_called_once_with(1,115200,1)
    arm.rm_get_tool_rs485_mode.return_value=dict(return_code=0,mode=1,baudrate=115200,timeout=2)
    assert not adapter.initialize_o7_modbus()['success']
    arm.rm_get_tool_rs485_mode.reset_mock();arm.rm_set_modbus_mode.return_value=1
    assert not adapter.initialize_o7_modbus()['success']
    arm.rm_get_tool_rs485_mode.assert_not_called()


def test_repeated_write_failures_recover_and_transport_wait_keeps_request():
    now=[100.];unavailable=[False];inits=[];calls=[];events=[]
    adapter=SimpleNamespace(arm=object(),handle=SimpleNamespace(id=7),
                            write_o7_hil_positions=lambda v:(calls.append(v),1)[1])
    def initialize(*args):
        if unavailable[0]:return None  # Only unavailable shared transport postpones init.
        inits.append(now[0]);return dict(success=True)
    with patch.object(HandController,'_solve_loop',lambda self:None):
        c=HandController(Mapper(),lambda:adapter,lambda:'',events.append,start_from_parameter=True,initializer=initialize)
    c.state.clock=lambda:now[0]
    thread=threading.Thread(target=c.write_loop);thread.start()
    try:
        wait_for(lambda:c.state.enabled);assert len(inits)==1
        old_epoch=c.state.epoch
        for attempt,delay in enumerate((1.,2.,4.),start=2):
            unavailable[0]=True;offer(c,.7);wait_for(lambda:len(calls)==attempt-1)
            now[0]+=delay;time.sleep(.04)
            assert len(inits)==attempt-1 and not c.state.enabled and c.state.startup_pending
            assert c.state.pending is None
            unavailable[0]=False;wait_for(lambda:len(inits)==attempt and c.state.enabled)
            time.sleep(.03);assert len(calls)==attempt-1  # No target replay.
        offer(c,.8);wait_for(lambda:c.state.fault_latched)
        assert len(inits)==4 and len(calls)==4
        adapter.handle.id=8;now[0]+=120;time.sleep(.03)
        assert len(inits)==4 and not c.state.enabled  # Handle changes cannot clear latch.
        c.state.offer(old_epoch,now[0],dict(valid=True,q_by_name={}))
        assert c.state.pending is None
    finally:c.shutdown();thread.join(1)


def test_initialization_failure_automatically_retries_without_enable_service():
    now=[100.];inits=[];events=[]
    adapter=SimpleNamespace(write_o7_hil_positions=lambda v:0)
    def initialize(*args):inits.append(now[0]);return dict(success=len(inits)>1)
    with patch.object(HandController,'_solve_loop',lambda self:None):
        c=HandController(Mapper(),lambda:adapter,lambda:'',events.append,start_from_parameter=True,initializer=initialize)
    c.state.clock=lambda:now[0]
    thread=threading.Thread(target=c.write_loop);thread.start()
    try:
        wait_for(lambda:len(inits)==1);assert not c.state.enabled
        now[0]+=1;wait_for(lambda:c.state.enabled)
        assert len(inits)==2 and c.state.pending is None
    finally:c.shutdown();thread.join(1)


def test_same_adapter_new_handle_value_requires_initialization():
    now=[100.];inits=[]
    adapter=SimpleNamespace(arm=object(),handle=SimpleNamespace(id=1))
    with patch.object(HandController,'_solve_loop',lambda self:None):
        c=HandController(Mapper(),lambda:adapter,lambda:'',lambda e:None,start_from_parameter=True,
                         initializer=lambda *a:(inits.append(a),dict(success=True))[1])
    c.state.clock=lambda:now[0]
    thread=threading.Thread(target=c.write_loop);thread.start()
    try:
        wait_for(lambda:c.state.enabled);adapter.handle.id=2;now[0]+=2
        wait_for(lambda:len(inits)==2 and c.state.enabled)
    finally:c.shutdown();thread.join(1)
