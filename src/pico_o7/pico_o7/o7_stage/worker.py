"""Extend the existing HIL owner: same adapter lifetime and worker slot, no new client."""
import threading
import time
import uuid
from rm_teleop.o7_single_session_hil import SingleSessionHil, ACTUAL_MOTION
from .protocol import read_existing_session
from .write_timing import call_position_write


class StageHil(SingleSessionHil):
    def __init__(self,emit):
        super().__init__(emit)
        self.operation_gate=threading.Lock()

    def trigger(self,*args,**kwargs):
        with self.operation_gate:
            with self.lock:
                if self.worker is not None and self.worker.is_alive():return False,'O7 worker busy'
            self.last_request_id=uuid.uuid4().hex
            return super().trigger(*args,**kwargs)

    def trigger_read(self,adapter,guard,*,index=None):
        with self.operation_gate:
            with self.lock:
                if self.closing:return False,'node closing'
                if self.used:return False,'calibration attempt already consumed; no read/write interleaving'
                if self.worker is not None and self.worker.is_alive():return False,'O7 worker busy'
                reason=guard()
                if reason:return False,reason
                request_id=uuid.uuid4().hex
                self.worker=threading.Thread(target=self._read,args=(adapter,guard,index,request_id),
                                             name='o7_shared_hil_worker',daemon=False)
                try:self.worker.start()
                except Exception as exc:
                    self.worker=None
                    return False,'read worker start failed: '+repr(exc)
                return True,request_id

    def trigger_index_locked(self,adapter,plan,guard):
        """Independent ARM-locked one-shot; original trigger/ACTIVE gate is unchanged."""
        with self.operation_gate:
            with self.lock:
                if self.closing or self.used:return False,'node closing or one-shot already consumed'
                if self.worker is not None and self.worker.is_alive():return False,'O7 worker busy'
                reason=guard()
                if reason:return False,reason
                try:plan.validate()
                except ValueError as exc:return False,str(exc)
                self.used=True;self.last_request_id=uuid.uuid4().hex
                self.worker=threading.Thread(target=run_index_once,
                    args=(adapter,plan,self.emit,guard),kwargs=dict(request_id=self.last_request_id,validated=True),
                    name='o7_shared_index2_once',daemon=False)
                try:self.worker.start()
                except Exception as exc:
                    self.worker=None;return False,'worker start failed; attempt consumed: '+repr(exc)
                return True,'accepted; one write maximum, no automatic return/retry'

    def start_hand_worker(self,controller):
        with self.operation_gate:
            with self.lock:
                if self.closing or self.used:return False,'node closing or other HIL attempt consumed'
                if self.worker is not None and self.worker.is_alive():return False,'O7 worker busy'
                self.used=True
                self.worker=threading.Thread(target=controller.write_loop,name='o7_shared_hand_worker',daemon=False)
                try:self.worker.start()
                except Exception as exc:
                    self.worker=None;return False,'worker start failed; attempt consumed: '+repr(exc)
                return True,'latest-value worker started; execution follows caller startup policy'

    def _read(self,adapter,guard,index,request_id):
        event=dict(request_id=request_id,timestamp_ns=time.time_ns(),session='existing_rm65_api2_handle',
                   mode='read_only',index=index)
        self.emit(dict(event,event='O7_READ_START'))
        try:
            with self.lock:closing=self.closing
            reason='node closing'if closing else guard()
            if reason:raise RuntimeError(reason)
            started=time.perf_counter()
            result=read_existing_session(adapter,index=index)
            result['duration_ms']=(time.perf_counter()-started)*1000
        except Exception as exc:
            result=dict(return_code=None,valid=False,positions=None,error=repr(exc))
        self.emit(dict(event,event='O7_READ_FINISH',timestamp_ns=time.time_ns(),**result))

    def _run_stream(self,adapter,config,safety):
        from .plan import Index2Plan
        if not isinstance(config,Index2Plan):return super()._run_stream(adapter,config,safety)
        def guard():
            if self.stream_cancel.is_set():return 'control_left_ACTIVE_or_node_closing'
            reason=safety()
            with self.lock:
                if not reason and(not self.history or time.monotonic()-self.history[-1][0]>.05):
                    reason='CANFD_control_cycle_stale'
            return reason
        try:
            config.validate()
        except Exception as exc:
            self.emit(dict(event='STREAM_FINISH',mode='index2_once',request_id=self.last_request_id,
                plan_sha256=config.hash,timestamp_ns=time.time_ns(),reason='plan_or_worker_error',
                error=repr(exc),schedule_completed=False,position_write_count=0,
                calls_returned=0,physical_write_result='requires_field_observation',
                sdk_exception_count=0,last_target_positions=None,last_return_code=None))
            return
        return run_index_once(adapter,config,self.emit,guard,request_id=self.last_request_id,validated=True)


def run_index_once(adapter,plan,emit,guard,*,request_id=None,validated=False):
    """Exactly zero or one position write, never a restore/retry/configuration write."""
    if not validated:plan.validate()
    target=plan.target
    common=dict(mode='index2_once',request_id=request_id or uuid.uuid4().hex,joint_index=2,session='existing_rm65_api2_handle',
                plan_sha256=plan.hash,hold_positions=list(plan.baseline),target_positions=list(target),
                target_position=target[2],actual_motion=ACTUAL_MOTION)
    def event(name,**extra):emit(dict(common,event=name,timestamp_ns=time.time_ns(),**extra))
    code=None;error=None;calls=0;reason='cancelled_before_submission';duration=None
    event('STREAM_START',max_write_calls=1,automatic_return_write=False)
    try:
        reason=guard()
        if reason:
            event('POSITION_WRITE_CANCELLED',reason=reason)
            return
        event('POSITION_WRITE_START',command_sequence=1,sequence=1)
        # Event publication may have yielded; recheck before entering the native call.
        reason=guard()
        if reason:
            event('POSITION_WRITE_CANCELLED',reason=reason)
            return
        calls=1
        timing=call_position_write(adapter,target)
        code=timing['return_code'];error=timing['error'];duration=timing['duration_ms']
        event('POSITION_WRITE_FINISH',command_sequence=1,sequence=1,**timing,
              return_code_policy='raw diagnostic only; no inference about physical write/motion',
              modbus_return_code=code,modbus_duration_ms=duration,
              modbus_start_timestamp=timing['call_start_monotonic_ns']/1e9,
              modbus_end_timestamp=timing['call_end_monotonic_ns']/1e9,
              start_timestamp=timing['call_start_timestamp_ns'],end_timestamp=timing['call_end_timestamp_ns'])
        reason='sdk_exception'if error else'single_submission_completed'
    finally:
        event('STREAM_FINISH',reason=reason,position_write_count=calls,schedule_completed=calls==1 and error is None,
              last_return_code=code,last_target_positions=list(target)if calls else None,
              physical_write_result='requires_field_observation',
              calls_returned=int(calls==1 and error is None),sdk_exception_count=int(error is not None))
