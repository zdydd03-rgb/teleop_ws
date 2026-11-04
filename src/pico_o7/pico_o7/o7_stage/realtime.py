"""Latest-value hand pipeline. All solver/native calls run outside ROS callbacks."""
import json
import os
from pathlib import Path
import select
import subprocess
import threading
import time
from .hand_input import input_valid
from .mapping import Mapper
from .write_timing import call_position_write

from pico_o7.paths import share_dir, solver_python as default_solver_python
ROOT=share_dir()
RECOVERABLE_CONTROL_REASONS=frozenset(('control_not_ACTIVE_sending','CANFD_not_continuously_sending'))

class LatestHand:
    """Admission, invalidation, rate limiting, and one-time consumption, SDK independent."""
    def __init__(self,mapper,clock=time.monotonic):
        self.mapper=mapper;self.clock=clock;self.lock=threading.RLock()
        self.enabled=False;self.startup_pending=False;self.resume_pending=False;self.epoch=0;self.raw=None;self.pending=None;self.closed=False;self.accept_after=float('inf')
        self.last_received=None;self.last_submitted=None;self.last_target=list(mapper.config['initial_hold'])
        self.started=clock();self.inflight=False;self.reason='default_disabled'
        self.counts={k:0 for k in ('input','valid_input','solved','invalid','raw_overwritten','target_overwritten','pre_enable_dropped','stale_dropped','obsolete_result_dropped','identical_dropped','cancelled_before_call','writes','calls_returned','call_exceptions')}
        self.return_codes={}
        self.last_code=None;self.last_duration_ms=None;self.last_solve_ms=None
    def clear(self,reason,disable=False):
        with self.lock:
            self.epoch+=1;self.raw=None;self.pending=None;self.last_received=None;self.reason=reason
            if disable:self.enabled=False;self.startup_pending=False;self.resume_pending=False
    def reject_control(self,reason):
        """Pause ordinary ARM enable/tracking interruptions; never clear a fault."""
        with self.lock:
            self.clear(reason,disable=True)
            if reason in RECOVERABLE_CONTROL_REASONS and not self.closed:
                self.startup_pending=True;self.resume_pending=True
    def request_startup(self):
        # Only the launch-selected controller requests this once. No service gate.
        with self.lock:
            if self.closed or self.enabled or self.inflight:raise RuntimeError('invalid startup state')
            self.startup_pending=True;self.resume_pending=False;self.reason='startup_waiting_for_safe_session'
    def enable(self):
        with self.lock:
            if self.closed:return False,'node closing'
            if self.inflight:return False,'native write still in flight; wait for finish'
            self.clear('waiting_for_NEW_frame')
            self.enabled=True;self.startup_pending=False;self.resume_pending=False;self.accept_after=self.clock()
            return True,'enabled; no cached target submitted; waiting for NEW frame'
    def receive(self,e):
        with self.lock:
            self.counts['input']+=1
            if not self.enabled:return
            received=e.get('received_monotonic');now=self.clock()
            if type(received)not in (float,int)or not 0<=now-received<=self.mapper.config['max_age_sec']:
                self.counts['stale_dropped']+=1;self.clear('stale_input');return
            if received<self.accept_after:
                self.counts['pre_enable_dropped']+=1;return
            if not e.get('valid')or not input_valid(e.get('payload')):
                self.counts['invalid']+=1;self.clear('tracking_invalid');return
            if e.get('sequence_status')in ('reset_after_gap','source_restart'):
                self.clear('source_sequence_reset')
            self.counts['valid_input']+=1
            if self.raw is not None:self.counts['raw_overwritten']+=1
            self.last_received=received
            self.raw=(self.epoch,received,e['payload'])
    def take_raw(self):
        with self.lock:
            raw=self.raw;self.raw=None
            return raw
    def offer(self,epoch,received,result):
        with self.lock:
            if not self.enabled or epoch!=self.epoch:
                self.counts['obsolete_result_dropped']+=1;return
            if self.clock()-received>self.mapper.config['max_age_sec']:
                self.counts['stale_dropped']+=1;self.clear('stale_solver_result');return
            if not result.get('valid'):
                self.counts['invalid']+=1;self.clear('solver_rejected: '+str(result.get('error')));return
            try:self.mapper.convert(result['q_by_name'])
            except (ValueError,KeyError,TypeError)as exc:
                self.clear('invalid_solution: '+str(exc));return
            self.counts['solved']+=1;self.last_solve_ms=result.get('solve_ms')
            if self.pending is not None:self.counts['target_overwritten']+=1
            self.pending=(epoch,received,result)
            self.reason='tracking_valid'
    def claim(self,guard=lambda:''):
        with self.lock:
            now=self.clock()
            if not self.enabled:return None
            if self.last_received is not None and now-self.last_received>self.mapper.config['max_age_sec']:
                self.counts['stale_dropped']+=1;self.clear('tracking_timeout');return None
            if self.inflight or self.pending is None:return None
            if self.last_submitted is not None and now-self.last_submitted<1/self.mapper.config['write_hz']:return None
            epoch,received,result=self.pending
            if epoch!=self.epoch or now-received>self.mapper.config['max_age_sec']:
                self.counts['stale_dropped']+=1;self.clear('stale_target');return None
            reason=guard()
            if reason:self.reject_control(reason);return None
            self.pending=None
            target,_=self.mapper.convert(result['q_by_name'],self.last_target)
            if self.last_submitted is not None and target==self.last_target:
                self.counts['identical_dropped']+=1;return None
            # Submission boundary. Disable cannot recall a native write already claimed.
            self._claim_previous=(list(self.last_target),self.last_submitted)
            self.last_target=target;self.last_submitted=now;self.inflight=True;self.counts['writes']+=1
            return dict(epoch=epoch,received_monotonic=received,seq=result.get('seq'),
                        timestamp=result.get('timestamp'),q_by_name=result['q_by_name'],target_positions=target,
                        command_sequence=self.counts['writes'])
    def authorize_call(self,claimed,guard=lambda:''):
        with self.lock:
            reason=guard()
            if self.enabled and claimed['epoch']==self.epoch and not reason and self.clock()-claimed['received_monotonic']<=self.mapper.config['max_age_sec']:
                return True
            self.last_target,self.last_submitted=self._claim_previous
            self.inflight=False;self.counts['writes']-=1;self.counts['cancelled_before_call']+=1
            if reason:self.reject_control(reason)
            return False
    def finish(self,code,duration_ms,error=None):
        with self.lock:
            self.inflight=False;self.last_code=code;self.last_duration_ms=duration_ms
            # Raw SDK code is diagnostic ONLY: no success inference, gate, or retry.
            key=str(code);self.return_codes[key]=self.return_codes.get(key,0)+1
            self.counts['calls_returned'if error is None else'call_exceptions']+=1
            if error is not None:self.clear('native_exception_requires_restart',disable=True)
            elif duration_ms>self.mapper.config['max_call_ms']:self.clear('slow_call_breaker_requires_restart',disable=True)
            return 'call_returned_physical_result_unverified'if error is None else'call_exception'
    def status(self):
        with self.lock:
            elapsed=max(.001,self.clock()-self.started)
            return dict(enabled=self.enabled,startup_pending=self.startup_pending,resume_pending=self.resume_pending,epoch=self.epoch,reason=self.reason,inflight=self.inflight,
                        pending=self.pending is not None,last_target=self.last_target,last_code=self.last_code,
                        last_duration_ms=self.last_duration_ms,last_solve_ms=self.last_solve_ms,
                        input_hz=self.counts['input']/elapsed,write_hz=self.counts['writes']/elapsed,
                        frequency_basis='since node startup; cumulative',counts=dict(self.counts),
                        raw_return_code_counts=dict(self.return_codes),physical_write_result='requires_field_observation')

class HandController:
    def __init__(self,mapper,adapter_getter,guard,emit,*,start_from_parameter=False,solver_python=None):
        self.solver_python=str(solver_python or default_solver_python())
        self.state=LatestHand(mapper);self.adapter_getter=adapter_getter;self.guard=guard;self.emit=emit
        if start_from_parameter:self.state.request_startup()
        self.stop=threading.Event();self.process=None
        self.diagnostic=None;self.diagnostic_used=False
        self.solver_thread=threading.Thread(target=self._solve_loop,name='o7_solver_bridge',daemon=False)
        self.solver_thread.start()
    def event(self,event,**v):
        self.emit(dict(event=event,mode='pico_o7_realtime',timestamp_ns=time.time_ns(),**v))
    def schedule_latency_once(self,plan,request_id,guard):
        """Known small target, original writer slot, PICO submissions kept disabled."""
        with self.state.lock:
            if self.stop.is_set()or self.state.closed:return False,'node closing / solver unavailable'
            if self.state.enabled or self.state.startup_pending:return False,'parameter-selected following must not share a fixed-target comparison'
            if self.state.inflight or self.diagnostic is not None:return False,'O7 worker busy'
            if self.diagnostic_used:return False,'fixed-target attempt consumed; no automatic retry'
            reason=guard()
            if reason:return False,reason
            try:plan.validate()
            except ValueError as exc:return False,str(exc)
            self.state.clear('fixed_target_latency_comparison',disable=True)
            self.diagnostic_used=True;self.state.inflight=True
            self.diagnostic=(plan,request_id,guard,self.state.epoch)
            return True,'queued ONE known small target; PICO target submissions remain disabled'
    def _run_latency_once(self,job):
        from .worker import run_index_once
        plan,request_id,guard,epoch=job
        def check():
            with self.state.lock:
                if self.stop.is_set()or self.state.epoch!=epoch:return 'fixed target cancelled / node closing'
                if self.state.enabled:return 'unexpected hand enable'
            return guard()
        try:
            def emit(e):self.emit(dict(e,comparison_environment='hand_start_ARM_locked_fixed_target'))
            run_index_once(self.adapter_getter(),plan,emit,check,request_id=request_id,validated=True)
        finally:
            with self.state.lock:
                self.state.inflight=False
                # Do not enable normal following after a diagnostic without restarting:
                # its step anchor is deliberately untouched by this comparison.
                self.state.closed=True
                self.state.reason='fixed_target_comparison_finished_restart_before_normal_follow'
    def _solve_loop(self):
        try:
            env=dict(os.environ);env.pop('PYTHONPATH',None);env.pop('PYTHONHOME',None)
            env['PYTHONPATH']=str(Path(__file__).resolve().parents[2])
            env['PICO_O7_SHARE']=str(ROOT)
            self.process=subprocess.Popen([self.solver_python,'-m','pico_o7.solver_process'],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1,env=env)
            epoch=None
            while not self.stop.is_set():
                raw=self.state.take_raw()
                if raw is None:self.stop.wait(.01);continue
                generation,received,payload=raw
                self.process.stdin.write(json.dumps(dict(payload=payload,reset=generation!=epoch),allow_nan=False)+'\n');self.process.stdin.flush()
                deadline=time.monotonic()+5
                while not self.stop.is_set():
                    ready,_,_=select.select([self.process.stdout],[],[],.05)
                    if ready:break
                    if time.monotonic()>deadline:raise RuntimeError('solver response deadline exceeded')
                if self.stop.is_set():break
                line=self.process.stdout.readline()
                if not line:raise RuntimeError('solver exited')
                result=json.loads(line);epoch=generation
                self.state.offer(generation,received,result)
        except Exception as exc:
            self.state.clear('solver_process_failure',disable=True)
            with self.state.lock:self.state.closed=True
            self.event('O7_SOLVER_FAILURE',error=repr(exc))
        finally:
            if self.process is not None:
                self.process.terminate()
                try:self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
                self.process.stdin.close();self.process.stdout.close()
    def write_loop(self):
        # Runs in the inherited SingleSessionHil.worker slot, not a ROS timer.
        while not self.stop.is_set():
            with self.state.lock:
                diagnostic=self.diagnostic;self.diagnostic=None
            if diagnostic is not None:
                self._run_latency_once(diagnostic)
                continue
            started=False;resumed=False
            with self.state.lock:
                if self.state.startup_pending:
                    resumed=self.state.resume_pending
                    reason=self.guard()
                    if reason:
                        if resumed and reason not in RECOVERABLE_CONTROL_REASONS:
                            self.state.clear(reason,disable=True)
                        else:self.state.reason=('resume_wait:' if resumed else 'startup_wait:')+reason
                    else:started,_=self.state.enable()
            if started:self.event('O7_HAND_RESUME' if resumed else 'O7_HAND_PARAMETER_START',status=self.state.status())
            # Poll even with an empty mailbox; every resumption invalidates cached
            # solver work and requires new tracking received after the safe gate.
            with self.state.lock:
                if self.state.enabled:
                    reason=self.guard()
                    if reason:
                        self.state.reject_control(reason)
                        self.event('O7_HAND_PAUSE' if self.state.resume_pending else 'O7_HAND_SAFETY_STOP',reason=reason)
            claimed=self.state.claim(self.guard)
            if claimed is None:self.stop.wait(.01);continue
            code=None;error=None
            self.event('O7_HAND_WRITE_START',**claimed)
            if not self.state.authorize_call(claimed,self.guard):
                self.event('O7_HAND_WRITE_CANCELLED',seq=claimed['seq'],reason=self.state.reason)
                continue
            timing=call_position_write(self.adapter_getter(),claimed['target_positions'])
            category=self.state.finish(timing['return_code'],timing['duration_ms'],timing['error'])
            self.event('O7_HAND_WRITE_FINISH',**claimed,**timing,category=category,status=self.state.status())
    def shutdown(self):
        self.state.clear('node_closing',disable=True)
        with self.state.lock:self.state.closed=True
        self.stop.set()
        self.solver_thread.join(timeout=6)
        if self.solver_thread.is_alive():raise RuntimeError('solver thread failed to stop')
