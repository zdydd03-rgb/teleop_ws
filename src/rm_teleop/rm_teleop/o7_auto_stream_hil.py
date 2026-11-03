"""Bounded automatic HIL: independent time-based targets, one serial SDK writer."""
from dataclasses import dataclass
import math
import threading
import time

from .o7_single_session_hil import ACTUAL_MOTION


@dataclass(frozen=True)
class AutoStreamConfig:
    hz: float = 10.0
    timeout_sec: float = 60.0
    open_pose_confirmed: bool = False

    def validate(self):
        if type(self.hz) not in (float, int) or not math.isfinite(self.hz) or self.hz != 10.0:
            raise ValueError('o7_auto_stream_hz must be 10.0 for this HIL')
        if (type(self.timeout_sec) not in (float, int) or not math.isfinite(self.timeout_sec)
                or not 0.1 <= self.timeout_sec <= 60.0):
            raise ValueError('o7_auto_stream_timeout_sec must be in [0.1, 60]')
        if self.open_pose_confirmed is not True:
            raise ValueError('o7_auto_stream_open_pose_confirmed=true required: safely open, '
                             'no object contact, all seven targets safe')


def triangle_positions(elapsed):
    """One second flexion, one second extension; all other targets are 255."""
    phase = max(0.0, elapsed) % 2.0
    pitch = 255.0 - 35.0 * (phase if phase <= 1.0 else 2.0 - phase)
    return (255, 255, int(round(pitch)), 255, 255, 255, 255)


def observed_hz(times):
    return ((len(times) - 1) / (times[-1] - times[0])
            if len(times) >= 2 and times[-1] > times[0] else None)


@dataclass(frozen=True)
class GeneratedTarget:
    generation_sequence: int
    generated_monotonic: float
    target_positions: tuple


class LatestTargetMailbox:
    """One pending value only. The lock never spans an SDK call or event publication."""
    def __init__(self):
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.reason = None
        self.overwritten = 0
        self.discarded = 0
        self.selected_cancelled = 0
        self.skipped_slots = 0
        self.generation_times = []
        self.write_times = []
        self.last_target = None

    def put(self, positions, now, skipped_slots=0):
        with self.condition:
            if self.closed:
                return None
            self.overwritten += int(self.pending is not None)
            self.skipped_slots += skipped_slots
            self.generation_times.append(now)
            target = GeneratedTarget(len(self.generation_times), now, tuple(positions))
            self.pending = target
            self.condition.notify_all()
            return target

    def take(self, timeout=.02):
        with self.condition:
            if self.pending is None and not self.closed:
                self.condition.wait(timeout)
            if self.closed:
                return None
            target, self.pending = self.pending, None
            return target

    def close(self, reason):
        with self.condition:
            if self.closed:
                return False
            self.closed = True
            self.reason = reason
            self.discarded += int(self.pending is not None)
            self.pending = None
            self.condition.notify_all()
            return True

    def claim_write(self, target, now, deadline):
        with self.condition:
            if self.closed or now >= deadline:
                self.selected_cancelled += 1
                return False
            self.write_times.append(now)
            self.last_target = target.target_positions
            return True

    def cancel_selected(self):
        with self.condition:
            self.selected_cancelled += 1

    def snapshot(self):
        with self.condition:
            return dict(target_generation_count=len(self.generation_times),
                        position_write_count=len(self.write_times),
                        target_generation_hz=observed_hz(self.generation_times),
                        actual_modbus_write_hz=observed_hz(self.write_times),
                        overwritten_target_count=self.overwritten,
                        skipped_generation_slots=self.skipped_slots,
                        discarded_pending_target_count=self.discarded,
                        cancelled_selected_target_count=self.selected_cancelled,
                        last_target_positions=list(self.last_target) if self.last_target else None)


class AutoStreamRunner:
    """run() is the sole SDK writer; the child generator never calls the adapter."""
    def __init__(self, adapter, config, emit, safety, *, clock=time.monotonic):
        self.adapter = adapter
        self.config = config
        self.emit = emit
        self.safety = safety
        self.clock = clock
        self.mailbox = LatestTargetMailbox()
        self.halt = threading.Event()
        self.first_selected = threading.Event()
        self.started = clock()
        self.deadline = self.started + config.timeout_sec
        self.generator = None
        self.sequence = 0
        self.acknowledged = 0
        self.unconfirmed = 0
        self.exceptions = 0
        self.last_code = None

    def event(self, name, **values):
        self.emit(dict(event=name, timestamp_ns=time.time_ns(), mode='auto_stream',
                       elapsed_time=self.clock() - self.started, joint_index=2,
                       session='existing_rm65_api2_handle', **values))

    def stop(self, reason):
        if self.mailbox.close(reason):
            self.halt.set()
            self.first_selected.set()
            # STOP marks admission closed, even when a native call is still in flight.
            self.event('AUTO_STREAM_STOP', reason=reason, **self.mailbox.snapshot())

    def check(self):
        if self.halt.is_set():
            return False
        reason = self.safety()
        if reason:
            self.stop(reason)
            return False
        if self.clock() >= self.deadline:
            self.stop('timeout_60s_limit' if self.config.timeout_sec == 60 else 'timeout_limit')
            return False
        return True

    def generate(self):
        try:
            if not self.check():
                return
            # Reserve the first 255 target until the sole writer selects it.
            self.publish_target((255,) * 7, self.clock())
            while not self.first_selected.wait(.02):
                if not self.check():
                    return
            period = 1.0 / self.config.hz
            slot = 1
            while self.check():
                due = self.started + slot * period
                remaining = min(due, self.deadline) - self.clock()
                if remaining > 0:
                    self.halt.wait(min(.02, remaining))
                    continue
                if not self.check():
                    return
                now = self.clock()
                current_slot = max(slot, math.floor((now - self.started) / period + 1e-9))
                self.publish_target(triangle_positions(now - self.started), now,
                                    skipped_slots=current_slot - slot)
                slot = current_slot + 1
        except Exception as exc:
            self.event('AUTO_STREAM_EXCEPTION', error=repr(exc), source='target_generator')
            self.stop('target_generator_exception')

    def publish_target(self, positions, now, skipped_slots=0):
        target = self.mailbox.put(positions, now, skipped_slots)
        if target is not None:
            self.event('TARGET_GENERATED', generation_sequence=target.generation_sequence,
                       generated_monotonic=target.generated_monotonic,
                       target_positions=list(positions), target_position=positions[2],
                       **{k: v for k, v in self.mailbox.snapshot().items()
                          if k != 'last_target_positions'})

    def write(self, target):
        self.sequence += 1
        metadata = dict(sequence=self.sequence, command_sequence=self.sequence,
                        generation_sequence=target.generation_sequence,
                        generated_monotonic=target.generated_monotonic,
                        target_age_sec=max(0.0, self.clock() - target.generated_monotonic),
                        target_positions=list(target.target_positions), target_position=target.target_positions[2])
        self.event('POSITION_WRITE_START', start_timestamp=time.time_ns(), end_timestamp=None,
                   return_code=None, duration_ms=None, **metadata)
        if not self.check():
            self.mailbox.cancel_selected()
            self.event('POSITION_WRITE_CANCELLED', **metadata)
            return
        if not self.mailbox.claim_write(target, self.clock(), self.deadline):
            self.stop('timeout_limit')
            self.event('POSITION_WRITE_CANCELLED', **metadata)
            return
        # This claim is the submission boundary. No mailbox/SDK lock spans the native call.
        self.first_selected.set()
        started = time.perf_counter()
        start_ns = time.time_ns()
        code, error = None, None
        try:
            code = self.adapter.write_o7_hil_positions(target.target_positions)
        except Exception as exc:
            error = exc
        ended = time.perf_counter()
        end_ns = time.time_ns()
        self.last_code = code
        if error is not None:
            self.exceptions += 1
            self.stop('sdk_exception')  # Prohibit new submissions before logging the result.
        else:
            self.acknowledged += int(code == 0)
            self.unconfirmed += int(code != 0)
        self.event('POSITION_WRITE_FINISH', start_timestamp=start_ns, end_timestamp=end_ns,
                   call_start_timestamp_ns=start_ns, call_end_timestamp_ns=end_ns,
                   modbus_start_timestamp=started, modbus_end_timestamp=ended,
                   duration_ms=(ended - started) * 1000, modbus_duration_ms=(ended - started) * 1000,
                   return_code=code, modbus_return_code=code,
                   sdk_acknowledged=error is None and code == 0,
                   error=repr(error) if error else None,
                   actual_modbus_write_hz=self.mailbox.snapshot()['actual_modbus_write_hz'], **metadata)
        if error is not None:
            self.event('AUTO_STREAM_EXCEPTION', error=repr(error), source='modbus_writer')

    def run(self):
        self.config.validate()
        self.event('AUTO_STREAM_START', configured_target_generation_hz=self.config.hz,
                   timeout_sec=self.config.timeout_sec, triangle_period_sec=2.0,
                   start_monotonic=self.started, deadline_monotonic=self.deadline,
                   position_min=220, position_max=255,
                   open_pose_confirmed=self.config.open_pose_confirmed, actual_motion=ACTUAL_MOTION)
        try:
            if self.check():
                self.generator = threading.Thread(target=self.generate, name='o7_auto_target_generator', daemon=False)
                self.generator.start()
            while self.check():
                target = self.mailbox.take()
                if target is not None:
                    self.write(target)
        except Exception as exc:
            self.event('AUTO_STREAM_EXCEPTION', error=repr(exc), source='auto_worker')
            self.stop('auto_worker_exception')
        finally:
            self.stop('auto_worker_exit')
            if self.generator is not None and self.generator.ident is not None:
                self.generator.join()  # Never performed by the 200 Hz control thread.
            self.event('AUTO_STREAM_FINISH', reason=self.mailbox.reason,
                       schedule_completed=self.mailbox.reason in ('timeout_limit', 'timeout_60s_limit'),
                       sdk_acknowledged_success_count=self.acknowledged,
                       sdk_unconfirmed_command_count=self.unconfirmed, sdk_exception_count=self.exceptions,
                       last_return_code=self.last_code, actual_motion=ACTUAL_MOTION,
                       configured_target_generation_hz=self.config.hz, **self.mailbox.snapshot())
