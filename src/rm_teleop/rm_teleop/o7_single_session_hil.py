"""One-shot lifecycle owner. No SDK lock is shared with CANFD."""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time


class SingleSessionHil:
    def __init__(self, emit):
        self.emit = emit
        self.lock = threading.Lock()
        self.history = deque(maxlen=250)
        self.used = False
        self.closing = False
        self.worker = None
        self.stream_cancel = threading.Event()
        self.streaming = False
        self.auto_streaming = False

    def record(self, now, active):
        with self.lock:
            gap = now - self.history[-1][0] if self.history else 0.0
            self.history.append((now, active))
            if self.streaming and (not active or (self.auto_streaming and gap > .010)):
                self.stream_cancel.set()

    def trigger(self, enabled, adapter, rejection="", *, stream_config=None, safety=None, auto_config=None):
        with self.lock:
            if not enabled:
                return False, ("enable_o7_auto_stream_hil=false" if auto_config is not None else
                               "enable_o7_stream_hil=false" if stream_config is not None else
                               "enable_o7_single_session_hil=false")
            if self.closing:
                return False, "node_closing"
            if self.used:
                return False, "HIL already accepted (running or completed); restart required"
            if rejection:
                return False, rejection
            now = time.monotonic()
            recent = [(t, ok) for t, ok in self.history if now - t <= 1.01]
            if (len(recent) < 180 or now - recent[0][0] < 0.99
                    or now - recent[-1][0] > 0.010
                    or not all(ok for _, ok in recent)
                    or any(b[0] - a[0] > 0.010 for a, b in zip(recent, recent[1:]))):
                return False, "requires 1s continuous ACTIVE CANFD at ~200Hz, no gap >10ms"
            if stream_config is not None and auto_config is not None:
                return False, "only one HIL mode may be selected"
            configuration = auto_config if auto_config is not None else stream_config
            if configuration is not None:
                try:
                    configuration.validate()
                except ValueError as exc:
                    return False, str(exc)
                if safety is None:
                    return False, "missing continuous safety guard"
            self.used = True  # Even errors consume the attempt: never retry an uncertain write.
            self.streaming = configuration is not None
            self.auto_streaming = auto_config is not None
            target = self._run_auto if self.auto_streaming else self._run_stream if self.streaming else self._run
            args = (adapter, configuration, safety) if self.streaming else (adapter,)
            self.worker = threading.Thread(target=target, args=args,
                                           name="o7_auto_modbus_writer" if self.auto_streaming else
                                                "o7_stream_hil" if self.streaming else "o7_single_session_hil",
                                           daemon=False)
            try:
                self.worker.start()
            except Exception as exc:
                self.worker = None
                return False, f"worker_start_failed:{exc} (attempt consumed)"
            return True, "accepted; execution result is reported on /rm65/o7_modbus_hil/event"

    def _run(self, adapter):
        # Keep the adapter/arm alive until the native call returns.
        adapter.write_o7_hil_speed_once(self.emit)

    def _run_stream(self, adapter, config, safety):
        def guard():
            if self.stream_cancel.is_set():
                return "control_left_ACTIVE_or_node_closing"
            reason = safety()
            with self.lock:
                if not reason and (not self.history or time.monotonic() - self.history[-1][0] > .05):
                    reason = "CANFD_control_cycle_stale"
            return reason
        run_position_stream(adapter, config, self.emit, guard, self.stream_cancel.wait)

    def _run_auto(self, adapter, config, safety):
        from .o7_auto_stream_hil import AutoStreamRunner

        def guard():
            reason = safety()
            if reason:
                return reason
            with self.lock:
                if self.stream_cancel.is_set():
                    return "control_left_ACTIVE_CANFD_gap_or_node_closing"
                if not self.history or time.monotonic() - self.history[-1][0] > .010:
                    return "CANFD_control_cycle_stale"
            return ""
        AutoStreamRunner(adapter, config, self.emit, guard).run()

    def begin_close(self):
        with self.lock:
            self.closing = True
            self.stream_cancel.set()

    def wait_closed(self, warn):
        with self.lock:
            worker = self.worker
        if worker is not None:
            worker.join(timeout=2.0)
            if worker.is_alive():
                warn("O7 HIL SDK call still blocked after 2s; retaining API2 handle and waiting. "
                     "Do not force handle deletion; use physical safety controls if needed.")
                worker.join()  # Cannot safely terminate a native SDK call in Python.


@dataclass(frozen=True)
class StreamConfig:
    hz: float = 1.0
    duration_sec: float = 10.0
    joint: int = 2
    open_pose_confirmed: bool = False

    def validate(self):
        # Only the explicitly authorized first stage is implemented.
        if type(self.hz) not in (int, float) or not math.isfinite(self.hz) or self.hz != 1.0:
            raise ValueError("o7_stream_hz must be 1.0 for this HIL stage")
        if (type(self.duration_sec) not in (int, float) or not math.isfinite(self.duration_sec)
                or not 2.0 <= self.duration_sec <= 10.0):
            raise ValueError("o7_stream_duration_sec must be in [2, 10]")
        if type(self.joint) is not int or self.joint != 2:
            raise ValueError("o7_stream_joint must be 2 (index pitch)")
        if self.open_pose_confirmed is not True:
            raise ValueError("o7_stream_open_pose_confirmed=true required: confirm O7 safely open, "
                             "no object contact, all seven fixed targets safe")


class StreamAbort(RuntimeError):
    pass


# Fixed targets for this diagnostic only. All seven targets are written each time.
POSITION_A = (255, 255, 220, 255, 255, 255, 255)
POSITION_B = (255, 255, 255, 255, 255, 255, 255)
ACTUAL_MOTION = "NOT AUTOMATICALLY VERIFIED"


def run_position_stream(adapter, config, emit, guard, wait, *, clock=time.monotonic):
    """Write-only, bounded diagnostic. Nonzero SDK results never cause retries."""
    config.validate()
    started = clock()
    deadline = started + config.duration_sec
    period = 1.0 / config.hz
    sequence = 0
    write_calls = 0
    acknowledged = 0
    unconfirmed = 0
    exceptions = 0
    failures = 0
    last_target = None
    last_code = None
    reason = "duration_complete"

    def event(name, **values):
        emit(dict(event=name, timestamp_ns=time.time_ns(), elapsed_time=clock() - started,
                  command_sequence=sequence, sequence=sequence, joint_index=config.joint,
                  consecutive_failures=failures, unconfirmed_writes=unconfirmed,
                  session="existing_rm65_api2_handle", **values))

    def check():
        problem = guard()
        if problem:
            raise StreamAbort(problem)
        if clock() >= deadline:
            raise StreamAbort("duration_complete")

    def wait_until(due):
        while clock() < due:
            check()
            wait(min(.02, due - clock()))
        check()

    def write(target):
        nonlocal sequence, write_calls, acknowledged, unconfirmed, exceptions, failures
        nonlocal last_target, last_code
        check()
        sequence += 1
        metadata = dict(target_position=target[config.joint], target_positions=list(target))
        event("POSITION_WRITE_START", start_timestamp=time.time_ns(), end_timestamp=None,
              return_code=None, duration_ms=None,
              modbus_return_code=None, modbus_duration_ms=None, **metadata)
        try:
            check()  # Publishing START may have yielded to a fault or shutdown.
        except StreamAbort as exc:
            event("POSITION_WRITE_CANCELLED", reason=str(exc), **metadata)
            raise
        write_calls += 1
        last_target = list(target)
        call_start = time.perf_counter()
        start_ns = time.time_ns()
        code = None
        error = None
        try:
            code = adapter.write_o7_hil_positions(target)
        except Exception as exc:
            exceptions += 1
            failures += 1
            error = exc
        ended = time.perf_counter()
        end_ns = time.time_ns()
        last_code = code
        if error is None:
            acknowledged += int(code == 0)
            unconfirmed += int(code != 0)
            failures = failures + 1 if code != 0 else 0
        # FINISH means the call returned/raised, not that O7 moved or acknowledged.
        event("POSITION_WRITE_FINISH", return_code=code, modbus_return_code=code,
              duration_ms=(ended - call_start) * 1000,
              modbus_duration_ms=(ended - call_start) * 1000,
              modbus_start_timestamp=call_start, modbus_end_timestamp=ended,
              start_timestamp=start_ns, end_timestamp=end_ns,
              call_start_timestamp_ns=start_ns, call_end_timestamp_ns=end_ns,
              sdk_acknowledged=error is None and code == 0,
              error=repr(error) if error is not None else None, **metadata)
        if error is not None:
            raise error
        # No nonzero-code stop or resend: next command belongs to the next absolute slot.
        check()

    event("STREAM_START", stream_hz=config.hz, duration_sec=config.duration_sec,
          position_A=list(POSITION_A), position_B=list(POSITION_B),
          open_pose_confirmed=config.open_pose_confirmed, actual_motion=ACTUAL_MOTION)
    try:
        slot = 0
        while True:
            due = started + slot * period
            wait_until(min(due, deadline))
            # A late wakeup also selects the current slot, never an old target.
            slot = max(slot, math.floor((clock() - started) / period))
            target = POSITION_A if slot % 2 == 0 else POSITION_B
            write(target)
            # Discard elapsed slots; no catch-up writes and no retry of an uncertain command.
            slot = max(slot + 1, math.floor((clock() - started) / period) + 1)
    except StreamAbort as exc:
        reason = str(exc)
    except Exception as exc:
        reason = "sdk_exception"
        event("STREAM_EXCEPTION", error=repr(exc))
    finally:
        results = dict(last_target_positions=last_target, last_return_code=last_code,
                       sdk_acknowledged_success_count=acknowledged,
                       sdk_unconfirmed_command_count=unconfirmed,
                       sdk_exception_count=exceptions, actual_motion=ACTUAL_MOTION)
        event("STREAM_STOP", reason=reason, **results)
        event("STREAM_FINISH", reason=reason, schedule_completed=reason == "duration_complete",
              position_write_count=write_calls, **results)
