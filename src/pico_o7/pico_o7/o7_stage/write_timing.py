"""Identical Adapter call timing for all comparisons; no return-code decisions."""
import contextlib
from datetime import datetime,timezone
import fcntl
import time
from pathlib import Path

from pico_o7.paths import workspace_root
ROOT=workspace_root()

@contextlib.contextmanager
def session_lease():
    folder=ROOT/'concurrency_logs/pico_o7';folder.mkdir(parents=True,exist_ok=True)
    with (folder/'o7_session.lock').open('a')as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Another project API2 entry is running; stop it before this comparison')
        try:yield
        finally:fcntl.flock(lock,fcntl.LOCK_UN)

def call_position_write(adapter,target):
    """One call, zero reads/retries. Includes Adapter validation/encoding + SDK."""
    code=None;error=None
    start_ns=time.time_ns();start_perf_ns=time.perf_counter_ns()
    try:code=adapter.write_o7_hil_positions(target)
    except (Exception,KeyboardInterrupt) as exc:error=repr(exc)
    end_perf_ns=time.perf_counter_ns();end_ns=time.time_ns()
    return dict(return_code=code,error=error,duration_ms=(end_perf_ns-start_perf_ns)/1e6,
        call_start_utc=datetime.fromtimestamp(start_ns/1e9,timezone.utc).isoformat(),
        call_end_utc=datetime.fromtimestamp(end_ns/1e9,timezone.utc).isoformat(),
        call_start_timestamp_ns=start_ns,call_end_timestamp_ns=end_ns,
        call_start_monotonic_ns=start_perf_ns,call_end_monotonic_ns=end_perf_ns,
        timing_scope='Adapter.write_o7_hil_positions: validation+encoding+SDK; excludes solver/queue/logging',
        physical_write_result='requires_field_observation',automatic_retry=False)
