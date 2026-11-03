"""ROS-independent capture, exact event phases and statistics."""
import csv
import json
import math
from collections import Counter
from pathlib import Path

CONTROL_FIELDS = (
    'state reason command_sent command_attempted command_interval_sec command_duration_ms '
    'movej_canfd_return_code feedback_age_sec ik_duration_ms fault_latched stop_command_sent '
    'motion_permitted ik_initialized motion_active').split()
EVENT_FIELDS = ('sequence start_timestamp end_timestamp duration_ms return_code sdk_acknowledged '
                'command_sequence joint_index target_position target_positions modbus_return_code '
                'modbus_duration_ms modbus_start_timestamp modbus_end_timestamp '
                'call_start_timestamp_ns call_end_timestamp_ns consecutive_failures '
                'unconfirmed_writes elapsed_time error mode generation_sequence generated_monotonic '
                'target_age_sec target_generation_count position_write_count '
                'target_generation_hz actual_modbus_write_hz configured_target_generation_hz '
                'overwritten_target_count skipped_generation_slots discarded_pending_target_count '
                'cancelled_selected_target_count').split()
FIELDS = ['timestamp_ns', 'rx_timestamp_ns', 'phase', 'record_type',
          'control_rx_interval_sec', *CONTROL_FIELDS, *EVENT_FIELDS, 'diagnostics_json', 'event_json']


def diagnostic_level(value):
    return int.from_bytes(value, 'little') if isinstance(value, bytes) else int(value)


def distribution(values):
    values = sorted(v for v in values if math.isfinite(v) and v >= 0)
    if not values:
        return dict(count=0, avg=None, p95=None, p99=None, max=None)
    def percentile(p):
        x = (len(values) - 1) * p
        i = int(x)
        return values[i] + (values[min(i + 1, len(values) - 1)] - values[i]) * (x - i)
    return dict(count=len(values), avg=sum(values) / len(values), p95=percentile(.95),
                p99=percentile(.99), max=values[-1])


class Capture:
    def __init__(self):
        self.rows = []
        self.events = []
        self.last_rx_ns = None
        self.start = None
        self.end = None

    def control(self, msg, stamp_ns, rx_ns):
        row = dict(timestamp_ns=stamp_ns, rx_timestamp_ns=rx_ns, record_type='control',
                   control_rx_interval_sec=(rx_ns - self.last_rx_ns) / 1e9
                   if self.last_rx_ns is not None else -1)
        self.last_rx_ns = rx_ns
        row.update({k: getattr(msg, k) for k in CONTROL_FIELDS})
        self.rows.append(row)

    def event(self, event, rx_ns):
        self.events.append(event)
        if event['event'] in ('MODBUS_START', 'STREAM_START', 'AUTO_STREAM_START'):
            self.start = event['timestamp_ns']
        elif event['event'] in ('MODBUS_FINISH', 'MODBUS_EXCEPTION', 'STREAM_FINISH', 'AUTO_STREAM_FINISH'):
            self.end = event['timestamp_ns']
        self.rows.append(dict(timestamp_ns=event['timestamp_ns'], rx_timestamp_ns=rx_ns,
                              record_type='event', event_json=json.dumps(event),
                              **{k: event[k] for k in EVENT_FIELDS if k in event}))

    def phase(self, stamp):
        if self.start is None:
            return 'UNTRIGGERED'
        if self.start - 10_000_000_000 <= stamp < self.start:
            return 'BASELINE'
        if stamp >= self.start and (self.end is None or stamp <= self.end):
            return 'INJECTION'
        if self.end is not None and self.end < stamp <= self.end + 10_000_000_000:
            return 'POST'
        return 'OUTSIDE'

    def save(self, output):
        base = Path(output).expanduser()
        base.mkdir(parents=True, exist_ok=True)
        for row in self.rows:
            row['phase'] = self.phase(row['timestamp_ns'])
        with (base / 'samples.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self.rows)
        with (base / 'events.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['timestamp_ns', 'event', *EVENT_FIELDS, 'payload_json'])
            writer.writeheader()
            for e in self.events:
                writer.writerow(dict(timestamp_ns=e['timestamp_ns'], event=e['event'],
                                     payload_json=json.dumps(e), **{k: e[k] for k in EVENT_FIELDS if k in e}))
        writes = [e for e in self.events if e['event'] in ('POSITION_WRITE_FINISH', 'POSITION_WRITE_FAILED')]
        summary = dict(events=self.events, complete_window=False, phases={}, stream=dict(
            write_results=len(writes),
            return_codes=dict(Counter(str(e.get('modbus_return_code')) for e in writes)),
            sdk_acknowledged_success_count=sum(e.get('modbus_return_code') == 0 for e in writes),
            sdk_unconfirmed_command_count=sum(e.get('modbus_return_code') is not None
                                             and e.get('modbus_return_code') != 0 for e in writes),
            sdk_exception_count=sum(e.get('error') is not None for e in writes),
            actual_motion='NOT AUTOMATICALLY VERIFIED',
            last_target_positions=writes[-1].get('target_positions') if writes else None,
            max_consecutive_failures=max((e.get('consecutive_failures', 0) for e in writes), default=0),
            modbus_duration_ms=distribution([e['modbus_duration_ms'] for e in writes]),
            completion=next((e for e in reversed(self.events) if e['event'] in ('STREAM_FINISH', 'AUTO_STREAM_FINISH')), None)))
        auto_start = next((e for e in self.events if e['event'] == 'AUTO_STREAM_START'), None)
        auto_stop = next((e for e in self.events if e['event'] == 'AUTO_STREAM_STOP'), None)
        auto_finish = next((e for e in self.events if e['event'] == 'AUTO_STREAM_FINISH'), None)
        if auto_start is not None or auto_finish is not None:
            from .o7_auto_stream_hil import observed_hz
            generated = [e['generated_monotonic'] for e in self.events if e['event'] == 'TARGET_GENERATED']
            called = [e['modbus_start_timestamp'] for e in writes if e.get('mode') == 'auto_stream']
            summary['auto_stream'] = dict(
                start=auto_start, stop=auto_stop, finish=auto_finish,
                target_events_received=len(generated), write_results_received=len(called),
                observed_target_generation_hz=observed_hz(sorted(generated)),
                observed_actual_modbus_write_hz=observed_hz(sorted(called)),
                node_reported_metrics={k: auto_finish.get(k) for k in (
                    'target_generation_count', 'position_write_count', 'configured_target_generation_hz',
                    'target_generation_hz', 'actual_modbus_write_hz', 'overwritten_target_count',
                    'skipped_generation_slots', 'discarded_pending_target_count',
                    'cancelled_selected_target_count')} if auto_finish else None)
        controls = [r for r in self.rows if r['record_type'] == 'control']
        if controls and self.start is not None and self.end is not None:
            summary['complete_window'] = (min(r['timestamp_ns'] for r in controls) <= self.start - 10e9
                                          and max(r['timestamp_ns'] for r in controls) >= self.end + 10e9)
        previous_fault = False
        for row in sorted(controls, key=lambda r: r['timestamp_ns']):
            row['_fault_entry'] = bool(row['fault_latched'] and not previous_fault)
            previous_fault = row['fault_latched']
        for phase in ('BASELINE', 'INJECTION', 'POST', 'UNTRIGGERED'):
            rows = [r for r in controls if r['phase'] == phase]
            sent = [r for r in rows if r['command_sent']]
            attempted = [r for r in rows if r['command_attempted']]
            summary['phases'][phase] = dict(
                samples=len(rows), sent=len(sent),
                send_failures=sum(not r['command_sent'] for r in attempted),
                not_sent=len(rows) - len(sent),
                not_sent_reasons=dict(Counter(r['reason'] for r in rows if not r['command_sent'])),
                fault_latched_samples=sum(r['fault_latched'] for r in rows),
                fault_latched_entries=sum(r['_fault_entry'] for r in rows),
                stop_samples=sum(r['stop_command_sent'] for r in rows),
                feedback_invalid_samples=sum(r['feedback_age_sec'] < 0 or r['feedback_age_sec'] > .05 for r in rows),
                feedback_invalid_reasons=dict(Counter(r['reason'] for r in rows if 'feedback' in r['reason'])),
                canfd_interval_sec=distribution([r['command_interval_sec'] for r in sent]),
                intervals_over_10ms=sum(r['command_interval_sec'] > .010 for r in sent),
                canfd_duration_ms=distribution([r['command_duration_ms'] for r in attempted]),
                control_rx_interval_sec=distribution([r['control_rx_interval_sec'] for r in rows]),
                feedback_age_sec=distribution([r['feedback_age_sec'] for r in rows]))
        with (base / 'summary.csv').open('w', newline='') as f:
            flat = []
            for phase, metrics in summary['phases'].items():
                row = {'phase': phase,
                       **{'stream_total_' + k: summary['stream'][k] for k in (
                           'sdk_acknowledged_success_count', 'sdk_unconfirmed_command_count',
                           'sdk_exception_count')},
                       'actual_motion': summary['stream']['actual_motion'],
                       'stream_last_target_positions': json.dumps(summary['stream']['last_target_positions'])}
                if 'auto_stream' in summary:
                    for k, v in (summary['auto_stream']['node_reported_metrics'] or {}).items():
                        row['auto_stream_total_' + k] = v
                for key, value in metrics.items():
                    if isinstance(value, dict) and 'avg' in value:
                        row.update({key + '_' + k: v for k, v in value.items()})
                    else:
                        row[key] = json.dumps(value) if isinstance(value, dict) else value
                flat.append(row)
            writer = csv.DictWriter(f, fieldnames=list(flat[0]))
            writer.writeheader()
            writer.writerows(flat)
        (base / 'summary.json').write_text(json.dumps(summary, indent=2))
        return summary
