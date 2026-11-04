#!/usr/bin/env python3
"""Read-only analysis of the existing ROS monitor capture; never imports an SDK."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rm_teleop'))
from rm_teleop.hil_monitor_data import distribution


def truth(value):
    return value is True or str(value).lower() == 'true'


def intervals(stamps):
    return [(b-a)/1e9 for a, b in zip(stamps, stamps[1:])]


def hz(stamps):
    return (len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else None


def window_metrics(controls, calls, begin, end):
    rows = [r for r in controls if begin <= int(r['timestamp_ns']) < end]
    sent = [r for r in rows if truth(r['command_sent'])]
    attempted = [r for r in rows if truth(r['command_attempted'])]
    selected = [e for e in calls if begin <= e['call_start_timestamp_ns'] < end]
    starts = sorted(e['call_start_monotonic_ns'] for e in selected)
    gaps = intervals(starts)
    return dict(
        control_samples=len(rows), canfd_sent=len(sent),
        canfd_attempted=len(attempted),
        canfd_failed=sum(not truth(r['command_sent']) for r in attempted),
        canfd_not_sent_reasons=dict(Counter(r['reason'] for r in rows if not truth(r['command_sent']))),
        canfd_interval_sec=distribution([float(r['command_interval_sec']) for r in sent]),
        canfd_intervals_over_10ms=sum(float(r['command_interval_sec'])>.010 for r in sent),
        canfd_duration_ms=distribution([float(r['command_duration_ms']) for r in attempted]),
        control_rx_interval_sec=distribution([float(r['control_rx_interval_sec']) for r in rows]),
        feedback_age_sec=distribution([float(r['feedback_age_sec']) for r in rows]),
        feedback_invalid_samples=sum(not 0<=float(r['feedback_age_sec'])<=.05 for r in rows),
        fault_latched_samples=sum(truth(r['fault_latched']) for r in rows),
        stop_samples=sum(truth(r['stop_command_sent']) for r in rows),
        o7_completed_calls=len(selected), o7_call_start_interval_sec=distribution(gaps),
        actual_modbus_write_hz=hz(starts),
        modbus_duration_ms=distribution([e['duration_ms'] for e in selected]),
        raw_return_codes=dict(Counter(str(e['return_code']) for e in selected)),
        sdk_exception_count=sum(e.get('error') is not None for e in selected),
        actual_motion='NOT AUTOMATICALLY VERIFIED')


def report(run):
    run=Path(run)
    with (run/'capture/samples.csv').open() as stream:
        controls=[r for r in csv.DictReader(stream) if r['record_type']=='control']
    with (run/'capture/events.csv').open() as stream:
        events=[json.loads(r['payload_json']) for r in csv.DictReader(stream)]
    markers=[json.loads(line) for line in (run/'markers.jsonl').read_text().splitlines()] if (run/'markers.jsonl').exists() else []
    calls=[e for e in events if e['event']=='O7_HAND_WRITE_FINISH']
    timed=[e for e in calls if all(k in e for k in ('call_start_timestamp_ns','call_start_monotonic_ns','duration_ms','return_code'))]
    starts=[e for e in events if e['event']=='O7_HAND_WRITE_START']
    cancelled=[e for e in events if e['event']=='O7_HAND_WRITE_CANCELLED']
    statuses=sorted([e for e in events if e['event']=='O7_HAND_STATUS'], key=lambda e:e['timestamp_ns'])
    timestamps=[int(r['timestamp_ns']) for r in controls]+[e['timestamp_ns'] for e in events]
    end=max(timestamps)+1 if timestamps else 0
    windows=[]
    for i, marker in enumerate(sorted(markers, key=lambda m:m['timestamp_ns'])):
        ordered=sorted(markers, key=lambda m:m['timestamp_ns'])
        stop=ordered[i+1]['timestamp_ns'] if i+1<len(ordered) else end
        windows.append(dict(label=marker['label'], begin_ns=marker['timestamp_ns'], end_ns=stop,
                            note=marker.get('note',''), **window_metrics(controls,timed,marker['timestamp_ns'],stop)))
    blocked=[]
    for i, status in enumerate(statuses):
        reason=status.get('reason','')
        if status.get('enabled') and not any(word in reason for word in ('tracking_invalid','tracking_timeout','stale_')):
            continue
        stop=statuses[i+1]['timestamp_ns'] if i+1<len(statuses) else end
        after=[e for e in timed if status['timestamp_ns'] <= e['call_start_timestamp_ns'] < stop]
        blocked.append(dict(observed_from_ns=status['timestamp_ns'], observed_until_ns=stop,
                            enabled=status.get('enabled'), reason=reason,
                            new_native_call_starts=len(after),
                            call_sequences=[e['command_sequence'] for e in after]))
    summary=dict(
        source=str(run), control_samples=len(controls), hand_start_events=len(starts),
        hand_finish_events=len(calls), hand_cancel_events=len(cancelled),
        missing_exact_call_timing=len(calls)-len(timed),
        unmatched_start_count=len(starts)-len(calls)-len(cancelled),
        overall=window_metrics(controls,timed,0,end), windows=windows,
        blocked_status_observations=blocked,
        last_target_positions=calls[-1].get('target_positions') if calls else None,
        physical_motion='REQUIRES_OPERATOR_OBSERVATION',
        notes=[
            'Configured write_hz comes from the saved device_config.json, not observed rates.',
            'Observed Hz is (N-1)/(last-first) over completed calls; includes tracking/idle gaps in its window.',
            'START events precede final authorization; exact native timing is available in FINISH. Unfinished calls remain unverified.',
            'Blocked windows use observed status timestamps, not the exact moment tracking was lost. Review the raw topic/bag too.',
            'An in-flight call may FINISH after disable. Review new native call START times separately.',
            'Zero faults/failures without ACTIVE samples is not an ARM concurrency pass.',
            'The original monitor stream summary counts legacy HIL, not O7_HAND_WRITE events.'])
    config=run/'device_config.json'
    if config.exists():
        summary['configured_write_hz']=json.loads(config.read_text())['write_hz']
    (run/'pico_o7_review.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action', required=True)
    mark=sub.add_parser('mark'); mark.add_argument('run',type=Path); mark.add_argument('label'); mark.add_argument('--note',default='')
    review=sub.add_parser('report'); review.add_argument('run',type=Path)
    args=parser.parse_args()
    if args.action=='mark':
        # A local observation marker only: no ROS request and no device action.
        entry=dict(timestamp_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),label=args.label,note=args.note)
        args.run.mkdir(parents=True,exist_ok=True)
        with (args.run/'markers.jsonl').open('a') as stream: stream.write(json.dumps(entry,ensure_ascii=False)+'\n')
        print(json.dumps(entry,ensure_ascii=False))
    else:
        result=report(args.run)
        print(json.dumps({k:result[k] for k in ('hand_start_events','hand_finish_events','hand_cancel_events','unmatched_start_count','overall')},indent=2))
        print('Saved:',args.run/'pico_o7_review.json')


if __name__=='__main__': main()
