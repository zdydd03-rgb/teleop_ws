#!/usr/bin/python3
"""Read existing logs only. No SDK/ROS imports or hardware calls."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics
from .paths import workspace_root
ROOT=workspace_root()


def events(path):
    with path.open(errors='replace')as stream:
        for number,line in enumerate(stream,1):
            line=re.sub(r'\x1b\[[0-9;]*m','',line)
            if 'O7 HIL 'in line:line=line.split('O7 HIL ',1)[1]
            elif not line.lstrip().startswith('{'):continue
            try:e=json.loads(line)
            except ValueError:continue
            if isinstance(e,dict)and'event'in e:yield number,e


def extract(path):
    starts={};rows=[]
    for number,e in events(path):
        key=(e.get('request_id'),e.get('mode'),e.get('command_sequence'))
        if e['event']in ('O7_HAND_WRITE_START','POSITION_WRITE_START'):
            starts[key]=e;continue
        if e['event']not in ('O7_HAND_WRITE_FINISH','POSITION_WRITE_FINISH'):continue
        begin=starts.get(key,{})
        rows.append(dict(source=str(path.resolve()),line=number,mode=e.get('comparison_environment',e.get('mode')),
            request_id=e.get('request_id',''),command_sequence=e.get('command_sequence'),
            target_positions=json.dumps(e.get('target_positions')),hold_positions=json.dumps(e.get('hold_positions')),seq=e.get('seq',''),
            start_event_timestamp_ns=begin.get('timestamp_ns',''),end_event_timestamp_ns=e.get('timestamp_ns',''),
            call_start_timestamp_ns=e.get('call_start_timestamp_ns',''),call_end_timestamp_ns=e.get('call_end_timestamp_ns',''),
            call_start_utc=e.get('call_start_utc',''),call_end_utc=e.get('call_end_utc',''),
            call_start_monotonic_ns=e.get('call_start_monotonic_ns',''),call_end_monotonic_ns=e.get('call_end_monotonic_ns',''),
            duration_ms=e.get('duration_ms'),raw_return_code=e.get('return_code'),error=e.get('error'),
            timing_scope=e.get('timing_scope','legacy adapter duration; exact call timestamps unavailable for hand events'),
            operator='',actual_motion='',moving_axis='',direction='',other_six_held='',video_time='',notes=''))
    return rows


def stats(rows):
    values=[float(r['duration_ms'])for r in rows if r.get('duration_ms')is not None]
    if not values:return dict(calls=0)
    values.sort()
    def pct(p):
        x=(len(values)-1)*p;lo=int(x)
        return values[lo]+(values[min(lo+1,len(values)-1)]-values[lo])*(x-lo)
    return dict(calls=len(values),mean_ms=statistics.mean(values),median_ms=statistics.median(values),
                p95_ms=pct(.95),min_ms=min(values),max_ms=max(values),
                raw_return_code_counts=dict(collections.Counter(str(r['raw_return_code'])for r in rows)),
                physical_motion='requires field observation; never inferred from code')


def write_rows(path,rows):
    if not rows:return
    with path.open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def compare(hand,standalone,output):
    output.mkdir(parents=True,exist_ok=True)
    a=extract(hand);b=extract(standalone)
    write_rows(output/'hand_calls_observations.csv',a);write_rows(output/'standalone_calls_observations.csv',b)
    result=dict(hand_log=str(hand),standalone_log=str(standalone),hand=stats(a),standalone=stats(b),
        same_target_vectors=bool(a and b)and set(r['target_positions']for r in a)==set(r['target_positions']for r in b),
        same_starting_hold_commands=bool(a and b)and set(r['hold_positions']for r in a)==set(r['hold_positions']for r in b),
        observations='Fill actual_motion, axis, direction, other_six_held, and video_time in generated CSVs. Empty is NOT passed.',
        note='No success/failure is inferred from raw SDK codes. A fixed target in hand-start compares its environment/worker, not the PICO mapping path. Record the starting hold/pose separately.')
    if a and b:result['difference_of_means_ms']=stats(a)['mean_ms']-stats(b)['mean_ms']
    (output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hand-log',type=Path);p.add_argument('--standalone-events',type=Path)
    p.add_argument('--output',type=Path,default=ROOT/'concurrency_logs/o7_write_latency/comparison')
    a=p.parse_args()
    if a.hand_log or a.standalone_events:
        if not(a.hand_log and a.standalone_events):p.error('both input logs required')
        compare(a.hand_log,a.standalone_events,a.output)
    else:p.error('Specify --hand-log and --standalone-events; archived source audits are reference documents only')
if __name__=='__main__':main()
