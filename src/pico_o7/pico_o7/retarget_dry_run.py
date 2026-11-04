#!/usr/bin/env python3
"""Real capture -> existing solver -> named linear mapping; zero SDK/ROS calls."""
import argparse
import csv
import json
from pathlib import Path
import time
import numpy as np
from pico_o7.capture_io import load_packets,bend_angle
from pico_o7.solver_process import Solver
from pico_o7.o7_stage.mapping import Mapper,CHANNELS,JOINTS
from pico_o7.o7_stage.realtime import LatestHand
from .paths import share_dir, workspace_root
ROOT=share_dir()

def run(path,output,config):
    mapper=Mapper(config);frames,audit=load_packets(path);solver=Solver();clock=[100.]
    state=LatestHand(mapper,lambda:clock[0]);state.enable()
    rows=[];samples=[];started=time.perf_counter();histories=0;solved=0;failures=[];invalid=0
    for f in frames:
        clock[0]=100+f.time_s if np.isfinite(f.time_s)else clock[0]
        state.receive(dict(received_monotonic=clock[0],valid=f.valid,payload=f.payload))
        raw=state.take_raw();result=dict(valid=False,error=f.reason or'invalid_input');desired=None;scheduled=None
        if raw:
            epoch,received,p=raw
            result=solver.solve(p,reset=getattr(solver,'_epoch',None)!=epoch);solver._epoch=epoch
            state.offer(epoch,received,result)
            if result['valid']:
                solved+=1;histories+=int(result['history_reset'])
                desired,norm=mapper.convert(result['q_by_name'])
                claimed=state.claim()
                if claimed:
                    scheduled=claimed['target_positions']
                    # Scheduling simulation only. No SDK, no invented SDK response.
                    with state.lock:state.inflight=False
                bends=[bend_angle(*f.xyz[s:s+3])for s in (5,9,13,17)]
                samples.append(dict(frame_index=f.index,seq=f.seq,time_s=f.time_s,
                    mean_four_finger_bend_deg=float(np.mean(bends)),
                    thumb_index_distance=float(np.linalg.norm(f.xyz[4]-f.xyz[8])),
                    model_angles_rad={n:result['q_by_name'][n]for n in JOINTS},
                    device_channels=list(CHANNELS),linear_target=desired,scheduled_target=scheduled))
            else:failures.append(dict(frame_index=f.index,seq=f.seq,error=result.get('error')))
        else:invalid+=1;solver.engine=None
        row=dict(frame_index=f.index,seq=f.seq,time_s=f.time_s,source_timestamp=f.source_timestamp,
                 capture_time_utc=f.capture_time_utc,input_valid=f.valid,solve_valid=result['valid'],
                 history_reset=result.get('history_reset',False),status='solved'if result['valid']else result.get('error'),
                 scheduled=scheduled is not None,solve_ms=result.get('solve_ms',''))
        for i,(c,n)in enumerate(zip(CHANNELS,JOINTS)):
            row[n+'_rad']=result.get('q_by_name',{}).get(n,'')
            row[c+'_desired_0_255']=desired[i]if desired is not None else''
            row[c+'_scheduled_0_255']=scheduled[i]if scheduled is not None else''
        rows.append(row)
    output.mkdir(parents=True,exist_ok=True)
    with (output/'device_targets.csv').open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    representative={}
    if samples:
        representative={'most_open_geometry_candidate':min(samples,key=lambda s:s['mean_four_finger_bend_deg']),
                        'most_flexed_geometry_candidate':max(samples,key=lambda s:s['mean_four_finger_bend_deg']),
                        'closest_pinch_geometry_candidate':min(samples,key=lambda s:s['thumb_index_distance'])}
    summary=dict(dry_run=True,hardware_sdk_calls=0,input=str(path.resolve()),raw_jsonl_sha256=audit['raw_jsonl_sha256'],
                 frames=len(frames),input_valid=sum(f.valid for f in frames),solved=solved,invalid_input=invalid,
                 solver_failures=failures,history_resets=histories,dry_scheduled_targets=state.counts['writes'],
                 device_order=list(CHANNELS),model_order_in_device_order=list(JOINTS),
                 output_range=[0,255],mapping_type='provisional model-limit linear mapping, NOT measured calibration',
                 note='Representative frames selected by geometry, not manual gesture identification. scheduled targets simulate configured admission/step limit; no SDK return code is synthesized.',
                 solve_elapsed_sec=time.perf_counter()-started,
                 solve_ms_median=float(np.median([r['solve_ms']for r in rows if r['solve_ms']!=''])),
                 representative_frames=representative,solver_config='config/o7_right_vectors.yaml',model='models/o7_right_kinematic.xml',
                 mapping_config=str(mapper.path),direction_pending=['thumb_yaw','thumb_roll'])
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in summary.items()if k!='representative_frames'},ensure_ascii=False))
    for label,s in representative.items():print(json.dumps(dict(candidate=label,**s),ensure_ascii=False))
    return summary

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,action='append',required=True,help='JSONL or ZIP; repeat for several captures')
    p.add_argument('--output',type=Path,default=workspace_root()/'concurrency_logs/pico_o7_offline')
    p.add_argument('--mapping',type=Path,default=ROOT/'config/o7_device_linear.json')
    a=p.parse_args()
    summaries=[run(f,a.output/f.stem,a.mapping)for f in a.input]
    (a.output/'summary.json').write_text(json.dumps(dict(hardware_sdk_calls=0,runs=summaries),indent=2,allow_nan=False)+'\n')
    return 1 if any(s['solver_failures']for s in summaries)else 0
if __name__=='__main__':raise SystemExit(main())
