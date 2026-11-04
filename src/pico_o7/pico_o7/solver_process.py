#!/usr/bin/env python3
"""Dedicated Python 3.11 solver. JSON lines only; never imports ROS or the SDK."""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from .paths import share_dir
ROOT=share_dir()
sys.path.insert(0,str(ROOT/'vendor/somehand/src'))
import numpy as np
from somehand.application.engine import RetargetingEngine
from somehand.domain import HandFrame
from pico_o7.capture_io import checked_landmarks
from pico_o7.pico_hand_skeleton_viewer import _parse_positions
from pico_o7.o7_stage.hand_input import input_valid
from pico_o7.o7_stage.mapping import Mapper

class Solver:
    def __init__(self):
        self.engine=None;self.last_time=None;self.last_seq=None
        self.mapping=json.loads((ROOT/'models/joint_mapping.json').read_text())
    def solve(self,p,reset=False,received=None):
        started=time.perf_counter()
        try:
            if not input_valid(p):raise ValueError('invalid tracking/keypoints')
            seq=p.get('seq');timestamp=p.get('timestamp')
            if type(seq)is not int or type(timestamp)not in (int,float)or not np.isfinite(timestamp):raise ValueError('invalid seq/time')
            if reset:self.engine=None;self.last_time=None;self.last_seq=None
            if self.last_time is not None:
                if timestamp<=self.last_time or seq<=self.last_seq:raise ValueError('nonincreasing seq/time')
                if timestamp-self.last_time>.25:self.engine=None
            xyz,valid=_parse_positions(p)
            if not valid:raise ValueError('position parser rejected frame')
            x=checked_landmarks(SimpleNamespace(xyz=xyz),-1,1.)
            history_reset=self.engine is None
            if self.engine is None:self.engine=RetargetingEngine.from_config_path(str(ROOT/'config/o7_right_vectors.yaml'))
            result=self.engine.process(HandFrame(x,None,'right'))
            opt=self.engine.retargeter.last_optimize_result
            if not opt.success:raise ValueError('optimizer_failed: '+str(opt.message))
            q={n:float(v)for n,v in zip(self.engine.hand_model.get_joint_names(),result.qpos)}
            for n,v in q.items():
                lo,hi=self.mapping['vendor_limits_rad'][n]
                if not np.isfinite(v)or not lo-1e-9<=v<=hi+1e-9:raise ValueError('nonfinite/model limits: '+n)
            # Also validate all seven named roots against effective, mimic-tightened bounds.
            for n,(lo,hi)in self.mapping['effective_independent_bounds_rad'].items():
                if not lo-1e-9<=q[n]<=hi+1e-9:raise ValueError('effective limit violation: '+n)
            self.last_seq=seq;self.last_time=timestamp
            return dict(valid=True,seq=seq,timestamp=timestamp,q_by_name=q,history_reset=history_reset,solve_ms=(time.perf_counter()-started)*1000)
        except Exception as exc:
            self.engine=None;self.last_seq=None;self.last_time=None
            return dict(valid=False,seq=p.get('seq'),error=repr(exc),solve_ms=(time.perf_counter()-started)*1000)

def main():
    solver=Solver()
    for line in sys.stdin:
        try:
            job=json.loads(line);result=solver.solve(job['payload'],reset=job.get('reset',False))
        except Exception as exc:result=dict(valid=False,error=repr(exc))
        print(json.dumps(result,allow_nan=False),flush=True)

if __name__=='__main__':main()
