"""Named, provisional model-angle -> device mapping; no SDK or ROS imports."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from pico_o7.paths import share_dir
ROOT=share_dir()
CHANNELS=('thumb_pitch','thumb_yaw','index_pitch','middle_pitch','ring_pitch','little_pitch','thumb_roll')
JOINTS=('thumb_cmc_pitch','thumb_cmc_yaw','index_mcp_pitch','middle_mcp_pitch','ring_mcp_pitch','pinky_mcp_pitch','thumb_cmc_roll')

class Mapper:
    def __init__(self,path=None):
        self.path=Path(path or ROOT/'config/o7_device_linear.json').resolve()
        self.config=json.loads(self.path.read_text())
        self.axes=self.config['axes']
        mapping=json.loads((ROOT/'models/joint_mapping.json').read_text())
        self.bounds=mapping['effective_independent_bounds_rad']
        mjcf=ET.parse(ROOT/'models/o7_right_kinematic.xml')
        actual={j.attrib['name']:[float(x)for x in j.attrib['range'].split()]
                for j in mjcf.findall('.//joint')if 'name'in j.attrib and 'range'in j.attrib}
        if len(self.axes)!=7:raise ValueError('exactly seven axes required')
        for i,a in enumerate(self.axes):
            if a['index']!=i or a['channel']!=CHANNELS[i] or a['joint']!=JOINTS[i]:raise ValueError('device name/order mismatch')
            if a['direction']not in ('decreasing','increasing'):raise ValueError('unknown direction')
            for k in ('device_min','device_max','max_step'):
                if type(a[k])is not int:raise ValueError('integer device configuration required')
            if not 0<=a['device_min']<a['device_max']<=255 or not 1<=a['max_step']<=255:raise ValueError('invalid output/step limits')
            if any(abs(x-y)>1e-10 for x,y in zip(self.bounds[a['joint']],actual[a['joint']])):raise ValueError('effective limits differ from actual MJCF')
        self.check_target(self.config['initial_hold'])
        for a,v in zip(self.axes,self.config['initial_hold']):
            if not a['device_min']<=v<=a['device_max']:raise ValueError('initial hold outside configured range')
        for k in ('write_hz','max_age_sec','max_call_ms'):
            if type(self.config[k])not in (int,float)or not math.isfinite(self.config[k])or self.config[k]<=0:raise ValueError('invalid '+k)
        if self.config['write_hz']>10:raise ValueError('write_hz must be <=10')

    @staticmethod
    def check_target(target):
        if len(target)!=7 or any(type(x)is not int or not 0<=x<=255 for x in target):raise ValueError('target must contain seven integers in 0..255')

    def convert(self,q_by_name,previous=None):
        # Always select by MODEL NAME, never by dictionary/CSV/solver column order.
        desired=[];normalized=[]
        if previous is not None:self.check_target(previous)
        for i,a in enumerate(self.axes):
            q=q_by_name[a['joint']]
            if type(q)not in (int,float)or not math.isfinite(q):raise ValueError('missing/nonfinite joint: '+a['joint'])
            lo,hi=self.bounds[a['joint']]
            u=max(0.,min(1.,(q-lo)/(hi-lo)))
            normalized.append(u)
            if a['direction']=='decreasing':u=1-u
            v=int(math.floor(a['device_min']+u*(a['device_max']-a['device_min'])+.5))
            if previous is not None:v=max(previous[i]-a['max_step'],min(previous[i]+a['max_step'],v))
            desired.append(max(a['device_min'],min(a['device_max'],v)))
        self.check_target(desired)
        return desired,normalized
