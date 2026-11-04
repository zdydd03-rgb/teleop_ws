"""Validate full-vector index2 plans without creating device commands from PICO/radians."""
import hashlib
import json
from datetime import datetime,timezone
from pathlib import Path
from .protocol import JOINT_NAMES


def vector(value, label):
    if not isinstance(value,(list,tuple)) or len(value)!=7 or any(type(x)is not int or not 0<=x<=255 for x in value):
        raise ValueError(label+' must contain seven integers in0..255 (no booleans)')
    return tuple(value)


class Index2Plan:
    def __init__(self, data):
        if data.get('schema_version')!=1 or data.get('joint_index')!=2 or type(data.get('joint_index'))is not int:
            raise ValueError('Require schema_version1 and joint_index2')
        self.baseline=vector(data.get('hold_positions'),'hold_positions')
        self.target=vector(data.get('target_positions'),'target_positions')
        for i in (0,1,3,4,5,6):
            if self.target[i]!=self.baseline[i]:
                raise ValueError(f'non-selected channel{i} changed: hold={self.baseline[i]}, target={self.target[i]}')
        if self.target[2]==self.baseline[2]:raise ValueError('index2 target must differ from baseline')
        bounds=data.get('approved_index_range')
        if not isinstance(bounds,list)or len(bounds)!=2 or any(type(x)is not int for x in bounds):
            raise ValueError('approved_index_range must contain two integer bounds')
        low,high=bounds
        if not 220<=low<high<=255:raise ValueError('This stage is restricted to an explicitly approved local window within220..255')
        if not all(low<=x<=high for x in (self.baseline[2],self.target[2])):
            raise ValueError('index2 baseline/target outside approved local window')
        flags=data.get('hold_confirmed_current_session')
        if not isinstance(flags,list)or len(flags)!=7 or any(type(x)is not bool for x in flags):
            raise ValueError('hold_confirmed_current_session must contain seven explicit booleans')
        self.data=json.loads(json.dumps(data))
        self.data['hold_positions']=list(self.baseline)
        self.data['target_positions']=list(self.target)
        self.hash=hashlib.sha256(json.dumps(self.data,sort_keys=True,separators=(',',':')).encode()).hexdigest()

    @classmethod
    def load(cls,path):return cls(json.loads(Path(path).read_text()))

    def missing_confirmations(self):
        missing=[]
        for i,flag in enumerate(self.data['hold_confirmed_current_session']):
            if not flag:missing.append(f'channel{i} current-session hold not confirmed')
        evidence=self.data.get('hold_evidence',[])
        for i in range(7):
            if not isinstance(evidence,list)or len(evidence)!=7 or not isinstance(evidence[i],str)or not evidence[i].strip():
                missing.append(f'channel{i} hold evidence missing')
        for key in ('operator','confirmation_utc'):
            if not isinstance(self.data.get(key),str)or not self.data[key].strip():missing.append(key+' missing')
        if self.data.get('local_window_confirmed_current_session')is not True:
            missing.append('local window not confirmed for current session')
        return missing

    def validate(self):
        missing=self.missing_confirmations()
        if missing:raise ValueError('; '.join(missing))
        try:
            confirmed=datetime.fromisoformat(self.data['confirmation_utc'].replace('Z','+00:00'))
            if confirmed.tzinfo is None:raise ValueError('timezone required')
            age=(datetime.now(timezone.utc)-confirmed).total_seconds()
            if not -60<=age<=1800:raise ValueError('confirmation older than30minutes or in the future')
        except ValueError as exc:
            raise ValueError('Invalid/stale confirmation_utc: '+str(exc)) from exc

    def dry_run(self):
        try:
            self.validate();ready=True;validation_error=None
        except ValueError as exc:
            ready=False;validation_error=str(exc)
        return dict(mode='DRY_RUN',sdk_calls=0,device_writes=0,joint_index=2,
                    joint_order=list(JOINT_NAMES),hold_positions=list(self.baseline),
                    target_positions=list(self.target),
                    non_selected_checks=[dict(index=i,name=JOINT_NAMES[i],hold=self.baseline[i],
                        target=self.target[i],unchanged=True,
                        field_confirmed=self.data['hold_confirmed_current_session'][i])for i in (0,1,3,4,5,6)],
                    structurally_valid=True,field_confirmations_complete=not self.missing_confirmations(),
                    plan_confirmations_valid_now=ready,live_plan_validation_error=validation_error,
                    missing_confirmations=self.missing_confirmations(),maximum_sdk_write_calls=1,
                    automatic_return_write=False,plan_sha256=self.hash,
                    note='Structural/dry-run validity does not confirm physical hold positions or enable hardware.')
