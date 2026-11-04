"""Thin raw-UDP adapter; reuse the validated Stage 1.5 position decoder."""
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parent
from .pico_hand_skeleton_viewer import _jsonl_lines, _parse_positions, _finite_number, bend_angle

@dataclass
class Packet:
    index: int
    line: int
    capture_time_utc: str = ''
    capture_unix: float | None = None
    source_timestamp: float | None = None
    time_s: float = float('nan')
    seq: int | None = None
    tracked: bool = False
    keypoints_valid: bool = False
    valid: bool = False
    reason: str = ''
    xyz: np.ndarray = field(default_factory=lambda: np.full((21,3),np.nan))
    payload: dict = field(default_factory=dict)

def load_packets(path):
    packets=[]; outer_keys=set(); payload_keys=set(); bad=[]; raw_hash=hashlib.sha256()
    t0=None
    for lineno,line in enumerate(_jsonl_lines(Path(path)),1):
        raw_hash.update(line.encode('utf-8'))
        if not line.strip():
            continue
        f=Packet(len(packets),lineno)
        try:
            row=json.loads(line)
            if not isinstance(row,dict):
                raise ValueError('outer record is not an object')
            outer_keys.update(row)
            f.payload=json.loads(row['raw_text']) if isinstance(row.get('raw_text'),str) else row
            if not isinstance(f.payload,dict):
                raise ValueError('UDP JSON is not an object')
            p=f.payload; payload_keys.update(p)
            f.capture_time_utc=str(row.get('capture_time_utc',''))
            if f.capture_time_utc:
                ts=datetime.fromisoformat(f.capture_time_utc.replace('Z','+00:00'))
                if ts.tzinfo is None:
                    raise ValueError('capture timestamp must include a timezone')
                f.capture_unix=ts.timestamp()
            elif _finite_number(row.get('capture_time_unix')):
                f.capture_unix=float(row['capture_time_unix'])
            f.source_timestamp=float(p['timestamp']) if _finite_number(p.get('timestamp')) else None
            t=f.capture_unix if f.capture_unix is not None else f.source_timestamp
            if t is None:
                raise ValueError('missing capture and source timestamp; no invented frame rate')
            if t0 is None:
                t0=t
            f.time_s=t-t0
            f.seq=p.get('seq') if isinstance(p.get('seq'),int) and not isinstance(p.get('seq'),bool) else None
            f.tracked=p.get('right_hand_tracked') is True
            f.xyz,f.keypoints_valid=_parse_positions(p)
            f.valid=f.tracked and f.keypoints_valid
            f.reason='' if f.valid else ('untracked' if not f.tracked else 'invalid_keypoints')
        except (ValueError,KeyError,TypeError) as e:
            f.valid=False;f.reason='parse_error: '+str(e);bad.append({'line':lineno,'error':str(e)})
        packets.append(f)
    if not packets:
        raise ValueError('No nonempty JSONL records')
    return packets,dict(raw_jsonl_sha256=raw_hash.hexdigest(),outer_keys=sorted(outer_keys),
                        udp_keys=sorted(payload_keys),parse_errors=bad)

def checked_landmarks(f, parity=-1, scale=1.):
    """No quaternion use. Parity is an empirical position-coordinate calibration."""
    x=f.xyz.copy()*scale
    x[:,2]*=parity
    # Reject degenerate palm explicitly: do not invoke somehand's global-axis fallback.
    a=x[5]-x[0]; b=x[9]-x[0]
    denom=np.linalg.norm(a)*np.linalg.norm(b)
    if denom < 1e-12 or np.linalg.norm(np.cross(a,b))/denom < 0.025:
        raise ValueError('degenerate palm frame')
    for start in (1,5,9,13,17):
        if np.min(np.linalg.norm(np.diff(x[start:start+4],axis=0),axis=1)) < 1e-6:
            raise ValueError('degenerate finger segment')
    return x

def runs(mask):
    result=[]; start=None
    for i,v in enumerate([*map(bool,mask),False]):
        if v and start is None:
            start=i
        if not v and start is not None:
            result.append([start,i-1]);start=None
    return result
