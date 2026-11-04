"""Receiver-side validation uses stdlib only (ROS Python 3.10)."""
import math
import json

def input_valid(p):
    if not isinstance(p,dict)or p.get('right_hand_tracked')is not True:return False
    points=p.get('right_keypoints')
    if not isinstance(points,list)or len(points)!=21:return False
    for k in points:
        if not isinstance(k,dict)or k.get('valid')is not True:return False
        pos=k.get('position')
        if not isinstance(pos,dict):return False
        if any(type(pos.get(a))not in (int,float)or not math.isfinite(pos[a])for a in ('x','y','z')):return False
    return True

class ForwardCore:
    """Tap the ORIGINAL accepted UDP path; never create/bind another socket."""
    def __init__(self,original,emit):self.original=original;self.emit=emit
    def __getattr__(self,name):return getattr(self.original,name)
    def process_datagram(self,data,now):
        result=self.original.process_datagram(data,now)
        if result.accepted:
            p=json.loads(data)
            valid=input_valid(p)
            hand={k:p.get(k)for k in ('seq','timestamp','right_hand_tracked')}
            if valid:hand['right_keypoints']=[dict(valid=True,position=k['position'])for k in p['right_keypoints']]
            self.emit(dict(received_monotonic=now,payload=hand if valid else {},valid=valid,
                           sequence_status=result.sequence_status.value))
        elif 'duplicate'not in result.reason and 'out_of_order'not in result.reason:
            self.emit(dict(received_monotonic=now,payload={},valid=False,reason=result.reason))
        return result
