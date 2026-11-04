"""Construct the actual overlay with ROS transports and connection worker stubbed."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest


@pytest.mark.parametrize('arm,hand',[(False,False),(False,True),(True,False),(True,True)])
def test_modes_use_parent_lifecycle_and_parameter_start_without_real_connection(tmp_path,arm,hand):
    from rm_teleop import rm65_canfd_high_follow_node as original
    from pico_o7 import shared_session_node as stage
    base=original.Rm65CanfdHighFollowNode
    overrides=dict(allow_motion=arm,motion_confirmation='RM65_CANFD_HIGH_FOLLOW_CONFIRMED' if arm else '',
                   enable_o7_retargeting=hand,canfd_csv_log_directory=str(tmp_path/'canfd'))
    services=[];calls=[];disconnected=[];events=[]
    def init(self,*args,**kwargs):self.offline_parameters={}
    def declare(self,name,value,*args,**kwargs):self.offline_parameters[name]=overrides.get(name,value)
    def service(self,kind,name,callback):services.append(name);return None
    logger=SimpleNamespace(info=lambda *args:None,warning=lambda *args:None,error=lambda *args:None)
    methods=dict(__init__=init,declare_parameter=declare,
                 get_parameter=lambda self,name:SimpleNamespace(value=self.offline_parameters[name]),
                 create_publisher=lambda *args,**kwargs:SimpleNamespace(publish=lambda *args:None),
                 create_subscription=lambda *args,**kwargs:None,create_service=service,
                 create_timer=lambda *args,**kwargs:None,get_logger=lambda *args:logger,
                 destroy_node=lambda *args:True)
    def check(args=None):
        klass=original.Rm65CanfdHighFollowNode
        # Guard logic is covered separately. This isolates session/worker/close ordering.
        klass._hand_rejection=lambda self:'' if self._connected else 'existing API2 session not connected'
        node=klass()
        try:
            assert node.motion_permitted==arm
            assert (node.hand is not None)==hand
            assert '/rm65/o7_hand/enable' not in services
            assert '/rm65/o7_hand/disable' not in services
            def write(values):calls.append(list(values));return 1
            def disconnect():
                assert not node._hil.worker or not node._hil.worker.is_alive()
                disconnected.append(True)
            adapter=SimpleNamespace(arm=object(),handle=object(),write_o7_hil_positions=write,disconnect=disconnect)
            with node._adapter_lock:node._adapter=adapter;node._connected=True
            if hand:
                deadline=time.monotonic()+1
                while not node.hand.state.enabled and time.monotonic()<deadline:time.sleep(.005)
                assert node.hand.state.enabled
                # Submit one solved mock target, through the actual inherited owner slot.
                state=node.hand.state;now=time.monotonic()
                with state.lock:
                    state.last_received=now
                    q={n:(lo+hi)/2 for n,(lo,hi) in state.mapper.bounds.items()}
                    state.pending=(state.epoch,now,dict(valid=True,seq=1,q_by_name=q))
                deadline=time.monotonic()+1
                while not calls and time.monotonic()<deadline:time.sleep(.005)
                assert len(calls)==1
            else:assert node._hil.worker is None and calls==[]
            assert not any(n.startswith('Robotic_Arm') for n in sys.modules)
        finally:node.destroy_node()
        assert disconnected==[True]
    with patch.object(original,'Rm65CanfdHighFollowNode',base),patch.object(base,'_connection_worker',lambda self:None),patch.object(original,'main',check),patch.multiple(original.Node,**methods):
        stage.main()
