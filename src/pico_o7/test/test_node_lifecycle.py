"""Construct the actual overlay with ROS transports and connection worker stubbed."""
from pathlib import Path
import sys
import time
import threading
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
        node=klass()
        try:
            assert node.motion_permitted==arm
            assert (node.hand is not None)==hand
            assert '/rm65/o7_hand/enable' in services
            assert '/rm65/o7_hand/disable' in services
            def write(values):calls.append(list(values));return 1
            def disconnect():
                assert not node._hil.worker or not node._hil.worker.is_alive()
                disconnected.append(True)
            adapter=SimpleNamespace(arm=object(),handle=object(),initialize_o7_modbus=lambda:dict(success=True,return_code=0,config=dict(return_code=0,mode=1,baudrate=115200,timeout=1)),write_o7_hil_positions=write,disconnect=disconnect)
            with node._adapter_lock:node._adapter=adapter;node._connected=True
            if hand:
                deadline=time.monotonic()+1
                while not node.hand.state.enabled and time.monotonic()<deadline:time.sleep(.005)
                assert node.hand.state.enabled
                previous=node._last_summary
                node._last_summary=SimpleNamespace(state='ACTIVE')
                assert node._initialize_hand(adapter,'test_ACTIVE_independence')['success']
                assert node._hand_rejection()==''
                node._fault_latched=True
                assert node._hand_rejection()==''
                assert node._initialize_hand(adapter,'test_ARM_fault_independence')['success']
                node._fault_latched=False
                assert 'on_timer' not in klass.__dict__
                # Block a mock native initialization while an inherited ARM tick runs.
                entered=threading.Event();release=threading.Event();timer_calls=[];results=[]
                normal_init=adapter.initialize_o7_modbus
                def blocked_init():
                    entered.set();assert release.wait(2);return normal_init()
                adapter.initialize_o7_modbus=blocked_init
                init_thread=threading.Thread(target=lambda:results.append(node._initialize_hand(adapter,'blocked_mock')))
                init_thread.start()
                try:
                    assert entered.wait(1)
                    with patch.object(base,'on_timer',lambda self:timer_calls.append('tick')):
                        node.on_timer()
                    assert timer_calls==['tick'] and init_thread.is_alive()
                finally:release.set();init_thread.join(2);adapter.initialize_o7_modbus=normal_init
                assert results[0]['success']

                node._last_summary=previous
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
