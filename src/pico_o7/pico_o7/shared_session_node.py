#!/usr/bin/python3
"""Field-only replacement for the existing high-follow node; never run by dry-run.

The original connection worker/Adapter owns the sole API2 session. New operations
use its existing SingleSessionHil owner/worker slot; no second SDK client exists.
"""
from pathlib import Path
import hashlib
import json
import sys
import time
import uuid

from .paths import share_dir, workspace_root, solver_python
ROOT=share_dir()


def main(args=None):
    from rm_teleop import rm65_canfd_high_follow_node as original
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from pico_o7.o7_stage.worker import StageHil
    from pico_o7.o7_stage.plan import Index2Plan

    class SharedO7Node(original.Rm65CanfdHighFollowNode):
        def _declare_parameters(self):
            super()._declare_parameters()
            self.declare_parameter('enable_o7_read_diag',False)
            self.declare_parameter('enable_o7_index2_calibration',False)
            self.declare_parameter('o7_index2_plan_path',str(ROOT/'calibration/index2_plan.example.json'))
            self.declare_parameter('enable_o7_retargeting',False)
            self.declare_parameter('o7_device_mapping_path',str(ROOT/'config/o7_device_linear.json'))
            self.declare_parameter('o7_solver_python',str(solver_python()))

        def _read_parameters(self):
            super()._read_parameters()
            if self.canfd_csv_log_directory=='~/teleop_ws/canfd_logs':
                self.canfd_csv_log_directory=str(workspace_root()/'canfd_logs')
            self.enable_o7_read_diag=bool(self.get_parameter('enable_o7_read_diag').value)
            self.enable_o7_index2_calibration=bool(self.get_parameter('enable_o7_index2_calibration').value)
            self.index2_plan=Index2Plan.load(str(self.get_parameter('o7_index2_plan_path').value))
            self.enable_o7_retargeting=bool(self.get_parameter('enable_o7_retargeting').value)
            self.o7_solver_python=str(self.get_parameter('o7_solver_python').value)
            from pico_o7.o7_stage.mapping import Mapper
            self.device_mapper=Mapper(str(self.get_parameter('o7_device_mapping_path').value))

        def _validate_parameters(self):
            super()._validate_parameters()
            if self.enable_o7_retargeting and not Path(self.o7_solver_python).is_file():
                raise ValueError('O7 solver interpreter missing: '+self.o7_solver_python+'; run tools/setup_o7_solver_env.py')
            if self.enable_o7_single_session_hil or self.enable_o7_stream_hil or self.enable_o7_auto_stream_hil:
                raise ValueError('This stage requires all three legacy HIL modes disabled; use the explicit index2 service')
            if sum((self.enable_o7_retargeting,self.enable_o7_index2_calibration,self.enable_o7_read_diag))>1:
                raise ValueError('Select only one O7 mode: realtime / one-shot / read diagnostic')
            if self.enable_o7_index2_calibration and self.allow_motion:
                raise ValueError('Independent index2 calibration requires allow_motion=false')

        def __init__(self):
            super().__init__()
            # All legacy modes were validated disabled before the connection worker started.
            self._hil=StageHil(self._emit_hil_event)
            self.stage_result_pub=self.create_publisher(String,'/rm65/o7_stage/result',
                QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self._read_all_service=self.create_service(Trigger,'/rm65/o7_diagnostics/read_positions_once',self._read_all)
            self._read_one_service=self.create_service(Trigger,'/rm65/o7_diagnostics/read_index_once',self._read_one)
            self._index_dry_service=self.create_service(Trigger,'/rm65/o7_calibration/index2/dry_run',self._dry)
            self._index_once_service=self.create_service(Trigger,'/rm65/o7_calibration/index2/write_once',self._index_once)
            self._latency_once_service=self.create_service(Trigger,'/rm65/o7_hand/latency_once',self._latency_once)
            self.hand=None
            self._hand_status_service=self.create_service(Trigger,'/rm65/o7_hand/status',self._hand_status)
            if self.enable_o7_retargeting:
                from pico_o7.o7_stage.realtime import HandController
                from rclpy.qos import qos_profile_sensor_data
                self.hand=HandController(self.device_mapper,self._hand_adapter,self._hand_rejection,self._emit_hil_event,
                                         start_from_parameter=True,solver_python=self.o7_solver_python)
                okay,reason=self._hil.start_hand_worker(self.hand)
                if not okay:
                    self.hand.shutdown();raise RuntimeError(reason)
                self._hand_sub=self.create_subscription(String,'/pico/right_hand_raw',self._hand_input,qos_profile_sensor_data)
                self._hand_status_timer=self.create_timer(1.,self._hand_report)
                self.get_logger().warning('O7 enabled by launch parameter; waiting for safe session/ARM and NEW tracked frame; no position reads')
            self.get_logger().warning('O7 stage: same API2 session and HIL worker; '+json.dumps(self.index2_plan.dry_run()))

        def _emit_hil_event(self,event):
            super()._emit_hil_event(event)
            if hasattr(self,'stage_result_pub'):
                self.stage_result_pub.publish(String(data=json.dumps(event,ensure_ascii=False)))

        def _publish_control_state(self,**v):
            super()._publish_control_state(**v)
            if self.enable_o7_index2_calibration and self.motion_permitted:
                self._hil.record(time.monotonic(),v['state']=='ACTIVE'and v['command_sent']and not v['fault_latched'])

        def _read_rejection(self):
            if not self.enable_o7_read_diag:return 'enable_o7_read_diag=false'
            # Strong static gate: no mid-read transition to CANFD ACTIVE is possible.
            if self.motion_permitted:return 'read diagnostics require allow_motion=false; no reads during ARM/calibration motion'
            if self._stop_event.is_set():return 'node_closing'
            with self._adapter_lock:
                if not self._connected or self._adapter is None or self._adapter.arm is None:return 'existing API2 session not connected'
            return ''

        def _read(self,response,index):
            rejection=self._read_rejection()
            if rejection:
                response.success=False;response.message=rejection;return response
            with self._adapter_lock:adapter=self._adapter
            accepted,message=self._hil.trigger_read(adapter,self._read_rejection,index=index)
            response.success=accepted
            response.message=json.dumps(dict(request_id=message,result_event='O7_READ_FINISH'))if accepted else message
            return response

        def _read_all(self,request,response):return self._read(response,None)
        def _read_one(self,request,response):return self._read(response,2)

        def _dry(self,request,response):
            response.success=True
            response.message=json.dumps(self.index2_plan.dry_run(),ensure_ascii=False)
            return response

        def _index_once(self,request,response):
            if not self.enable_o7_index2_calibration:
                response.success=False;response.message='enable_o7_index2_calibration=false';return response
            try:self.index2_plan.validate()
            except ValueError as exc:
                response.success=False;response.message=str(exc);return response
            with self._adapter_lock:adapter=self._adapter
            accepted,message=self._hil.trigger_index_locked(adapter,self.index2_plan,self._index_rejection)
            response.success=accepted
            response.message=json.dumps(dict(message=message,plan_sha256=self.index2_plan.hash,
                request_id=self._hil.last_request_id if accepted else None,result_event='STREAM_FINISH'))
            return response

        def _hand_adapter(self):
            with self._adapter_lock:return self._adapter

        def _hand_rejection(self):
            if self._stop_event.is_set():return 'node_closing'
            if self._fault_latched:return 'ARM_fault_latched'
            with self._adapter_lock:
                if not self._connected or self._adapter is None or self._adapter.arm is None:return 'existing API2 session not connected'
            # Locked-arm following remains independent of ARM ACTIVE. Combined
            # following uses the original ARM safety snapshot, without SDK calls.
            if self.motion_permitted:
                reason=self._hil_rejection()
                if reason:return reason
                last=self._last_command_send_monotonic
                if last is None or not 0<=time.monotonic()-last<=.010:
                    return 'CANFD_not_continuously_sending'
            return ''

        def _index_rejection(self):
            if not self.enable_o7_index2_calibration:return 'index2 calibration disabled'
            if self.motion_permitted:return 'index2 calibration requires allow_motion=false'
            return self._hand_rejection()

        def _latency_rejection(self):
            if self.motion_permitted:return 'fixed-target latency comparison requires allow_motion=false'
            return self._hand_rejection()

        def _latency_once(self,request,response):
            if self.hand is None:
                response.success=False;response.message='Requires hand-start environment';return response
            request_id=uuid.uuid4().hex
            okay,message=self.hand.schedule_latency_once(self.index2_plan,request_id,self._latency_rejection)
            response.success=okay
            response.message=json.dumps(dict(request_id=request_id,result_event='STREAM_FINISH',
                target_positions=list(self.index2_plan.target),message=message))if okay else message
            return response

        def _hand_input(self,msg):
            try:self.hand.state.receive(json.loads(msg.data))
            except (ValueError,TypeError,KeyError):self.hand.state.clear('malformed_hand_topic')

        def _hand_status(self,request,response):
            response.success=True;response.message=json.dumps(self.hand.state.status()if self.hand else dict(enabled=False,reason='mode not prepared'))
            return response

        def _hand_report(self):
            if self.hand is not None:self._emit_hil_event(dict(event='O7_HAND_STATUS',timestamp_ns=time.time_ns(),**self.hand.state.status()))

        def destroy_node(self):
            if getattr(self,'hand',None)is not None:
                self.hand.shutdown()
                self._hil.begin_close()
                self._hil.wait_closed(self.get_logger().error)
            return super().destroy_node()

    original.Rm65CanfdHighFollowNode=SharedO7Node
    from pico_o7.o7_stage.write_timing import session_lease
    with session_lease():original.main(args=args)


if __name__=='__main__':main()
