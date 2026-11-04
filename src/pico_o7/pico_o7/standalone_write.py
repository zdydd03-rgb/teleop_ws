#!/usr/bin/python3
"""Position-only derivative of old standalone O7 scripts; defaults to zero-I/O dry-run.
Run only AFTER hand-start/teleop exits. One API2 client, no ROS, no ARM motion API,
no port configuration, no speed/torque write, no readback, no retry/return write.
"""
import argparse
import json
from pathlib import Path
import sys
import time
from pico_o7.o7_stage.plan import Index2Plan
from pico_o7.o7_stage.write_timing import session_lease
from .paths import share_dir, workspace_root
ROOT=share_dir()

def run(plan,mode,output,*,module=None):
    # module injection is for offline unit tests only; normal execution loads API2 HERE.
    plan.validate()
    from rm_teleop.rm_api2_adapter import load_api2_module
    from rm_teleop.rm_api2_motion_adapter import RmApi2MotionAdapter
    from pico_o7.o7_stage.worker import run_index_once
    module=module or load_api2_module('/home/ai/RM_API2/Python')
    output.mkdir(parents=True,exist_ok=False)
    with (output/'events.jsonl').open('w')as stream:
        def emit(event):
            event=dict(event,comparison_environment='standalone_no_ROS_no_ARM_motion',thread_mode=mode,
                       port=1,slave=39,address=0,num=7,fc=16)
            line=json.dumps(event,ensure_ascii=False,allow_nan=False)
            stream.write(line+'\n');stream.flush();print(line,flush=True)
        arm=module.RoboticArm(getattr(module.rm_thread_mode_e,'RM_'+mode+'_MODE_E'))
        connected=False
        try:
            handle=arm.rm_create_robot_arm('192.168.125.19',8080,level=3)
            if int(handle.id)<0:raise RuntimeError('API2 connection not established; no position call')
            connected=True
            emit(dict(event='STANDALONE_CONNECTED',timestamp_ns=time.time_ns(),handle_id=int(handle.id),
                sdk_module_path=getattr(module,'__file__',None),robot_ip='192.168.125.19',tcp_port=8080,
                port_configuration='assumed already configured; NOT read/set',speed_write=False,position_read=False,
                realtime_push_configuration='unchanged; no UDP callback registered',arm_motion_api_calls=0))
            # Borrow ONLY the inspected Adapter's position method; never run connect()
            # (which initializes ARM realtime feedback/IK). Same original encoder/FC16.
            adapter=object.__new__(RmApi2MotionAdapter)
            adapter.arm=arm;adapter.module=module
            run_index_once(adapter,plan,emit,lambda:'',validated=True)
        finally:
            if connected:
                code=arm.rm_delete_robot_arm()
                emit(dict(event='STANDALONE_DISCONNECTED',timestamp_ns=time.time_ns(),raw_return_code=code))
    return 0 # Process completion only, never a physical-write success result.

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,default=ROOT/'calibration/index2_plan.example.json')
    p.add_argument('--thread-mode',choices=['TRIPLE','DUAL'],default='TRIPLE',
        help='TRIPLE matches hand-start and old backup; DUAL matches current old speed_test, a separate factor')
    p.add_argument('--output',type=Path,default=None)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--confirm-other-session-stopped',action='store_true')
    a=p.parse_args();plan=Index2Plan.load(a.plan)
    dry=dict(plan.dry_run(),entry='position-only standalone derivative, NOT unmodified legacy script',
             thread_mode=a.thread_mode,port=1,slave=39,address=0,num=7,fc=16,
             port_setup_calls=0,speed_writes=0,read_calls=0,arm_motion_calls=0,
             automatic_return_write=False)
    if not a.execute:
        print(json.dumps(dry,ensure_ascii=False,indent=2));return 0
    if not a.confirm_other_session_stopped:p.error('--confirm-other-session-stopped required; NEVER run beside hand-start or old SDK client')
    plan.validate()
    output=a.output or workspace_root()/'concurrency_logs/o7_write_latency'/('standalone_'+time.strftime('%Y%m%d_%H%M%S'))
    with session_lease():return run(plan,a.thread_mode,output)
if __name__=='__main__':raise SystemExit(main())
