#!/usr/bin/env python3
"""Inspect and dry-run offline, or explicitly request existing shared-node services."""
import argparse
import json
from pathlib import Path
import time
from pico_o7.o7_stage.plan import Index2Plan
from pico_o7.o7_stage.protocol import READ_CODES

from .paths import share_dir
ROOT=share_dir()
SERVICES={'read':'/rm65/o7_diagnostics/read_positions_once',
          'read-index':'/rm65/o7_diagnostics/read_index_once',
          'node-dry-run':'/rm65/o7_calibration/index2/dry_run',
          'write-once':'/rm65/o7_calibration/index2/write_once',
          'latency-once':'/rm65/o7_hand/latency_once',
          'hand-status':'/rm65/o7_hand/status'}


def request(command,timeout):
    import rclpy
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    rclpy.init()
    node=rclpy.create_node('o7_shared_session_request')
    events=[]
    def received(msg):
        try:events.append(json.loads(msg.data))
        except ValueError:pass
    subscription=node.create_subscription(String,'/rm65/o7_stage/result',received,
        QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client=node.create_client(Trigger,SERVICES[command])
    try:
        if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('Service unavailable; use the shared-stage replacement node, not a second SDK client')
        # Discover the result publisher before starting an operation; no automatic retry.
        deadline=time.monotonic()+timeout
        if command!='node-dry-run':
            while node.count_publishers('/rm65/o7_stage/result')<1:
                if time.monotonic()>=deadline:raise RuntimeError('Result publisher unavailable; no operation requested')
                rclpy.spin_once(node,timeout_sec=.05)
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise RuntimeError('Service response timeout; no automatic resend')
        response=future.result()
        if not response.success:
            print(json.dumps(dict(accepted=False,message=response.message),ensure_ascii=False,indent=2));return 3
        info=json.loads(response.message)
        if command=='node-dry-run'or command.startswith('hand-'):print(json.dumps(info,ensure_ascii=False,indent=2));return 0
        print(json.dumps(dict(accepted=True,service=SERVICES[command],**info),ensure_ascii=False),flush=True)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            for event in events:
                if command in ('read','read-index'):
                    match=event.get('event')=='O7_READ_FINISH'and event.get('request_id')==info['request_id']
                else:
                    match=event.get('event')=='STREAM_FINISH'and event.get('mode')=='index2_once'and event.get('request_id')==info['request_id']
                if match:
                    print(json.dumps(event,ensure_ascii=False,indent=2))
                    # Exit status describes service/call completion, NEVER physical write success.
                    okay=event.get('valid') if command in ('read','read-index') else event.get('calls_returned')==1
                    return 0 if okay else 2
            rclpy.spin_once(node,timeout_sec=.05)
        raise RuntimeError('Result wait timeout; operation may be in flight. Do not resend or delete API2 handle.')
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['inspect','dry-run',*SERVICES])
    parser.add_argument('--plan',type=Path,default=ROOT/'calibration/index2_plan.example.json')
    parser.add_argument('--timeout',type=float,default=15.)
    parser.add_argument('--execute-index2',action='store_true',help='Explicitly request ONE real write through the existing node')
    parser.add_argument('--execute-latency',action='store_true',help='One known small target in locked hand-start environment; no PICO submissions')
    args=parser.parse_args()
    if args.command=='inspect':
        print(json.dumps(dict(teleop_ws_existing_read_entry=False,
            bulk_sdk='rm_read_multiple_input_registers(port=1,device=39,address=0,num=7)',
            single_sdk='rm_read_input_registers(port=1,device=39,address=2,num=1)',
            read_codes=READ_CODES,write_return_code_policy='raw log only; never infers physical write success or gates writes',
            note='Read code1 and write code1 have different local SDK docstrings; never decode nonzero-return raw zeros as positions.'),ensure_ascii=False,indent=2));return 0
    if args.command=='dry-run':
        print(json.dumps(Index2Plan.load(args.plan).dry_run(),ensure_ascii=False,indent=2));return 0
    if args.timeout<=0:parser.error('timeout must be positive')
    if args.command=='write-once'and not args.execute_index2:parser.error('write-once requires --execute-index2; default is no hardware request')
    if args.command=='latency-once'and not args.execute_latency:parser.error('latency-once requires --execute-latency')
    return request(args.command,args.timeout)


if __name__=='__main__':raise SystemExit(main())
