"""ROS subscriptions only, except explicitly requested Trigger service invocation."""
import argparse
from datetime import datetime
import json
import time

from .hil_monitor_data import Capture, diagnostic_level


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from pico_rm65_teleop_interfaces.msg import RmCanfdControlState

    parser = argparse.ArgumentParser(description=__doc__)
    execution = parser.add_mutually_exclusive_group()
    execution.add_argument('--observe-auto-stream', action='store_true',
                           help='Passively observe automatic HIL; never send a Trigger request')
    execution.add_argument('--execute-stream', action='store_true',
                           help='Request ONE real 1Hz index-finger movement HIL after 10s ACTIVE')
    execution.add_argument('--execute-injection', action='store_true',
                        help='After 10s continuous ACTIVE baseline, request ONE real O7 speed write')
    parser.add_argument('--duration', type=float, default=30,
                        help='Capture/event wait deadline seconds; use 120 for automatic 60s HIL (default 30)')
    parser.add_argument('--output', default='concurrency_logs/single_session_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    options, ros_args = parser.parse_known_args(args)
    execute = options.execute_injection or options.execute_stream
    if options.duration < 10:
        parser.error('--duration must be >=10')
    rclpy.init(args=ros_args)
    capture = Capture()

    class Monitor(Node):
        def __init__(self):
            super().__init__('single_session_hil_monitor')
            self.started = time.monotonic()
            self.active_since = None
            self.last_control = None
            self.request = None
            self.end_received = None
            self.deadline = self.started + options.duration
            self.done = False
            self.failure = False
            qos = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(RmCanfdControlState, '/rm65/canfd_control_state', self.control, qos)
            self.create_subscription(DiagnosticArray, '/rm65/status', self.status, qos)
            self.create_subscription(String, '/rm65/o7_modbus_hil/event', self.event, qos)
            self.client = self.create_client(Trigger, '/rm65/o7_stream_hil/start' if options.execute_stream
                                             else '/rm65/o7_modbus_hil/trigger_once') if execute else None
            self.create_timer(.02, self.tick)
            self.get_logger().info(f'Capturing; execute_injection={options.execute_injection}; execute_stream={options.execute_stream}')

        def control(self, msg):
            now = time.monotonic()
            capture.control(msg, stamp_ns(msg.header.stamp), time.time_ns())
            healthy = (msg.state == 'ACTIVE' and msg.command_sent and not msg.fault_latched
                       and msg.motion_permitted and msg.ik_initialized and 0 <= msg.feedback_age_sec <= .05)
            if not healthy or (self.last_control is not None and now - self.last_control > .05):
                self.active_since = None
            elif self.active_since is None:
                self.active_since = now
            self.last_control = now

        def status(self, msg):
            data = [dict(name=s.name, level=diagnostic_level(s.level), message=s.message,
                         hardware_id=s.hardware_id, values={v.key: v.value for v in s.values}) for s in msg.status]
            capture.rows.append(dict(timestamp_ns=stamp_ns(msg.header.stamp), rx_timestamp_ns=time.time_ns(),
                                     record_type='diagnostic', diagnostics_json=json.dumps(data)))

        def event(self, msg):
            try:
                data = json.loads(msg.data)
                capture.event(data, time.time_ns())
                if data['event'] == 'AUTO_STREAM_START' and options.observe_auto_stream:
                    self.deadline = time.monotonic() + options.duration
                if data['event'] in ('MODBUS_FINISH', 'MODBUS_EXCEPTION', 'STREAM_FINISH', 'AUTO_STREAM_FINISH'):
                    self.end_received = time.monotonic()
                    if data['event'] in ('STREAM_FINISH', 'AUTO_STREAM_FINISH'):
                        # Completion is a scheduling result, never proof of physical motion.
                        self.failure = (not data.get('schedule_completed', data.get('success', False))
                                        or data.get('sdk_exception_count', 0) > 0)
                    else:
                        self.failure = data['event'] == 'MODBUS_EXCEPTION' or data.get('return_code') != 0
                self.get_logger().info(msg.data)
            except (ValueError, KeyError) as exc:
                self.get_logger().error('Invalid HIL event: ' + str(exc))

        def tick(self):
            now = time.monotonic()
            if self.end_received is not None:
                self.done = now - self.end_received >= 10.1
                return
            if execute and self.request is None:
                if (self.active_since is not None and now - self.active_since >= 10
                        and self.last_control is not None and now - self.last_control < .05
                        and self.client.service_is_ready()):
                    self.request = self.client.call_async(Trigger.Request())
                    self.deadline = now + options.duration
                    self.get_logger().warning('Sending one Trigger request; waiting for service acceptance and actual SDK events')
                    self.request.add_done_callback(self.response)
            if now >= self.deadline:
                self.failure = execute or capture.start is not None
                self.get_logger().warning('Capture deadline reached; missing events/window remain incomplete')
                self.done = True

        def response(self, future):
            try:
                response = future.result()
                capture.event(dict(event='TRIGGER_RESPONSE', timestamp_ns=time.time_ns(),
                                   accepted=response.success, message=response.message), time.time_ns())
                self.get_logger().info(f'Trigger accepted={response.success}: {response.message}')
                if not response.success:
                    self.failure = self.done = True
            except Exception as exc:
                self.get_logger().error('Trigger failed: ' + str(exc))
                self.failure = self.done = True

    node = Monitor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.failure = True
    finally:
        summary = capture.save(options.output)
        print(f'Output: {options.output}; complete_window={summary["complete_window"]}')
        if summary['stream']['completion'] is not None:
            print(json.dumps({k: summary['stream'][k] for k in (
                'sdk_acknowledged_success_count', 'sdk_unconfirmed_command_count',
                'sdk_exception_count', 'actual_motion', 'last_target_positions')}, indent=2))
        if 'auto_stream' in summary:
            print(json.dumps(summary['auto_stream'], indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
