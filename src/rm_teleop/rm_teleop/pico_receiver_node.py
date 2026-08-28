from __future__ import annotations

import socket
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from pico_rm65_teleop_interfaces.msg import PicoTeleopState

from .protocol import PoseData, ProtocolConfig, TeleopPacket
from .receiver_core import ReceiverCore, SequenceStatus

MAX_UDP_DATAGRAM = 65535


class PicoReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("pico_receiver_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5005)
        self.declare_parameter("allowed_source_ip", "")
        self.declare_parameter("frame_id", "pico_tracking")
        self.declare_parameter("state_topic", "/pico/teleop_state")
        self.declare_parameter("status_topic", "/pico/status")
        self.declare_parameter("poll_hz", 200.0)
        self.declare_parameter("diagnostics_hz", 2.0)
        self.declare_parameter("receiver_timeout_sec", 0.15)
        self.declare_parameter("safe_state_repeat_sec", 0.10)
        self.declare_parameter("sequence_reset_after_sec", 1.0)
        self.declare_parameter("source_restart_rollback_sec", 0.5)
        self.declare_parameter("quaternion_norm_min", 0.5)
        self.declare_parameter("quaternion_norm_max", 1.5)
        self.declare_parameter("max_abs_position_m", 10.0)
        self.declare_parameter("socket_receive_buffer_bytes", 1048576)
        self.declare_parameter("max_packets_per_poll", 100)
        self.declare_parameter("log_first_packets", 3)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.allowed_source_ip = str(
            self.get_parameter("allowed_source_ip").value
        ).strip()
        self.frame_id = str(self.get_parameter("frame_id").value)
        state_topic = str(self.get_parameter("state_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        poll_hz = float(self.get_parameter("poll_hz").value)
        diagnostics_hz = float(self.get_parameter("diagnostics_hz").value)
        self.receiver_timeout_sec = float(
            self.get_parameter("receiver_timeout_sec").value
        )
        self.safe_state_repeat_sec = float(
            self.get_parameter("safe_state_repeat_sec").value
        )
        sequence_reset_after_sec = float(
            self.get_parameter("sequence_reset_after_sec").value
        )
        source_restart_rollback_sec = float(
            self.get_parameter("source_restart_rollback_sec").value
        )
        quaternion_norm_min = float(
            self.get_parameter("quaternion_norm_min").value
        )
        quaternion_norm_max = float(
            self.get_parameter("quaternion_norm_max").value
        )
        max_abs_position_m = float(
            self.get_parameter("max_abs_position_m").value
        )
        socket_receive_buffer_bytes = int(
            self.get_parameter("socket_receive_buffer_bytes").value
        )
        self.max_packets_per_poll = int(
            self.get_parameter("max_packets_per_poll").value
        )
        self.log_first_packets = int(
            self.get_parameter("log_first_packets").value
        )

        self._validate_parameters(
            poll_hz=poll_hz,
            diagnostics_hz=diagnostics_hz,
            quaternion_norm_min=quaternion_norm_min,
            quaternion_norm_max=quaternion_norm_max,
        )

        protocol_config = ProtocolConfig(
            quaternion_norm_min=quaternion_norm_min,
            quaternion_norm_max=quaternion_norm_max,
            max_abs_position_m=max_abs_position_m,
        )
        self.core = ReceiverCore(
            protocol_config=protocol_config,
            sequence_reset_after_sec=sequence_reset_after_sec,
            source_restart_rollback_sec=source_restart_rollback_sec,
        )

        self.state_pub = self.create_publisher(
            PicoTeleopState,
            state_topic,
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            DiagnosticArray,
            status_topic,
            10,
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_RCVBUF,
            socket_receive_buffer_bytes,
        )
        try:
            self.sock.bind((self.host, self.port))
        except OSError as exc:
            self.sock.close()
            self.get_logger().error(
                f"failed to bind udp://{self.host}:{self.port}: {exc}"
            )
            raise
        self.sock.setblocking(False)

        self.source_rejected = 0
        self.socket_errors = 0
        self.published_states = 0
        self.safe_states = 0
        self.logged_packets = 0
        self.last_peer = ""
        self.last_result_reason = "startup"
        self.last_safe_publish_monotonic = float("-inf")
        self.last_warning_times: dict[str, float] = {}

        self.poll_timer = self.create_timer(1.0 / poll_hz, self.on_poll_timer)
        self.diagnostics_timer = self.create_timer(
            1.0 / diagnostics_hz,
            self.publish_diagnostics,
        )

        source_text = self.allowed_source_ip or "any"
        actual_buffer = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.get_logger().info(
            f"listening udp://{self.host}:{self.port} "
            f"allowed_source_ip={source_text} frame_id={self.frame_id} "
            f"socket_rcvbuf={actual_buffer}"
        )

    def _validate_parameters(
        self,
        poll_hz: float,
        diagnostics_hz: float,
        quaternion_norm_min: float,
        quaternion_norm_max: float,
    ) -> None:
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        if poll_hz <= 0.0:
            raise ValueError("poll_hz must be positive")
        if diagnostics_hz <= 0.0:
            raise ValueError("diagnostics_hz must be positive")
        if self.receiver_timeout_sec <= 0.0:
            raise ValueError("receiver_timeout_sec must be positive")
        if self.safe_state_repeat_sec <= 0.0:
            raise ValueError("safe_state_repeat_sec must be positive")
        if self.max_packets_per_poll <= 0:
            raise ValueError("max_packets_per_poll must be positive")
        if quaternion_norm_min <= 0.0:
            raise ValueError("quaternion_norm_min must be positive")
        if quaternion_norm_max <= quaternion_norm_min:
            raise ValueError(
                "quaternion_norm_max must be greater than quaternion_norm_min"
            )

    def on_poll_timer(self) -> None:
        packets_read = 0

        while packets_read < self.max_packets_per_poll:
            try:
                data, addr = self.sock.recvfrom(MAX_UDP_DATAGRAM)
            except BlockingIOError:
                break
            except OSError as exc:
                self.socket_errors += 1
                self.last_result_reason = "socket_error"
                self._warn_throttled("socket", f"UDP receive error: {exc}")
                break

            packets_read += 1
            peer = f"{addr[0]}:{addr[1]}"
            self.last_peer = peer

            if self.allowed_source_ip and addr[0] != self.allowed_source_ip:
                self.source_rejected += 1
                self.last_result_reason = "source_rejected"
                self._warn_throttled(
                    "source",
                    f"rejected UDP packet from {peer}; "
                    f"allowed_source_ip={self.allowed_source_ip}",
                )
                continue

            now_monotonic = time.monotonic()
            result = self.core.process_datagram(data, now_monotonic)
            self.last_result_reason = result.reason

            if not result.accepted or result.packet is None:
                self._warn_throttled(
                    result.reason,
                    f"discarded UDP packet from={peer} bytes={len(data)} "
                    f"reason={result.reason}",
                )
                continue

            if self.logged_packets < self.log_first_packets:
                self.logged_packets += 1
                self.get_logger().info(
                    f"accepted UDP packet {self.logged_packets}/"
                    f"{self.log_first_packets} from={peer} bytes={len(data)} "
                    f"seq={result.packet.seq} mode={result.packet.input_mode}"
                )

            if result.sequence_status == SequenceStatus.DROP:
                self._warn_throttled(
                    "sequence_drop",
                    f"sequence gap from={peer} seq={result.packet.seq} "
                    f"dropped={result.dropped}",
                )
            elif result.sequence_status in (
                SequenceStatus.RESET_AFTER_GAP,
                SequenceStatus.SOURCE_RESTART,
            ):
                self.get_logger().warn(
                    f"sequence tracker reset status={result.sequence_status.value} "
                    f"seq={result.packet.seq} source_timestamp="
                    f"{result.packet.source_timestamp_sec:.3f}"
                )

            self.publish_packet(result.packet, result.sequence_status)

        now_monotonic = time.monotonic()
        timeout_became_active = self.core.latch_timeout_if_new(
            now_monotonic,
            self.receiver_timeout_sec,
        )
        timed_out = self.core.is_timed_out(
            now_monotonic,
            self.receiver_timeout_sec,
        )

        if timeout_became_active:
            age = self.core.accepted_age_sec(now_monotonic)
            age_text = "never" if age is None else f"{age:.3f}s"
            self.get_logger().warn(
                f"receiver timeout: last accepted packet age={age_text}"
            )

        if (
            timed_out
            and now_monotonic - self.last_safe_publish_monotonic
            >= self.safe_state_repeat_sec
        ):
            self.publish_safe_state("receiver_timeout")
            self.last_safe_publish_monotonic = now_monotonic

    def publish_packet(
        self,
        packet: TeleopPacket,
        sequence_status: SequenceStatus | None,
    ) -> None:
        msg = PicoTeleopState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.protocol_version = packet.protocol_version
        msg.seq = packet.seq
        msg.source_timestamp_sec = packet.source_timestamp_sec
        msg.target_hz = packet.target_hz

        msg.packet_valid = True
        msg.transport_alive = True
        msg.deadman = packet.deadman
        msg.hand_teleop_enabled = packet.hand_teleop_enabled
        msg.teleop_armed = packet.teleop_armed
        msg.left_controller_connected = packet.left_controller_connected
        msg.right_hand_tracked = packet.right_hand_tracked

        msg.input_mode = packet.input_mode
        msg.stop_reason = packet.stop_reason
        msg.receiver_reason = (
            "accepted"
            if sequence_status is None
            else f"accepted:{sequence_status.value}"
        )

        msg.reset_origin = packet.reset_origin
        msg.record_toggle = packet.record_toggle

        msg.palm_valid = packet.palm.valid
        msg.palm_pose = self._pose_message(packet.palm)
        msg.wrist_valid = packet.wrist.valid
        msg.wrist_pose = self._pose_message(packet.wrist)

        self.state_pub.publish(msg)
        self.published_states += 1

    def publish_safe_state(self, reason: str) -> None:
        msg = PicoTeleopState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        last_packet = self.core.last_accepted_packet
        if last_packet is not None:
            msg.protocol_version = last_packet.protocol_version
            msg.seq = last_packet.seq
            msg.source_timestamp_sec = last_packet.source_timestamp_sec
            msg.target_hz = last_packet.target_hz

        msg.packet_valid = False
        msg.transport_alive = False
        msg.deadman = False
        msg.hand_teleop_enabled = False
        msg.teleop_armed = False
        msg.left_controller_connected = False
        msg.right_hand_tracked = False
        msg.input_mode = ""
        msg.stop_reason = ""
        msg.receiver_reason = reason
        msg.reset_origin = False
        msg.record_toggle = False
        msg.palm_valid = False
        msg.palm_pose.orientation.w = 1.0
        msg.wrist_valid = False
        msg.wrist_pose.orientation.w = 1.0

        self.state_pub.publish(msg)
        self.published_states += 1
        self.safe_states += 1

    @staticmethod
    def _pose_message(pose: PoseData) -> Pose:
        msg = Pose()
        msg.position.x = pose.position.x
        msg.position.y = pose.position.y
        msg.position.z = pose.position.z
        msg.orientation.x = pose.rotation.x
        msg.orientation.y = pose.rotation.y
        msg.orientation.z = pose.rotation.z
        msg.orientation.w = pose.rotation.w
        return msg

    def publish_diagnostics(self) -> None:
        now_monotonic = time.monotonic()
        timed_out = self.core.is_timed_out(
            now_monotonic,
            self.receiver_timeout_sec,
        )
        accepted_age = self.core.accepted_age_sec(now_monotonic)
        valid_palm_age = self.core.valid_palm_age_sec(now_monotonic)
        counters = self.core.counters
        last_packet = self.core.last_accepted_packet

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "pico_receiver"
        status.hardware_id = self.last_peer or "pico_udp"
        status.level = DiagnosticStatus.WARN if timed_out else DiagnosticStatus.OK
        status.message = "receiver_timeout" if timed_out else "receiving"
        status.values = [
            KeyValue(key="last_peer", value=self.last_peer or "none"),
            KeyValue(
                key="accepted_age_sec",
                value="never" if accepted_age is None else f"{accepted_age:.6f}",
            ),
            KeyValue(
                key="valid_palm_age_sec",
                value=(
                    "never"
                    if valid_palm_age is None
                    else f"{valid_palm_age:.6f}"
                ),
            ),
            KeyValue(key="datagrams", value=str(counters.datagrams)),
            KeyValue(key="accepted", value=str(counters.accepted)),
            KeyValue(key="invalid", value=str(counters.invalid)),
            KeyValue(key="dropped", value=str(counters.dropped)),
            KeyValue(key="duplicates", value=str(counters.duplicates)),
            KeyValue(key="out_of_order", value=str(counters.out_of_order)),
            KeyValue(
                key="sequence_resets",
                value=str(counters.sequence_resets),
            ),
            KeyValue(
                key="source_restarts",
                value=str(counters.source_restarts),
            ),
            KeyValue(key="timeouts", value=str(counters.timeouts)),
            KeyValue(key="source_rejected", value=str(self.source_rejected)),
            KeyValue(key="socket_errors", value=str(self.socket_errors)),
            KeyValue(key="published_states", value=str(self.published_states)),
            KeyValue(key="safe_states", value=str(self.safe_states)),
            KeyValue(key="last_result_reason", value=self.last_result_reason),
            KeyValue(
                key="invalid_reasons",
                value=";".join(
                    f"{key}:{value}"
                    for key, value in sorted(counters.invalid_reasons.items())
                ),
            ),
            KeyValue(
                key="last_seq",
                value="none" if last_packet is None else str(last_packet.seq),
            ),
            KeyValue(
                key="last_input_mode",
                value="none" if last_packet is None else last_packet.input_mode,
            ),
            KeyValue(
                key="last_hand_teleop_enabled",
                value=(
                    "false"
                    if last_packet is None
                    else str(last_packet.hand_teleop_enabled).lower()
                ),
            ),
        ]

        array.status.append(status)
        self.status_pub.publish(array)

    def _warn_throttled(
        self,
        key: str,
        message: str,
        period_sec: float = 2.0,
    ) -> None:
        now = time.monotonic()
        last = self.last_warning_times.get(key, float("-inf"))
        if now - last >= period_sec:
            self.last_warning_times[key] = now
            self.get_logger().warn(message)

    def destroy_node(self) -> bool:
        try:
            self.sock.close()
        finally:
            return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: PicoReceiverNode | None = None

    try:
        node = PicoReceiverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
