from __future__ import annotations

import math
import time
from typing import Sequence

from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from pico_rm65_teleop_interfaces.msg import PicoTeleopState, RmTeleopTarget

from .target_generator_core import (
    FlangeInput,
    PicoInput,
    TargetGeneratorConfig,
    TargetGeneratorCore,
)
from .target_math import IDENTITY_POSE, Pose3D, Vector3, validate_pose


class TargetGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__("target_generator_node")

        self.declare_parameter("pico_topic", "/pico/teleop_state")
        self.declare_parameter("flange_pose_topic", "/rm65/current_flange_pose")
        self.declare_parameter("target_topic", "/rm_teleop/flange_target")
        self.declare_parameter("flange_frame_id", "rm65_base")
        self.declare_parameter("require_flange_frame_id_match", True)

        self.declare_parameter("control_hz", 100.0)
        self.declare_parameter("pico_freshness_timeout_sec", 0.10)
        self.declare_parameter("flange_freshness_timeout_sec", 0.10)
        self.declare_parameter("position_scale", 0.25)
        self.declare_parameter("max_translation_speed_mps", 0.025)
        self.declare_parameter("max_rotation_speed_deg_s", 12.0)
        for axis in ("rx", "ry", "rz"):
            self.declare_parameter(f"max_rotation_{axis}_speed_deg_s", -1.0)

        self.declare_parameter("motion_gate_enabled", True)
        self.declare_parameter("input_filter_time_constant_sec", 0.060)
        self.declare_parameter("motion_detection_window_sec", 0.40)
        self.declare_parameter("motion_start_translation_m", 0.004)
        self.declare_parameter("motion_start_rotation_deg", 2.0)
        self.declare_parameter("motion_stop_translation_m", 0.0035)
        self.declare_parameter("motion_stop_rotation_deg", 1.5)
        self.declare_parameter("motion_stop_hold_sec", 0.30)

        self.declare_parameter("workspace_limit_enabled", False)
        self.declare_parameter("workspace_min_xyz", [-10.0, -10.0, -10.0])
        self.declare_parameter("workspace_max_xyz", [10.0, 10.0, 10.0])

        self.pico_topic = str(self.get_parameter("pico_topic").value)
        self.flange_pose_topic = str(
            self.get_parameter("flange_pose_topic").value
        )
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.flange_frame_id = str(
            self.get_parameter("flange_frame_id").value
        )
        self.require_flange_frame_id_match = bool(
            self.get_parameter("require_flange_frame_id_match").value
        )

        control_hz = float(self.get_parameter("control_hz").value)
        config = TargetGeneratorConfig(
            control_hz=control_hz,
            pico_freshness_timeout_sec=float(
                self.get_parameter("pico_freshness_timeout_sec").value
            ),
            flange_freshness_timeout_sec=float(
                self.get_parameter("flange_freshness_timeout_sec").value
            ),
            position_scale=float(self.get_parameter("position_scale").value),
            max_translation_speed_mps=float(
                self.get_parameter("max_translation_speed_mps").value
            ),
            max_rotation_speed_rad_s=math.radians(
                float(self.get_parameter("max_rotation_speed_deg_s").value)
            ),
            max_rotation_axis_speeds_rad_s=tuple(
                math.radians(
                    float(self.get_parameter("max_rotation_speed_deg_s").value)
                    if float(self.get_parameter(f"max_rotation_{axis}_speed_deg_s").value) == -1.0
                    else float(self.get_parameter(f"max_rotation_{axis}_speed_deg_s").value)
                ) for axis in ("rx", "ry", "rz")
            ),
            motion_gate_enabled=bool(
                self.get_parameter("motion_gate_enabled").value
            ),
            input_filter_time_constant_sec=float(
                self.get_parameter("input_filter_time_constant_sec").value
            ),
            motion_detection_window_sec=float(
                self.get_parameter("motion_detection_window_sec").value
            ),
            motion_start_translation_m=float(
                self.get_parameter("motion_start_translation_m").value
            ),
            motion_start_rotation_rad=math.radians(
                float(self.get_parameter("motion_start_rotation_deg").value)
            ),
            motion_stop_translation_m=float(
                self.get_parameter("motion_stop_translation_m").value
            ),
            motion_stop_rotation_rad=math.radians(
                float(self.get_parameter("motion_stop_rotation_deg").value)
            ),
            motion_stop_hold_sec=float(
                self.get_parameter("motion_stop_hold_sec").value
            ),
            workspace_limit_enabled=bool(
                self.get_parameter("workspace_limit_enabled").value
            ),
            workspace_min_xyz=self._parameter_vector3("workspace_min_xyz"),
            workspace_max_xyz=self._parameter_vector3("workspace_max_xyz"),
        )
        config.validate()
        self.core = TargetGeneratorCore(config)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pico_sub = self.create_subscription(
            PicoTeleopState,
            self.pico_topic,
            self.on_pico_state,
            sensor_qos,
        )
        self.flange_sub = self.create_subscription(
            PoseStamped,
            self.flange_pose_topic,
            self.on_flange_pose,
            sensor_qos,
        )
        self.target_pub = self.create_publisher(
            RmTeleopTarget,
            self.target_topic,
            target_qos,
        )

        self._last_state_key: tuple[str, str, bool] | None = None
        self._last_flange_error = ""
        self._last_flange_error_log_time = 0.0

        self.timer = self.create_timer(1.0 / control_hz, self.on_timer)

        self.get_logger().info(
            "target generator started "
            f"pico_topic={self.pico_topic} "
            f"flange_topic={self.flange_pose_topic} "
            f"target_topic={self.target_topic} "
            f"control_hz={control_hz:.1f} "
            f"motion_gate={config.motion_gate_enabled} "
            f"filter_tau={config.input_filter_time_constant_sec:.3f}s "
            f"rotation_limits_rx_ry_rz_deg_s={tuple(round(math.degrees(v), 1) for v in config.max_rotation_axis_speeds_rad_s)} "
            f"stop_threshold={config.motion_stop_translation_m * 1000.0:.1f}mm/"
            f"{math.degrees(config.motion_stop_rotation_rad):.1f}deg "
            "target_output=enabled"
        )

    def _parameter_vector3(self, name: str) -> Vector3:
        value = self.get_parameter(name).value
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError(f"parameter {name} must contain exactly 3 numbers")
        values = tuple(float(component) for component in value)
        if len(values) != 3:
            raise ValueError(f"parameter {name} must contain exactly 3 numbers")
        return values  # type: ignore[return-value]

    def on_pico_state(self, msg: PicoTeleopState) -> None:
        now = time.monotonic()
        palm_valid = bool(msg.palm_valid)
        palm_pose = IDENTITY_POSE

        if palm_valid:
            try:
                palm_pose = self._pose_from_ros(msg.palm_pose)
            except ValueError as exc:
                palm_valid = False
                self.get_logger().warn(f"rejecting invalid PICO palm pose: {exc}")

        hand_teleop_enabled = bool(msg.hand_teleop_enabled) and palm_valid

        try:
            self.core.update_pico(
                PicoInput(
                    seq=int(msg.seq),
                    received_monotonic=now,
                    packet_valid=bool(msg.packet_valid),
                    transport_alive=bool(msg.transport_alive),
                    hand_teleop_enabled=hand_teleop_enabled,
                    palm_valid=palm_valid,
                    reset_origin=bool(msg.reset_origin),
                    palm_pose=palm_pose,
                )
            )
        except ValueError as exc:
            self.get_logger().warn(f"rejecting PICO state: {exc}")

    def on_flange_pose(self, msg: PoseStamped) -> None:
        now = time.monotonic()

        if (
            self.require_flange_frame_id_match
            and msg.header.frame_id != self.flange_frame_id
        ):
            self._log_flange_error_throttled(
                f"flange frame mismatch: expected {self.flange_frame_id!r}, "
                f"received {msg.header.frame_id!r}"
            )
            return

        try:
            pose = self._pose_from_ros(msg.pose)
            self.core.update_flange(
                FlangeInput(received_monotonic=now, pose=pose)
            )
        except ValueError as exc:
            self._log_flange_error_throttled(f"invalid flange pose: {exc}")

    def on_timer(self) -> None:
        output = self.core.step(time.monotonic())

        msg = RmTeleopTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.flange_frame_id
        msg.source_seq = output.source_seq
        msg.active = output.active
        msg.target_valid = output.target_valid
        msg.state = output.state
        msg.reason = output.reason
        msg.pico_age_sec = self._finite_age_or_negative(output.pico_age_sec)
        msg.flange_age_sec = self._finite_age_or_negative(output.flange_age_sec)
        self._copy_pose_to_ros(output.raw_pico_pose, msg.raw_pico_pose)
        self._copy_pose_to_ros(output.filtered_pico_pose, msg.filtered_pico_pose)
        msg.input_motion_state = output.input_motion_state
        msg.still_hold_active = output.still_hold_active
        msg.settling_active = output.settling_active
        msg.settling_remaining_translation_m = (
            output.settling_remaining_translation_m
        )
        msg.settling_remaining_rotation_deg = (
            output.settling_remaining_rotation_deg
        )
        msg.motion_window_translation_m = output.motion_window_translation_m
        msg.motion_window_rotation_deg = output.motion_window_rotation_deg
        self._copy_pose_to_ros(output.raw_target_pose, msg.raw_target_pose)
        self._copy_pose_to_ros(output.limited_target_pose, msg.limited_target_pose)
        self.target_pub.publish(msg)

        state_key = (output.state, output.reason, output.active)
        if state_key != self._last_state_key:
            previous = self._last_state_key
            self._last_state_key = state_key
            self.get_logger().info(
                "target state transition "
                f"from={previous} to={state_key} "
                f"seq={output.source_seq} "
                f"pico_age={self._format_age(output.pico_age_sec)} "
                f"flange_age={self._format_age(output.flange_age_sec)}"
            )

    @staticmethod
    def _pose_from_ros(msg: Pose) -> Pose3D:
        return validate_pose(
            Pose3D(
                position=(
                    float(msg.position.x),
                    float(msg.position.y),
                    float(msg.position.z),
                ),
                orientation=(
                    float(msg.orientation.x),
                    float(msg.orientation.y),
                    float(msg.orientation.z),
                    float(msg.orientation.w),
                ),
            )
        )

    @staticmethod
    def _copy_pose_to_ros(source: Pose3D, target: Pose) -> None:
        target.position.x = source.position[0]
        target.position.y = source.position[1]
        target.position.z = source.position[2]
        target.orientation.x = source.orientation[0]
        target.orientation.y = source.orientation[1]
        target.orientation.z = source.orientation[2]
        target.orientation.w = source.orientation[3]

    @staticmethod
    def _finite_age_or_negative(value: float) -> float:
        return value if math.isfinite(value) else -1.0

    @staticmethod
    def _format_age(value: float) -> str:
        return f"{value:.3f}s" if math.isfinite(value) else "unavailable"

    def _log_flange_error_throttled(self, message: str) -> None:
        now = time.monotonic()
        if (
            message != self._last_flange_error
            or now - self._last_flange_error_log_time >= 2.0
        ):
            self._last_flange_error = message
            self._last_flange_error_log_time = now
            self.get_logger().warn(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetGeneratorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
