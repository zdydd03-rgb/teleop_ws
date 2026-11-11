from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

from .o7_single_session_hil import SingleSessionHil, StreamConfig
from .o7_auto_stream_hil import AutoStreamConfig

from pico_rm65_teleop_interfaces.msg import RmCanfdControlState, RmTeleopTarget

from .canfd_control_core import (
    MOTION_CONFIRMATION_TOKEN,
    NOT_ATTEMPTED,
    motion_permission,
    tracking_error,
)
from .canfd_csv_logger import CanfdCsvLogger
from .rm_api2_adapter import Api2AlgorithmError, Api2ConnectionError, Api2UnavailableError
from .rm_api2_motion_adapter import MotionApiResult, RmApi2MotionAdapter
from .rm_feedback_core import FeedbackStore, RmFeedbackSnapshot, compare_poses, snapshot_from_api2_state
from .target_math import IDENTITY_POSE, Pose3D, validate_pose


@dataclass(frozen=True)
class HighFollowTarget:
    source_seq: int
    received_monotonic: float
    active: bool
    target_valid: bool
    upstream_state: str
    upstream_reason: str
    input_motion_state: str
    still_hold_active: bool
    settling_active: bool
    settling_remaining_translation_m: float
    settling_remaining_rotation_deg: float
    motion_window_translation_m: float
    motion_window_rotation_deg: float
    raw_pico_pose: Pose3D
    filtered_pico_pose: Pose3D
    raw_target_pose: Pose3D
    pose: Pose3D


@dataclass(frozen=True)
class ControlSummary:
    state: str = "STARTING"
    reason: str = "not_connected"
    input_active: bool = False
    motion_active: bool = False
    command_sent: bool = False
    fault_latched: bool = False
    max_tracking_error_deg: float = 0.0
    movej_return_code: int = NOT_ATTEMPTED


class Rm65CanfdHighFollowNode(Node):
    """RM65专用高跟随节点。

    控制路径刻意保持简单：
    1. 接收目标法兰位姿；
    2. 使用上一周期成功IK解作为下一周期种子；
    3. 检查有限值、机器人关节物理限位、机器人速度限位、J3硬限位和自碰撞；
    4. 直接以高跟随模式发送Remote IK关节解。

    不包含：相对锚点范围、额外关节步长限幅、额外跟踪误差停机、
    原始IK步长停机、FK误差停机或发送周期停机。

    正常失能/手部丢失/目标暂时无效时仅停止发送，不调用停止API，
    避免控制器进入RM_STOP_E(状态9)。仅严重硬件/反馈/命令错误才快速停止并锁存。
    """

    def __init__(self) -> None:
        super().__init__("rm65_canfd_high_follow_node")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        permission, permission_reason = motion_permission(
            self.allow_motion,
            self.motion_confirmation,
        )
        if self.allow_motion and not permission:
            raise ValueError(
                "allow_motion=true but motion_confirmation is invalid; expected exact token "
                f"{MOTION_CONFIRMATION_TOKEN!r}"
            )
        self.motion_permitted = permission
        self.motion_permission_reason = permission_reason

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        configuration_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.joint_pub = self.create_publisher(JointState, "/rm65/joint_states", sensor_qos)
        self.flange_pub = self.create_publisher(PoseStamped, "/rm65/current_flange_pose", sensor_qos)
        self.waypoint_pub = self.create_publisher(PoseStamped, "/rm65/controller_waypoint_pose", sensor_qos)
        self.feedback_valid_pub = self.create_publisher(Bool, "/rm65/feedback_valid", reliable_qos)
        self.arm_status_pub = self.create_publisher(Int32, "/rm65/arm_current_status", reliable_qos)
        self.status_pub = self.create_publisher(DiagnosticArray, "/rm65/status", reliable_qos)
        self.configuration_pub = self.create_publisher(String, self.configuration_topic, configuration_qos)
        self.control_state_pub = self.create_publisher(
            RmCanfdControlState,
            self.control_state_topic,
            reliable_qos,
        )
        self.target_sub = self.create_subscription(
            RmTeleopTarget,
            self.ik_target_topic,
            self.on_target,
            reliable_qos,
        )

        self.csv_logger = CanfdCsvLogger(self.canfd_csv_log_directory)
        self.store = FeedbackStore(rate_window_size=600)

        self._target_lock = threading.Lock()
        self._target: HighFollowTarget | None = None

        self._adapter_lock = threading.Lock()
        self._adapter: RmApi2MotionAdapter | None = None
        self._connected = False
        self._connection_error = "not_connected"
        self._configuration_json = ""
        self._ik_initialized = False
        self._ik_initialization_error = ""

        self._session_active = False
        self._ik_seed_q_deg: tuple[float, ...] | None = None
        self._last_command_q_deg: tuple[float, ...] | None = None
        self._last_command_send_monotonic: float | None = None
        self._motion_active = False
        self._fault_latched = False
        self._fault_reason = ""
        self._last_stop_return_code = NOT_ATTEMPTED
        self._last_summary = ControlSummary()
        self._last_state_key: tuple[Any, ...] | None = None
        self._last_diagnostic_key: tuple[Any, ...] | None = None

        self._cycles = 0
        self._ik_attempts = 0
        self._ik_successes = 0
        self._ik_no_solution = 0
        self._ik_limited_holds = 0
        self._commands_sent = 0
        self._command_failures = 0
        self._hard_stops = 0
        self._max_tracking_error_seen = 0.0
        self._max_ik_step_seen = 0.0
        self._max_command_duration_ms = 0.0
        self._max_command_interval_sec = 0.0

        self.hil_event_pub = self.create_publisher(String, "/rm65/o7_modbus_hil/event", reliable_qos)
        self._hil = SingleSessionHil(self._emit_hil_event)
        self._hil_service = self.create_service(
            Trigger, "/rm65/o7_modbus_hil/trigger_once", self._trigger_hil_once)
        self._stream_hil_service = self.create_service(
            Trigger, "/rm65/o7_stream_hil/start", self._start_stream_hil)
        self._stop_event = threading.Event()
        self._connection_thread = threading.Thread(
            target=self._connection_worker,
            name="rm65_high_follow_connection",
            daemon=True,
        )
        self._connection_thread.start()
        self.create_timer(1.0 / self.publish_hz, self.on_timer)
        self._auto_hil_last_gate = None
        if self.enable_o7_auto_stream_hil:
            # Admission only; no native SDK call, join or trajectory generation here.
            self._auto_hil_timer = self.create_timer(.05, self._maybe_start_auto_hil)

        csv_path = str(self.csv_logger.path) if self.csv_logger.path is not None else "disabled"
        self.get_logger().warning(
            "RM65 dedicated CANFD high-follow node started "
            f"motion_permitted={self.motion_permitted} reason={self.motion_permission_reason} "
            f"robot={self.robot_ip}:{self.tcp_port} udp={self.udp_target_ip}:{self.udp_port} "
            f"control_hz={self.publish_hz:.1f} follow=True trajectory_mode={self.canfd_trajectory_mode} "
            "command_policy=direct_remote_ik no_custom_step_limit no_tracking_error_stop "
            f"csv={csv_path}"
        )
        if not self.motion_permitted:
            self.get_logger().warning(
                "MOTION OUTPUT IS LOCKED. The node will not call rm_movej_canfd."
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("api2_python_path", "")
        self.declare_parameter("robot_ip", "192.168.125.19")
        self.declare_parameter("tcp_port", 8080)
        self.declare_parameter("udp_target_ip", "192.168.125.100")
        self.declare_parameter("udp_port", 8089)
        self.declare_parameter("udp_period_ms", 5)
        self.declare_parameter("force_coordinate", 0)
        self.declare_parameter("enable_joint_speed", True)
        self.declare_parameter("enable_arm_current_status", True)
        self.declare_parameter("restore_udp_config_on_shutdown", True)

        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("feedback_freshness_timeout_sec", 0.05)
        self.declare_parameter("feedback_disconnect_timeout_sec", 0.20)
        self.declare_parameter("target_freshness_timeout_sec", 0.25)
        self.declare_parameter("reconnect_period_sec", 2.0)

        self.declare_parameter("flange_frame_id", "rm65_base")
        self.declare_parameter("configuration_topic", "/rm65/configuration")
        self.declare_parameter("control_state_topic", "/rm65/canfd_control_state")
        self.declare_parameter("ik_target_topic", "/rm_teleop/flange_target")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )

        self.declare_parameter("ik_dt_sec", 0.005)
        self.declare_parameter("ik_tool_or_work", 1)
        self.declare_parameter("j3_hard_min_deg", 10.0)
        self.declare_parameter("j3_activation_min_deg", 15.0)
        self.declare_parameter("ik_error_weight", [1.0] * 6)
        self.declare_parameter("ik_dq_weight", [1.0] * 6)
        self.declare_parameter("allowed_arm_current_statuses", [0, 5])

        self.declare_parameter("enable_o7_single_session_hil", False)
        self.declare_parameter("enable_o7_stream_hil", False)
        self.declare_parameter("enable_o7_auto_stream_hil", False)
        self.declare_parameter("o7_auto_stream_hz", 10.0)
        self.declare_parameter("o7_auto_stream_timeout_sec", 60.0)
        self.declare_parameter("o7_auto_stream_open_pose_confirmed", False)
        self.declare_parameter("o7_stream_hz", 1.0)
        self.declare_parameter("o7_stream_duration_sec", 10.0)
        self.declare_parameter("o7_stream_joint", 2)
        self.declare_parameter("o7_stream_open_pose_confirmed", False)
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("motion_confirmation", "")
        self.declare_parameter("canfd_trajectory_mode", 0)
        self.declare_parameter("canfd_radio", 0)
        self.declare_parameter("stop_on_shutdown", True)
        self.declare_parameter("canfd_csv_log_directory", "~/teleop_ws/canfd_logs")

    def _read_parameters(self) -> None:
        p = lambda name: self.get_parameter(name).value
        self.api2_python_path = str(p("api2_python_path"))
        self.robot_ip = str(p("robot_ip"))
        self.tcp_port = int(p("tcp_port"))
        self.udp_target_ip = str(p("udp_target_ip"))
        self.udp_port = int(p("udp_port"))
        self.udp_period_ms = int(p("udp_period_ms"))
        self.force_coordinate = int(p("force_coordinate"))
        self.enable_joint_speed = bool(p("enable_joint_speed"))
        self.enable_arm_current_status = bool(p("enable_arm_current_status"))
        self.restore_udp_config_on_shutdown = bool(p("restore_udp_config_on_shutdown"))

        self.publish_hz = float(p("publish_hz"))
        self.feedback_freshness_timeout_sec = float(p("feedback_freshness_timeout_sec"))
        self.feedback_disconnect_timeout_sec = float(p("feedback_disconnect_timeout_sec"))
        self.target_freshness_timeout_sec = float(p("target_freshness_timeout_sec"))
        self.reconnect_period_sec = float(p("reconnect_period_sec"))

        self.flange_frame_id = str(p("flange_frame_id"))
        self.configuration_topic = str(p("configuration_topic"))
        self.control_state_topic = str(p("control_state_topic"))
        self.ik_target_topic = str(p("ik_target_topic"))
        self.joint_names = self._string_list_parameter("joint_names", 6)

        self.ik_dt_sec = float(p("ik_dt_sec"))
        self.ik_tool_or_work = int(p("ik_tool_or_work"))
        self.j3_hard_min_deg = float(p("j3_hard_min_deg"))
        self.j3_activation_min_deg = float(p("j3_activation_min_deg"))
        self.ik_error_weight = self._float_tuple_parameter("ik_error_weight", 6)
        self.ik_dq_weight = self._float_tuple_parameter("ik_dq_weight", 6)
        self.allowed_arm_current_statuses = self._int_tuple_parameter("allowed_arm_current_statuses")

        self.enable_o7_single_session_hil = bool(p("enable_o7_single_session_hil"))
        self.enable_o7_stream_hil = bool(p("enable_o7_stream_hil"))
        self.enable_o7_auto_stream_hil = bool(p("enable_o7_auto_stream_hil"))
        self.o7_auto_stream_config = AutoStreamConfig(
            p("o7_auto_stream_hz"), p("o7_auto_stream_timeout_sec"),
            p("o7_auto_stream_open_pose_confirmed"))
        self.o7_stream_config = StreamConfig(p("o7_stream_hz"), p("o7_stream_duration_sec"),
                                            p("o7_stream_joint"), p("o7_stream_open_pose_confirmed"))
        self.allow_motion = bool(p("allow_motion"))
        self.motion_confirmation = str(p("motion_confirmation"))
        self.canfd_trajectory_mode = int(p("canfd_trajectory_mode"))
        self.canfd_radio = int(p("canfd_radio"))
        self.stop_on_shutdown = bool(p("stop_on_shutdown"))
        self.canfd_csv_log_directory = str(p("canfd_csv_log_directory"))

    def _validate_parameters(self) -> None:
        for name, value in (("tcp_port", self.tcp_port), ("udp_port", self.udp_port)):
            if not 1 <= value <= 65535:
                raise ValueError(f"{name} must be in [1, 65535]")
        if self.udp_period_ms <= 0 or self.udp_period_ms % 5 != 0:
            raise ValueError("udp_period_ms must be a positive multiple of 5")
        for name, value in (
            ("publish_hz", self.publish_hz),
            ("feedback_freshness_timeout_sec", self.feedback_freshness_timeout_sec),
            ("feedback_disconnect_timeout_sec", self.feedback_disconnect_timeout_sec),
            ("target_freshness_timeout_sec", self.target_freshness_timeout_sec),
            ("reconnect_period_sec", self.reconnect_period_sec),
            ("ik_dt_sec", self.ik_dt_sec),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if abs(self.ik_dt_sec - 1.0 / self.publish_hz) > 1e-6:
            raise ValueError("ik_dt_sec must equal 1/publish_hz")
        if 1.0 / self.publish_hz > 0.010 + 1e-12:
            raise ValueError("high-follow CANFD requires publish_hz >= 100 Hz")
        if self.feedback_disconnect_timeout_sec < self.feedback_freshness_timeout_sec:
            raise ValueError("feedback_disconnect_timeout_sec must be >= freshness timeout")
        if self.ik_tool_or_work not in (0, 1):
            raise ValueError("ik_tool_or_work must be 0 or 1")
        if not math.isfinite(self.j3_hard_min_deg):
            raise ValueError("j3_hard_min_deg must be finite")
        if not math.isfinite(self.j3_activation_min_deg):
            raise ValueError("j3_activation_min_deg must be finite")
        if self.j3_activation_min_deg <= self.j3_hard_min_deg:
            raise ValueError("j3_activation_min_deg must be greater than j3_hard_min_deg")
        if self.canfd_trajectory_mode not in (0, 2):
            raise ValueError("high-follow trajectory_mode must be 0 or 2")
        if not 0 <= self.canfd_radio <= 999:
            raise ValueError("canfd_radio must be in [0, 999]")

    def _connection_worker(self) -> None:
        while not self._stop_event.is_set():
            adapter: RmApi2MotionAdapter | None = None
            try:
                adapter = RmApi2MotionAdapter(
                    api2_python_path=self.api2_python_path,
                    robot_ip=self.robot_ip,
                    tcp_port=self.tcp_port,
                    udp_target_ip=self.udp_target_ip,
                    udp_port=self.udp_port,
                    udp_period_ms=self.udp_period_ms,
                    force_coordinate=self.force_coordinate,
                    enable_joint_speed=self.enable_joint_speed,
                    enable_arm_current_status=self.enable_arm_current_status,
                    restore_udp_config_on_shutdown=self.restore_udp_config_on_shutdown,
                    feedback_handler=self._on_api2_feedback,
                )
                configuration = adapter.connect()
                remote_ik_config = adapter.initialize_remote_ik(
                    dt_sec=self.ik_dt_sec,
                    tool_or_work=self.ik_tool_or_work,
                    error_weight=self.ik_error_weight,
                    dq_weight=self.ik_dq_weight,
                    j3_hard_min_deg=self.j3_hard_min_deg,
                )
                configuration.update(
                    {
                        "enable_o7_single_session_hil": self.enable_o7_single_session_hil,
                        "enable_o7_stream_hil": self.enable_o7_stream_hil,
                        "o7_stream_config": vars(self.o7_stream_config),
                        "remote_ik": remote_ik_config,
                        "operation_mode": "canfd_high_follow_realtime",
                        "motion_output_enabled": self.motion_permitted,
                        "motion_permission_reason": self.motion_permission_reason,
                        "canfd": {
                            "follow": True,
                            "trajectory_mode": self.canfd_trajectory_mode,
                            "radio": self.canfd_radio,
                            "control_hz": self.publish_hz,
                            "command_policy": "direct_remote_ik",
                            "normal_disable_policy": "stop_stream_only",
                            "critical_fault_policy": "rm_set_arm_stop_and_latch",
                            "custom_step_limit": False,
                            "tracking_error_stop": False,
                            "anchor_envelope": False,
                            "fk_error_stop": False,
                            "ik_limited_policy": "hold_last_command_and_retry",
                            "target_constraint_policy": "hold_last_command_and_retry",
                        },
                    }
                )
                configuration_json = json.dumps(
                    configuration,
                    ensure_ascii=False,
                    sort_keys=True,
                    default=str,
                )
                with self._adapter_lock:
                    closing = self._stop_event.is_set()
                    if not closing:
                        self._adapter = adapter
                        self._connected = True
                        self._connection_error = ""
                        self._ik_initialized = True
                        self._ik_initialization_error = ""
                        self._configuration_json = configuration_json
                if closing:
                    adapter.disconnect()
                    return
                self.configuration_pub.publish(String(data=configuration_json))
                self.get_logger().warning(
                    "connected to RM65; dedicated high-follow mode initialized; "
                    f"motion_permitted={self.motion_permitted} robot_info={configuration.get('robot_info')}"
                )
                return
            except (Api2UnavailableError, Api2ConnectionError, Api2AlgorithmError, OSError, ValueError) as exc:
                if adapter is not None:
                    try:
                        adapter.disconnect()
                    except Exception:
                        pass
                with self._adapter_lock:
                    self._connected = False
                    self._connection_error = str(exc)
                    self._ik_initialized = False
                    self._ik_initialization_error = str(exc)
                self.get_logger().error(f"RM65 high-follow initialization failed: {exc}")
                self._stop_event.wait(self.reconnect_period_sec)

    def _on_api2_feedback(self, state: Any) -> None:
        try:
            snapshot = snapshot_from_api2_state(state, time.monotonic(), dof=6)
            if snapshot.source_ip and snapshot.source_ip != self.robot_ip:
                raise ValueError(f"unexpected feedback source {snapshot.source_ip}")
            self.store.update(snapshot)
        except (TypeError, ValueError, AttributeError) as exc:
            self.store.reject(str(exc))

    def on_target(self, msg: RmTeleopTarget) -> None:
        target_valid = bool(msg.target_valid)
        pose = IDENTITY_POSE
        raw_target_pose = IDENTITY_POSE
        raw_pico_pose = IDENTITY_POSE
        filtered_pico_pose = IDENTITY_POSE
        try:
            raw_pico_pose = self._pose_from_ros(msg.raw_pico_pose)
            filtered_pico_pose = self._pose_from_ros(msg.filtered_pico_pose)
            raw_target_pose = self._pose_from_ros(msg.raw_target_pose)
            if target_valid:
                pose = self._pose_from_ros(msg.limited_target_pose)
        except ValueError as exc:
            target_valid = False
            self.get_logger().warning(f"rejecting invalid target diagnostics: {exc}")
        target = HighFollowTarget(
            source_seq=int(msg.source_seq),
            received_monotonic=time.monotonic(),
            active=bool(msg.active),
            target_valid=target_valid,
            upstream_state=str(msg.state),
            upstream_reason=str(msg.reason),
            input_motion_state=str(msg.input_motion_state),
            still_hold_active=bool(msg.still_hold_active),
            settling_active=bool(msg.settling_active),
            settling_remaining_translation_m=float(
                msg.settling_remaining_translation_m
            ),
            settling_remaining_rotation_deg=float(
                msg.settling_remaining_rotation_deg
            ),
            motion_window_translation_m=float(msg.motion_window_translation_m),
            motion_window_rotation_deg=float(msg.motion_window_rotation_deg),
            raw_pico_pose=raw_pico_pose,
            filtered_pico_pose=filtered_pico_pose,
            raw_target_pose=raw_target_pose,
            pose=pose,
        )
        with self._target_lock:
            self._target = target

    def on_timer(self) -> None:
        now = time.monotonic()
        snapshot = self.store.latest()
        statistics = self.store.statistics()
        with self._adapter_lock:
            adapter = self._adapter
            connected = self._connected
            connection_error = self._connection_error
            configuration_json = self._configuration_json
            ik_initialized = self._ik_initialized
            ik_initialization_error = self._ik_initialization_error
        if configuration_json:
            self.configuration_pub.publish(String(data=configuration_json))

        feedback_age = max(0.0, now - snapshot.received_monotonic) if snapshot is not None else math.inf
        feedback_valid = connected and snapshot is not None and feedback_age <= self.feedback_freshness_timeout_sec
        self.feedback_valid_pub.publish(Bool(data=feedback_valid))

        fk_pose: Pose3D | None = None
        fk_error = ""
        if adapter is not None and snapshot is not None and feedback_valid:
            try:
                fk_pose = adapter.forward_kinematics(snapshot.joint_position_deg)
            except (Api2ConnectionError, ValueError, TypeError, OSError) as exc:
                fk_error = str(exc)
                feedback_valid = False
                self.feedback_valid_pub.publish(Bool(data=False))

        stamp = self.get_clock().now().to_msg()
        if snapshot is not None:
            self._publish_joint_state(snapshot, stamp)
            self._publish_pose(self.waypoint_pub, snapshot.waypoint, stamp)
            self.arm_status_pub.publish(Int32(data=snapshot.arm_current_status))
        if fk_pose is not None:
            self._publish_pose(self.flange_pub, fk_pose, stamp)

        self._run_control_cycle(
            now=now,
            stamp=stamp,
            adapter=adapter,
            connected=connected,
            ik_initialized=ik_initialized,
            snapshot=snapshot,
        )
        self._publish_diagnostics(
            snapshot=snapshot,
            feedback_age=feedback_age,
            feedback_valid=feedback_valid,
            connected=connected,
            connection_error=connection_error,
            statistics=statistics,
            fk_pose=fk_pose,
            fk_error=fk_error,
            ik_initialized=ik_initialized,
            ik_initialization_error=ik_initialization_error,
        )

    def _run_control_cycle(
        self,
        *,
        now: float,
        stamp: Any,
        adapter: RmApi2MotionAdapter | None,
        connected: bool,
        ik_initialized: bool,
        snapshot: RmFeedbackSnapshot | None,
    ) -> None:
        self._cycles += 1
        with self._target_lock:
            target = self._target

        q_actual = snapshot.joint_position_deg if snapshot is not None else (0.0,) * 6
        q_seed = self._ik_seed_q_deg if self._ik_seed_q_deg is not None else q_actual
        q_ik = (0.0,) * 6
        q_command = self._last_command_q_deg if self._last_command_q_deg is not None else q_actual
        command_step = (0.0,) * 6
        tracking = (0.0,) * 6
        max_tracking = 0.0
        max_ik_step = 0.0
        max_command_step = 0.0

        source_seq = target.source_seq if target is not None else 0
        target_pose = target.pose if target is not None else IDENTITY_POSE
        raw_target_pose = target.raw_target_pose if target is not None else IDENTITY_POSE
        raw_pico_pose = target.raw_pico_pose if target is not None else IDENTITY_POSE
        filtered_pico_pose = (
            target.filtered_pico_pose if target is not None else IDENTITY_POSE
        )
        target_input_motion_state = (
            target.input_motion_state if target is not None else "UNAVAILABLE"
        )
        target_still_hold_active = bool(
            target is not None and target.still_hold_active
        )
        target_settling_active = bool(
            target is not None and target.settling_active
        )
        settling_remaining_translation_m = (
            target.settling_remaining_translation_m if target is not None else 0.0
        )
        settling_remaining_rotation_deg = (
            target.settling_remaining_rotation_deg if target is not None else 0.0
        )
        motion_window_translation_m = (
            target.motion_window_translation_m if target is not None else 0.0
        )
        motion_window_rotation_deg = (
            target.motion_window_rotation_deg if target is not None else 0.0
        )
        target_age = max(0.0, now - target.received_monotonic) if target is not None else math.inf
        feedback_age = max(0.0, now - snapshot.received_monotonic) if snapshot is not None else math.inf
        input_active = bool(target is not None and target.active and target.target_valid)

        ik_attempted = False
        ik_success = False
        ik_output_valid = False
        command_attempted = False
        command_sent = False
        stop_command_sent = False
        ik_return_code = NOT_ATTEMPTED
        ik_position_limit = NOT_ATTEMPTED
        ik_velocity_limit = NOT_ATTEMPTED
        ik_collision = NOT_ATTEMPTED
        command_position_limit = NOT_ATTEMPTED
        command_velocity_limit = NOT_ATTEMPTED
        command_collision = NOT_ATTEMPTED
        movej_return_code = NOT_ATTEMPTED
        stop_return_code = NOT_ATTEMPTED
        ik_duration_ms = 0.0
        command_duration_ms = 0.0
        command_interval_sec = -1.0
        solved_fk = IDENTITY_POSE
        fk_position_error = -1.0
        fk_orientation_error_deg = -1.0
        state = "READY"
        reason = "waiting_for_enable"

        if self._last_command_q_deg is not None:
            track_result = tracking_error(
                actual_deg=q_actual,
                commanded_deg=self._last_command_q_deg,
            )
            tracking = track_result.error_deg
            max_tracking = track_result.max_abs_error_deg
            self._max_tracking_error_seen = max(self._max_tracking_error_seen, max_tracking)
        if self._last_command_send_monotonic is not None:
            command_interval_sec = max(0.0, now - self._last_command_send_monotonic)
            self._max_command_interval_sec = max(
                self._max_command_interval_sec,
                command_interval_sec,
            )

        critical_reason = self._critical_feedback_reason(
            connected=connected,
            ik_initialized=ik_initialized,
            snapshot=snapshot,
            feedback_age=feedback_age,
        )

        if self._fault_latched:
            state = "FAULT_LATCHED"
            reason = self._fault_reason
        elif critical_reason:
            # 启动/等待阶段只报告状态，不锁存。只有已经进入控制会话后
            # 出现反馈、关节或控制器错误，才快速停止并锁存。
            if self._session_active or self._motion_active:
                state, reason, stop_command_sent, stop_return_code = self._critical_fault(
                    adapter,
                    critical_reason,
                )
            else:
                state = "WAITING_ROBOT"
                reason = critical_reason
                self._end_normal_session()
        elif not self.motion_permitted:
            state = "MOTION_LOCKED"
            reason = self.motion_permission_reason
            self._end_normal_session()
        elif target is None:
            state = "WAITING_TARGET"
            reason = "no_target_message"
            self._end_normal_session()
        elif not input_active:
            state = "READY"
            reason = target.upstream_reason or "waiting_for_enable"
            self._end_normal_session()
        elif target_age > self.target_freshness_timeout_sec:
            state = "READY"
            reason = "target_stale"
            self._end_normal_session()
        elif adapter is None or snapshot is None:
            state = "READY"
            reason = "adapter_or_feedback_unavailable"
            self._end_normal_session()
        elif not self._session_active:
            activation_error = self._activation_error(snapshot)
            if activation_error:
                state = "READY"
                reason = activation_error
                self._end_normal_session()
            else:
                self._session_active = True
                self._motion_active = False
                self._ik_seed_q_deg = tuple(q_actual)
                self._last_command_q_deg = None
                self._last_command_send_monotonic = None
                q_seed = tuple(q_actual)
                state = "ACTIVE"
                reason = "activated"

        if (
            not self._fault_latched
            and self.motion_permitted
            and self._session_active
            and input_active
            and target_age <= self.target_freshness_timeout_sec
            and adapter is not None
            and snapshot is not None
        ):
            q_seed = self._ik_seed_q_deg if self._ik_seed_q_deg is not None else q_actual
            ik_attempted = True
            self._ik_attempts += 1
            try:
                result = adapter.solve_remote_ik(target_pose, q_seed)
                ik_return_code = result.return_code
                q_ik = result.q_out_deg
                ik_duration_ms = result.duration_ms
                ik_success = ik_return_code == 0

                if ik_return_code == -1:
                    # 普通无解是可恢复状态。停止发送新关节目标，但保持本次
                    # 遥操作会话和上一周期有效IK种子。用户把手移回可解区域后，
                    # 后续周期会继续求解并自动恢复发送。
                    self._ik_no_solution += 1
                    state = "IK_HOLD"
                    reason = "ik_no_solution_waiting_for_recovery"
                elif ik_return_code == -2:
                    # Remote IK限位通常表示目标接近J3软限位、关节边界或
                    # 奇异区域。它不是通信或硬件故障，不能调用快速停止，
                    # 也不能锁存故障。保留上一条有效命令和IK种子，持续重试；
                    # 用户将手移回安全方向后，IK返回0并自动恢复高跟随。
                    self._ik_limited_holds += 1
                    state = "SINGULARITY_HOLD"
                    reason = "ik_limited_waiting_for_recovery"
                elif ik_return_code != 0:
                    # -4等不可恢复的模型/算法错误仍按严重故障处理。
                    state, reason, stop_command_sent, stop_return_code = self._critical_fault(
                        adapter,
                        f"ik_return_code:{ik_return_code}",
                    )
                elif not all(math.isfinite(value) for value in q_ik):
                    state, reason, stop_command_sent, stop_return_code = self._critical_fault(
                        adapter,
                        "ik_nonfinite",
                    )
                else:
                    self._ik_successes += 1
                    delta = tuple(q_ik[i] - q_seed[i] for i in range(6))
                    max_ik_step = max(abs(value) for value in delta)
                    self._max_ik_step_seen = max(self._max_ik_step_seen, max_ik_step)

                    ik_position_limit = adapter.check_joint_position_limit(q_ik)
                    ik_velocity_limit = adapter.check_joint_velocity_limit(
                        dt_sec=self.ik_dt_sec,
                        q_ref_deg=q_seed,
                        q_out_deg=q_ik,
                    )
                    ik_collision = adapter.check_self_collision(q_ik)

                    if q_ik[2] < self.j3_hard_min_deg:
                        # 目标侧约束均按“保持最后有效命令并等待恢复”处理，
                        # 不调用停止API、不锁存故障。操作者将手移回安全方向后，
                        # 下一次合法IK解会自动恢复发送。
                        state = "SINGULARITY_HOLD"
                        reason = "j3_hard_limit_waiting_for_recovery"
                    elif ik_position_limit != 0:
                        state = "LIMIT_HOLD"
                        reason = f"joint_position_limit_waiting_for_recovery:{ik_position_limit}"
                    elif ik_velocity_limit != 0:
                        state = "LIMIT_HOLD"
                        reason = f"joint_velocity_limit_waiting_for_recovery:{ik_velocity_limit}"
                    elif ik_collision != 0:
                        state = "COLLISION_HOLD"
                        reason = "self_collision_waiting_for_recovery"
                    else:
                        ik_output_valid = True
                        q_command = tuple(q_ik)
                        command_step = delta
                        max_command_step = max_ik_step
                        command_position_limit = ik_position_limit
                        command_velocity_limit = ik_velocity_limit
                        command_collision = ik_collision
                        command_attempted = True
                        motion_result = adapter.send_joint_canfd(
                            q_command,
                            follow=True,
                            trajectory_mode=self.canfd_trajectory_mode,
                            radio=self.canfd_radio,
                        )
                        movej_return_code = motion_result.return_code
                        command_duration_ms = motion_result.duration_ms
                        self._max_command_duration_ms = max(
                            self._max_command_duration_ms,
                            command_duration_ms,
                        )
                        if movej_return_code == 0:
                            command_sent = True
                            self._commands_sent += 1
                            self._ik_seed_q_deg = tuple(q_ik)
                            self._last_command_q_deg = tuple(q_command)
                            self._last_command_send_monotonic = now
                            self._motion_active = True
                            state = "ACTIVE"
                            reason = "command_sent"
                        else:
                            self._command_failures += 1
                            state, reason, stop_command_sent, stop_return_code = self._critical_fault(
                                adapter,
                                f"movej_canfd_failed:{movej_return_code}",
                            )
            except (Api2AlgorithmError, Api2ConnectionError, ValueError, TypeError, OSError) as exc:
                state, reason, stop_command_sent, stop_return_code = self._critical_fault(
                    adapter,
                    f"control_exception:{exc}",
                )

        self._publish_control_state(
            stamp=stamp,
            source_seq=source_seq,
            input_active=input_active,
            target_pose=target_pose,
            raw_target_pose=raw_target_pose,
            raw_pico_pose=raw_pico_pose,
            filtered_pico_pose=filtered_pico_pose,
            target_input_motion_state=target_input_motion_state,
            target_still_hold_active=target_still_hold_active,
            target_settling_active=target_settling_active,
            settling_remaining_translation_m=settling_remaining_translation_m,
            settling_remaining_rotation_deg=settling_remaining_rotation_deg,
            motion_window_translation_m=motion_window_translation_m,
            motion_window_rotation_deg=motion_window_rotation_deg,
            target_age=target_age,
            feedback_age=feedback_age,
            q_actual=q_actual,
            q_ik=q_ik,
            q_command=q_command,
            command_step=command_step,
            tracking=tracking,
            max_ik_step=max_ik_step,
            max_command_step=max_command_step,
            max_tracking=max_tracking,
            ik_attempted=ik_attempted,
            ik_success=ik_success,
            ik_output_valid=ik_output_valid,
            command_attempted=command_attempted,
            command_sent=command_sent,
            stop_command_sent=stop_command_sent,
            state=state,
            reason=reason,
            ik_return_code=ik_return_code,
            ik_position_limit=ik_position_limit,
            ik_velocity_limit=ik_velocity_limit,
            ik_collision=ik_collision,
            command_position_limit=command_position_limit,
            command_velocity_limit=command_velocity_limit,
            command_collision=command_collision,
            movej_return_code=movej_return_code,
            stop_return_code=stop_return_code,
            ik_duration_ms=ik_duration_ms,
            command_duration_ms=command_duration_ms,
            command_interval_sec=command_interval_sec,
            solved_fk=solved_fk,
            fk_position_error=fk_position_error,
            fk_orientation_error_deg=fk_orientation_error_deg,
        )

    def _critical_feedback_reason(
        self,
        *,
        connected: bool,
        ik_initialized: bool,
        snapshot: RmFeedbackSnapshot | None,
        feedback_age: float,
    ) -> str:
        if not connected:
            return "robot_disconnected"
        if not ik_initialized:
            return "ik_not_initialized"
        if snapshot is None:
            return "no_robot_feedback"
        if feedback_age > self.feedback_disconnect_timeout_sec:
            return "robot_feedback_disconnected"
        if feedback_age > self.feedback_freshness_timeout_sec:
            return "robot_feedback_stale"
        if not all(snapshot.joint_enabled):
            return "joint_not_enabled"
        if any(code != 0 for code in snapshot.joint_error_codes):
            return "joint_error"
        if any(code != 0 for code in snapshot.arm_error_codes):
            return "arm_error"
        if snapshot.arm_current_status == 9:
            return "controller_stop_requires_manual_reset"
        if snapshot.arm_current_status not in self.allowed_arm_current_statuses:
            return f"arm_status_not_allowed:{snapshot.arm_current_status}"
        return ""

    def _activation_error(self, snapshot: RmFeedbackSnapshot) -> str:
        if snapshot.arm_current_status == 9:
            return "controller_stop_requires_manual_reset"
        if snapshot.arm_current_status not in self.allowed_arm_current_statuses:
            return f"activation_rejected:arm_status_not_allowed:{snapshot.arm_current_status}"
        if snapshot.joint_position_deg[2] < self.j3_activation_min_deg:
            return "activation_rejected:j3_below_activation_min"
        return ""

    def _end_normal_session(self) -> None:
        self._session_active = False
        self._ik_seed_q_deg = None
        self._last_command_q_deg = None
        self._last_command_send_monotonic = None
        self._motion_active = False

    def _critical_fault(
        self,
        adapter: RmApi2MotionAdapter | None,
        reason: str,
    ) -> tuple[str, str, bool, int]:
        if self._fault_latched:
            return "FAULT_LATCHED", self._fault_reason, False, self._last_stop_return_code
        self._fault_latched = True
        self._fault_reason = reason
        stop_sent = False
        stop_code = NOT_ATTEMPTED
        if adapter is not None and self._motion_active:
            try:
                result: MotionApiResult = adapter.hard_stop()
                stop_sent = True
                stop_code = result.return_code
                self._last_stop_return_code = stop_code
                self._hard_stops += 1
                self.get_logger().error(
                    f"issued quick_stop reason={reason} return_code={stop_code}"
                )
            except Exception as exc:
                stop_sent = True
                stop_code = -998
                self._last_stop_return_code = stop_code
                self.get_logger().error(f"quick_stop failed reason={reason}: {exc}")
        self._end_normal_session()
        self.get_logger().error(f"CANFD high-follow fault latched: {reason}")
        return "FAULT_LATCHED", reason, stop_sent, stop_code

    def _publish_control_state(self, **v: Any) -> None:
        q_actual = v["q_actual"]
        q_ik = v["q_ik"]
        q_command = v["q_command"]
        msg = RmCanfdControlState()
        msg.header.stamp = v["stamp"]
        msg.header.frame_id = self.flange_frame_id
        msg.source_seq = v["source_seq"]
        msg.input_active = v["input_active"]
        msg.ik_initialized = self._ik_initialized
        msg.motion_permitted = self.motion_permitted
        msg.motion_active = self._motion_active
        msg.ik_attempted = v["ik_attempted"]
        msg.ik_success = v["ik_success"]
        msg.ik_output_valid = v["ik_output_valid"]
        msg.command_attempted = v["command_attempted"]
        msg.command_sent = v["command_sent"]
        msg.stop_command_sent = v["stop_command_sent"]
        msg.fault_latched = self._fault_latched
        msg.state = v["state"]
        msg.reason = v["reason"]
        msg.ik_return_code = v["ik_return_code"]
        msg.ik_joint_position_limit_result = v["ik_position_limit"]
        msg.ik_joint_velocity_limit_result = v["ik_velocity_limit"]
        msg.ik_collision_result = v["ik_collision"]
        msg.command_joint_position_limit_result = v["command_position_limit"]
        msg.command_joint_velocity_limit_result = v["command_velocity_limit"]
        msg.command_collision_result = v["command_collision"]
        msg.movej_canfd_return_code = v["movej_return_code"]
        msg.stop_return_code = v["stop_return_code"]
        msg.target_age_sec = self._finite_age_or_negative(v["target_age"])
        msg.feedback_age_sec = self._finite_age_or_negative(v["feedback_age"])
        msg.ik_duration_ms = v["ik_duration_ms"]
        msg.command_duration_ms = v["command_duration_ms"]
        msg.command_interval_sec = v["command_interval_sec"]
        msg.q_actual_deg = list(q_actual)
        msg.q_ik_deg = list(q_ik)
        msg.q_command_deg = list(q_command)
        msg.command_step_deg = list(v["command_step"])
        msg.tracking_error_deg = list(v["tracking"])
        msg.max_ik_step_deg = v["max_ik_step"]
        msg.max_command_step_deg = v["max_command_step"]
        msg.max_tracking_error_deg = v["max_tracking"]
        msg.j3_actual_deg = q_actual[2]
        msg.j3_ik_deg = q_ik[2]
        msg.j3_command_deg = q_command[2]
        msg.ik_fk_position_error_m = v["fk_position_error"]
        msg.ik_fk_orientation_error_deg = v["fk_orientation_error_deg"]
        self._copy_pose(v["target_pose"], msg.target_pose)
        self._copy_pose(v["solved_fk"], msg.ik_solved_fk_pose)
        if (self.enable_o7_single_session_hil or self.enable_o7_stream_hil
                or self.enable_o7_auto_stream_hil):
            self._hil.record(time.monotonic(), msg.state == "ACTIVE" and msg.command_sent
                             and not msg.fault_latched)
        self.control_state_pub.publish(msg)

        self._last_summary = ControlSummary(
            state=v["state"],
            reason=v["reason"],
            input_active=v["input_active"],
            motion_active=self._motion_active,
            command_sent=v["command_sent"],
            fault_latched=self._fault_latched,
            max_tracking_error_deg=v["max_tracking"],
            movej_return_code=v["movej_return_code"],
        )
        state_key = (
            v["state"],
            v["reason"],
            v["input_active"],
            self._motion_active,
            self._fault_latched,
        )
        if state_key != self._last_state_key:
            self._last_state_key = state_key
            self.get_logger().warning(
                "CANFD transition "
                f"state={v['state']} reason={v['reason']} input_active={v['input_active']} "
                f"motion_active={self._motion_active} command_sent={v['command_sent']}"
            )

        self.csv_logger.write(
            source_seq=v["source_seq"],
            state=v["state"],
            reason=v["reason"],
            input_active=v["input_active"],
            motion_permitted=self.motion_permitted,
            motion_active=self._motion_active,
            fault_latched=self._fault_latched,
            ik_return_code=v["ik_return_code"],
            ik_output_valid=v["ik_output_valid"],
            ik_duration_ms=v["ik_duration_ms"],
            ik_position_limit=v["ik_position_limit"],
            ik_velocity_limit=v["ik_velocity_limit"],
            ik_collision=v["ik_collision"],
            command_attempted=v["command_attempted"],
            command_sent=v["command_sent"],
            movej_canfd_return_code=v["movej_return_code"],
            command_duration_ms=v["command_duration_ms"],
            command_interval_sec=v["command_interval_sec"],
            stop_command_sent=v["stop_command_sent"],
            stop_return_code=v["stop_return_code"],
            target_age_sec=self._finite_age_or_negative(v["target_age"]),
            feedback_age_sec=self._finite_age_or_negative(v["feedback_age"]),
            target_input_motion_state=v["target_input_motion_state"],
            target_still_hold_active=v["target_still_hold_active"],
            target_settling_active=v["target_settling_active"],
            settling_remaining_translation_m=v[
                "settling_remaining_translation_m"
            ],
            settling_remaining_rotation_deg=v[
                "settling_remaining_rotation_deg"
            ],
            motion_window_translation_m=v["motion_window_translation_m"],
            motion_window_rotation_deg=v["motion_window_rotation_deg"],
            max_ik_step_deg=v["max_ik_step"],
            max_command_step_deg=v["max_command_step"],
            max_tracking_error_deg=v["max_tracking"],
            j3_actual_deg=q_actual[2],
            j3_ik_deg=q_ik[2],
            j3_command_deg=q_command[2],
            ik_fk_position_error_m=v["fk_position_error"],
            ik_fk_orientation_error_deg=v["fk_orientation_error_deg"],
            q_actual_deg=q_actual,
            q_ik_deg=q_ik,
            q_command_deg=q_command,
            command_step_deg=v["command_step"],
            tracking_error_deg=v["tracking"],
            target_position_m=v["target_pose"].position,
            target_quaternion_xyzw=v["target_pose"].orientation,
            raw_target_position_m=v["raw_target_pose"].position,
            raw_target_quaternion_xyzw=v["raw_target_pose"].orientation,
            raw_pico_position_m=v["raw_pico_pose"].position,
            raw_pico_quaternion_xyzw=v["raw_pico_pose"].orientation,
            filtered_pico_position_m=v["filtered_pico_pose"].position,
            filtered_pico_quaternion_xyzw=v["filtered_pico_pose"].orientation,
        )

    def _publish_diagnostics(
        self,
        *,
        snapshot: RmFeedbackSnapshot | None,
        feedback_age: float,
        feedback_valid: bool,
        connected: bool,
        connection_error: str,
        statistics: Any,
        fk_pose: Pose3D | None,
        fk_error: str,
        ik_initialized: bool,
        ik_initialization_error: str,
    ) -> None:
        level = DiagnosticStatus.OK
        message = "ok"
        if not connected:
            level, message = DiagnosticStatus.ERROR, "disconnected"
        elif snapshot is None:
            level, message = DiagnosticStatus.WARN, "waiting_for_udp_feedback"
        elif feedback_age > self.feedback_disconnect_timeout_sec:
            level, message = DiagnosticStatus.ERROR, "udp_feedback_disconnected"
        elif not feedback_valid:
            level, message = DiagnosticStatus.WARN, "udp_feedback_stale"
        elif fk_error:
            level, message = DiagnosticStatus.ERROR, "fk_invalid"
        elif snapshot is not None and any(code != 0 for code in snapshot.joint_error_codes):
            level, message = DiagnosticStatus.ERROR, "joint_error"
        elif snapshot is not None and any(code != 0 for code in snapshot.arm_error_codes):
            level, message = DiagnosticStatus.ERROR, "arm_error"
        elif snapshot is not None and not all(snapshot.joint_enabled):
            level, message = DiagnosticStatus.ERROR, "joint_not_enabled"
        elif not ik_initialized:
            level, message = DiagnosticStatus.ERROR, "ik_not_initialized"
        elif self._fault_latched:
            level, message = DiagnosticStatus.ERROR, "motion_fault_latched"
        elif not self.motion_permitted:
            level, message = DiagnosticStatus.WARN, "motion_locked"

        comparison = (
            compare_poses(fk_pose, snapshot.waypoint)
            if snapshot is not None and fk_pose is not None
            else None
        )
        values = [
            KeyValue(key="operation_mode", value="canfd_high_follow_realtime"),
            KeyValue(key="motion_output_enabled", value=str(self.motion_permitted).lower()),
            KeyValue(key="motion_permission_reason", value=self.motion_permission_reason),
            KeyValue(key="connected", value=str(connected).lower()),
            KeyValue(key="connection_error", value=connection_error),
            KeyValue(key="feedback_valid", value=str(feedback_valid).lower()),
            KeyValue(
                key="feedback_age_sec",
                value=f"{feedback_age:.6f}" if math.isfinite(feedback_age) else "unavailable",
            ),
            KeyValue(key="feedback_hz", value=f"{statistics.current_hz:.2f}"),
            KeyValue(key="received", value=str(statistics.received)),
            KeyValue(key="rejected", value=str(statistics.rejected)),
            KeyValue(key="last_callback_error", value=statistics.last_error),
            KeyValue(key="ik_initialized", value=str(ik_initialized).lower()),
            KeyValue(key="ik_initialization_error", value=ik_initialization_error),
            KeyValue(key="control_state", value=self._last_summary.state),
            KeyValue(key="control_reason", value=self._last_summary.reason),
            KeyValue(key="motion_active", value=str(self._motion_active).lower()),
            KeyValue(key="fault_latched", value=str(self._fault_latched).lower()),
            KeyValue(key="fault_reason", value=self._fault_reason),
            KeyValue(key="cycles", value=str(self._cycles)),
            KeyValue(key="ik_attempts", value=str(self._ik_attempts)),
            KeyValue(key="ik_successes", value=str(self._ik_successes)),
            KeyValue(key="ik_no_solution", value=str(self._ik_no_solution)),
            KeyValue(key="ik_limited_holds", value=str(self._ik_limited_holds)),
            KeyValue(key="commands_sent", value=str(self._commands_sent)),
            KeyValue(key="command_failures", value=str(self._command_failures)),
            KeyValue(key="hard_stops", value=str(self._hard_stops)),
            KeyValue(key="max_ik_step_seen_deg", value=f"{self._max_ik_step_seen:.6f}"),
            KeyValue(key="max_tracking_error_seen_deg", value=f"{self._max_tracking_error_seen:.6f}"),
            KeyValue(key="max_command_duration_ms", value=f"{self._max_command_duration_ms:.6f}"),
            KeyValue(key="max_command_interval_sec", value=f"{self._max_command_interval_sec:.6f}"),
            KeyValue(key="canfd_follow", value="true"),
            KeyValue(key="canfd_trajectory_mode", value=str(self.canfd_trajectory_mode)),
            KeyValue(key="command_policy", value="direct_remote_ik"),
            KeyValue(key="normal_disable_policy", value="stop_stream_only"),
            KeyValue(key="critical_fault_policy", value="quick_stop_and_latch"),
            KeyValue(key="custom_step_limit", value="false"),
            KeyValue(key="tracking_error_stop", value="false"),
            KeyValue(key="anchor_envelope", value="false"),
            KeyValue(key="fk_error_stop", value="false"),
            KeyValue(key="ik_limited_policy", value="hold_last_command_and_retry"),
            KeyValue(key="target_constraint_policy", value="hold_last_command_and_retry"),
            KeyValue(
                key="csv_log_path",
                value=str(self.csv_logger.path) if self.csv_logger.path else "disabled",
            ),
        ]
        if snapshot is not None:
            values.extend(
                [
                    KeyValue(key="source", value=f"{snapshot.source_ip}:{snapshot.source_port}"),
                    KeyValue(
                        key="joint_enabled",
                        value=",".join(str(v).lower() for v in snapshot.joint_enabled),
                    ),
                    KeyValue(
                        key="joint_error_codes",
                        value=",".join(str(v) for v in snapshot.joint_error_codes),
                    ),
                    KeyValue(
                        key="arm_error_codes",
                        value=",".join(str(v) for v in snapshot.arm_error_codes),
                    ),
                    KeyValue(key="arm_current_status", value=str(snapshot.arm_current_status)),
                    KeyValue(key="j3_deg", value=f"{snapshot.joint_position_deg[2]:.6f}"),
                ]
            )
        if comparison is not None:
            values.extend(
                [
                    KeyValue(
                        key="fk_vs_controller_position_error_m",
                        value=f"{comparison.position_error_m:.9f}",
                    ),
                    KeyValue(
                        key="fk_vs_controller_orientation_error_deg",
                        value=f"{math.degrees(comparison.orientation_error_rad):.9f}",
                    ),
                ]
            )
        status = DiagnosticStatus()
        status.name = "rm65_canfd_high_follow_realtime"
        status.hardware_id = self.robot_ip
        status.level = level
        status.message = message
        status.values = values
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self.status_pub.publish(msg)

        key = (level, message, connected, feedback_valid, self._last_summary.state)
        if key != self._last_diagnostic_key:
            self._last_diagnostic_key = key
            self.get_logger().warning(
                f"diagnostic transition level={level} message={message} "
                f"connected={connected} feedback_valid={feedback_valid}"
            )

    def _publish_joint_state(self, snapshot: RmFeedbackSnapshot, stamp: Any) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.flange_frame_id
        msg.name = self.joint_names
        msg.position = [math.radians(value) for value in snapshot.joint_position_deg]
        msg.velocity = [math.radians(value) for value in snapshot.joint_speed_deg_s]
        self.joint_pub.publish(msg)

    def _publish_pose(self, publisher: Any, pose: Pose3D, stamp: Any) -> None:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.flange_frame_id
        self._copy_pose(pose, msg.pose)
        publisher.publish(msg)

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
    def _copy_pose(source: Pose3D, target: Pose) -> None:
        target.position.x, target.position.y, target.position.z = source.position
        (
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
        ) = source.orientation

    @staticmethod
    def _finite_age_or_negative(value: float) -> float:
        return value if math.isfinite(value) else -1.0

    def _string_list_parameter(self, name: str, expected_length: int) -> list[str]:
        value = self.get_parameter(name).value
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError(f"parameter {name} must be a string list")
        result = [str(item) for item in value]
        if len(result) != expected_length or any(item == "" for item in result):
            raise ValueError(f"parameter {name} must contain {expected_length} names")
        return result

    def _float_tuple_parameter(self, name: str, expected_length: int) -> tuple[float, ...]:
        value = self.get_parameter(name).value
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError(f"parameter {name} must be a numeric list")
        result = tuple(float(item) for item in value)
        if len(result) != expected_length:
            raise ValueError(f"parameter {name} must contain {expected_length} values")
        return result

    def _int_tuple_parameter(self, name: str) -> tuple[int, ...]:
        value = self.get_parameter(name).value
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError(f"parameter {name} must be an integer list")
        result = tuple(int(item) for item in value)
        if not result:
            raise ValueError(f"parameter {name} must not be empty")
        return result

    def _emit_hil_event(self, event: dict) -> None:
        # Wall-clock timestamp is shared with the external monitor; perf_counter measures duration.
        payload = json.dumps(event, ensure_ascii=False)
        self.get_logger().warning(f"O7 HIL {payload}")
        try:
            self.hil_event_pub.publish(String(data=payload))
        except Exception as exc:
            self.get_logger().error(f"HIL event publish failed: {exc}; event={payload}")

    def _hil_rejection(self):
        # Read-only safety snapshot, also polled by the stream worker. No native calls.
        with self._adapter_lock:
            adapter = self._adapter
            connected = self._connected
            initialized = self._ik_initialized
        snapshot = self.store.latest()
        age = time.monotonic() - snapshot.received_monotonic if snapshot else math.inf
        reason = self._critical_feedback_reason(
            connected=connected, ik_initialized=initialized, snapshot=snapshot, feedback_age=age)
        if self._stop_event.is_set():
            reason = "node_closing"
        elif adapter is None or adapter.arm is None or adapter.handle is None:
            reason = "api2_not_connected"
        elif self._fault_latched:
            reason = "FAULT_LATCHED:" + self._fault_reason
        elif not self.motion_permitted:
            reason = "motion_not_permitted"
        elif abs(self.publish_hz - 200.0) > 1e-6:
            reason = "HIL_requires_200Hz_control"
        elif self._last_summary.state != "ACTIVE" or not self._last_summary.command_sent:
            reason = reason or "control_not_ACTIVE_sending"
        return reason

    def _trigger_hil_once(self, request, response):
        # Service and timer share the default mutually exclusive callback group.
        reason = self._hil_rejection()
        response.success, response.message = self._hil.trigger(
            self.enable_o7_single_session_hil, self._adapter, reason)
        return response

    def _start_stream_hil(self, request, response):
        reason = self._hil_rejection()
        response.success, response.message = self._hil.trigger(
            self.enable_o7_stream_hil, self._adapter, reason,
            stream_config=self.o7_stream_config, safety=self._hil_rejection)
        return response

    def _maybe_start_auto_hil(self):
        if not self.enable_o7_auto_stream_hil or self._hil.used or self._stop_event.is_set():
            return
        reason = self._hil_rejection()
        accepted, message = self._hil.trigger(
            True, self._adapter, reason, auto_config=self.o7_auto_stream_config,
            safety=self._hil_rejection)
        gate = (accepted, message)
        if gate != self._auto_hil_last_gate:
            self._auto_hil_last_gate = gate
            self.get_logger().warning(f"O7 auto HIL accepted={accepted}: {message}")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._hil.begin_close()
        with self._adapter_lock:
            adapter = self._adapter
            self._adapter = None
            self._connected = False
            self._ik_initialized = False
        if adapter is not None:
            if self.stop_on_shutdown and self._motion_active:
                try:
                    result = adapter.hard_stop()
                    self.get_logger().error(
                        f"issued quick_stop reason=node_shutdown return_code={result.return_code}"
                    )
                except Exception as exc:
                    self.get_logger().error(f"shutdown quick_stop failed: {exc}")
            self._hil.wait_closed(self.get_logger().error)
            try:
                adapter.disconnect()
            except Exception as exc:
                self.get_logger().error(f"API2 disconnect failed: {exc}")
        self.csv_logger.close()
        if self._connection_thread.is_alive():
            self._connection_thread.join(timeout=2.0)
            if self._connection_thread.is_alive():
                self.get_logger().error("API2 initialization still blocked; waiting for safe cleanup")
                self._connection_thread.join()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Rm65CanfdHighFollowNode()
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
