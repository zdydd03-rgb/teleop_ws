from __future__ import annotations

import ctypes
import importlib
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from .rm_feedback_core import pose_from_fk_result
from .target_math import Pose3D, quaternion_to_matrix, validate_pose


class Api2UnavailableError(RuntimeError):
    pass


class Api2ConnectionError(RuntimeError):
    pass


class Api2AlgorithmError(RuntimeError):
    pass


@dataclass(frozen=True)
class RemoteIkResult:
    return_code: int
    q_out_deg: tuple[float, ...]
    duration_ms: float


def _candidate_python_paths(explicit_path: str) -> list[Path]:
    candidates: list[Path] = []

    def add(value: str | Path | None) -> None:
        if value is None or str(value).strip() == "":
            return

        path = Path(value).expanduser().resolve()

        if path not in candidates:
            candidates.append(path)

    add(explicit_path)
    add(os.environ.get("RM_API2_PYTHON_PATH"))

    home = Path.home()
    add(home / "RM_API2" / "Python")
    add(home / "RM_API2-1.1.5" / "Python")
    add(home / "RealMan" / "RM_API2" / "Python")

    return candidates


def load_api2_module(explicit_path: str = "") -> Any:
    import_error: Exception | None = None

    try:
        return importlib.import_module("Robotic_Arm.rm_robot_interface")
    except (ImportError, OSError) as exc:
        import_error = exc

    checked: list[str] = []

    for path in _candidate_python_paths(explicit_path):
        checked.append(str(path))

        if not (path / "Robotic_Arm" / "rm_robot_interface.py").is_file():
            continue

        path_text = str(path)

        if path_text not in sys.path:
            sys.path.insert(0, path_text)

        try:
            return importlib.import_module("Robotic_Arm.rm_robot_interface")
        except (ImportError, OSError) as exc:
            import_error = exc

    details = ", ".join(checked) if checked else "no candidate paths"
    raise Api2UnavailableError(
        "unable to import RealMan API2 Python package; "
        f"checked: {details}; last error: {import_error}"
    )


class RmApi2ReadOnlyAdapter:
    """
    API2连接、读取、算法FK和Remote IK空跑适配层。

    本类不暴露任何机械臂运动、停止、使能或清错方法。
    """

    def __init__(
        self,
        *,
        api2_python_path: str,
        robot_ip: str,
        tcp_port: int,
        udp_target_ip: str,
        udp_port: int,
        udp_period_ms: int,
        force_coordinate: int,
        enable_joint_speed: bool,
        enable_arm_current_status: bool,
        restore_udp_config_on_shutdown: bool,
        feedback_handler: Callable[[Any], None],
    ) -> None:
        if udp_period_ms <= 0 or udp_period_ms % 5 != 0:
            raise ValueError("udp_period_ms must be a positive multiple of 5")

        self.module = load_api2_module(api2_python_path)
        self.robot_ip = robot_ip
        self.tcp_port = int(tcp_port)
        self.udp_target_ip = udp_target_ip
        self.udp_port = int(udp_port)
        self.udp_period_ms = int(udp_period_ms)
        self.force_coordinate = int(force_coordinate)
        self.enable_joint_speed = bool(enable_joint_speed)
        self.enable_arm_current_status = bool(enable_arm_current_status)
        self.restore_udp_config_on_shutdown = bool(restore_udp_config_on_shutdown)
        self.feedback_handler = feedback_handler

        self.arm: Any | None = None
        self.handle: Any | None = None
        self._callback: Any | None = None
        self._previous_udp_config: dict[str, Any] | None = None
        self.robot_info: dict[str, Any] = {}
        self.configuration_snapshot: dict[str, Any] = {}
        self.remote_ik_initialized = False

    def connect(self) -> dict[str, Any]:
        if self.arm is not None:
            return self.configuration_snapshot

        self.arm = self.module.RoboticArm(
            self.module.rm_thread_mode_e.RM_TRIPLE_MODE_E
        )
        self.handle = self.arm.rm_create_robot_arm(
            self.robot_ip,
            self.tcp_port,
            level=3,
        )

        if int(getattr(self.handle, "id", -1)) < 0:
            self.arm = None
            self.handle = None
            raise Api2ConnectionError(
                f"failed to connect to RM65 at {self.robot_ip}:{self.tcp_port}"
            )

        info_ret, info = self.arm.rm_get_robot_info()

        if info_ret != 0:
            self.disconnect()
            raise Api2ConnectionError(f"rm_get_robot_info failed: {info_ret}")

        self.robot_info = dict(info)

        if int(info.get("arm_dof", 0)) != 6 or info.get("arm_model") != "RM_65":
            self.disconnect()
            raise Api2ConnectionError(
                "connected robot is not an RM65 with 6 DOF: " f"{info}"
            )

        callback_type = self.module.rm_realtime_arm_state_callback_ptr
        self._callback = callback_type(self.feedback_handler)
        self.arm.rm_realtime_arm_state_call_back(self._callback)

        previous_ret, previous_config = self.arm.rm_get_realtime_push()

        if previous_ret == 0:
            self._previous_udp_config = dict(previous_config)

        custom = self.module.rm_udp_custom_config_t(
            joint_speed=1 if self.enable_joint_speed else 0,
            lift_state=0,
            expand_state=0,
            arm_current_status=1 if self.enable_arm_current_status else 0,
            hand_state=0,
            aloha_state=0,
            plus_base=0,
            plus_state=0,
        )

        push_config = self.module.rm_realtime_push_config_t(
            cycle=self.udp_period_ms // 5,
            enable=True,
            port=self.udp_port,
            force_coordinate=self.force_coordinate,
            ip=self.udp_target_ip,
            custom_config=custom,
        )

        set_ret = self.arm.rm_set_realtime_push(push_config)

        if set_ret != 0:
            self.disconnect()
            raise Api2ConnectionError(f"rm_set_realtime_push failed: {set_ret}")

        active_push_ret, active_push = self.arm.rm_get_realtime_push()
        current_state_ret, current_state = self.arm.rm_get_current_arm_state()
        install_pose = self.arm.rm_get_install_pose()
        tool_ret, controller_tool = self.arm.rm_get_current_tool_frame()
        work_ret, controller_work = self.arm.rm_get_current_work_frame()
        algo_tool_before = self.arm.rm_algo_get_curr_toolframe()
        algo_work_before = self.arm.rm_algo_get_curr_workframe()

        try:
            algo_install_angle = tuple(float(value) for value in self.arm.rm_algo_get_angle())
        except Exception as exc:
            algo_install_angle = (f"unavailable:{exc}",)

        identity_tool = self.module.rm_frame_t(
            frame_name="flange",
            pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            payload=0.0,
            x=0.0,
            y=0.0,
            z=0.0,
        )
        identity_work = self.module.rm_frame_t(
            frame_name="base",
            pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            payload=0.0,
            x=0.0,
            y=0.0,
            z=0.0,
        )

        # 仅修改SDK进程内算法坐标系，不修改控制器当前工具/工作坐标系。
        self.arm.rm_algo_set_toolframe(identity_tool)
        self.arm.rm_algo_set_workframe(identity_work)

        algo_tool_after = self.arm.rm_algo_get_curr_toolframe()
        algo_work_after = self.arm.rm_algo_get_curr_workframe()

        self.configuration_snapshot = {
            "read_only": True,
            "motion_output_enabled": False,
            "robot_ip": self.robot_ip,
            "tcp_port": self.tcp_port,
            "handle_id": int(self.handle.id),
            "robot_info": self.robot_info,
            "udp_requested": {
                "period_ms": self.udp_period_ms,
                "cycle_multiple": self.udp_period_ms // 5,
                "target_ip": self.udp_target_ip,
                "port": self.udp_port,
                "force_coordinate": self.force_coordinate,
                "joint_speed": self.enable_joint_speed,
                "arm_current_status": self.enable_arm_current_status,
            },
            "udp_previous": self._previous_udp_config,
            "udp_active_return_code": active_push_ret,
            "udp_active": active_push,
            "current_state_return_code": current_state_ret,
            "current_state": current_state,
            "controller_install_pose": install_pose,
            "algorithm_install_angle": algo_install_angle,
            "controller_tool_return_code": tool_ret,
            "controller_tool": controller_tool,
            "controller_work_return_code": work_ret,
            "controller_work": controller_work,
            "algorithm_tool_before": algo_tool_before,
            "algorithm_work_before": algo_work_before,
            "algorithm_tool_after": algo_tool_after,
            "algorithm_work_after": algo_work_after,
        }

        return self.configuration_snapshot

    def initialize_remote_ik(
        self,
        *,
        dt_sec: float,
        tool_or_work: int,
        error_weight: tuple[float, ...],
        dq_weight: tuple[float, ...],
        j3_hard_min_deg: float,
    ) -> dict[str, Any]:
        arm = self._require_arm()

        if not math.isfinite(dt_sec) or dt_sec <= 0.0:
            raise ValueError("Remote IK dt_sec must be finite and greater than zero")

        if tool_or_work not in (0, 1):
            raise ValueError("tool_or_work must be 0 or 1")

        if len(error_weight) != 6 or len(dq_weight) != 6:
            raise ValueError("Remote IK weights must each contain exactly 6 values")

        for name, values in (("error_weight", error_weight), ("dq_weight", dq_weight)):
            if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in values):
                raise ValueError(f"{name} values must be finite and in [0, 1]")

        if not math.isfinite(j3_hard_min_deg):
            raise ValueError("j3_hard_min_deg must be finite")

        limits_before = {
            "joint_min_deg": tuple(float(v) for v in arm.rm_algo_get_joint_min_limit()),
            "joint_max_deg": tuple(float(v) for v in arm.rm_algo_get_joint_max_limit()),
            "joint_max_speed_rpm": tuple(float(v) for v in arm.rm_algo_get_joint_max_speed()),
        }

        arm.rm_algo_ik_remote_init(float(dt_sec), int(tool_or_work))
        arm.rm_algo_set_error_weight(list(error_weight))
        arm.rm_algo_set_dq_weight(list(dq_weight))
        arm.rm_algo_set_enable_limit_holdon(0)

        limit_ret = arm.rm_algo_set_joint_limit_angle(
            self.module.rm_dofType_e.DOF_TYPE_6,
            self.module.rm_jointType_e.JOINT_Q3,
            self.module.rm_limitType_e.LIMIT_MIN,
            float(j3_hard_min_deg),
        )

        if limit_ret != 0:
            raise Api2AlgorithmError(
                f"rm_algo_set_joint_limit_angle failed: {limit_ret}"
            )

        limits_after = {
            "joint_min_deg": tuple(float(v) for v in arm.rm_algo_get_joint_min_limit()),
            "joint_max_deg": tuple(float(v) for v in arm.rm_algo_get_joint_max_limit()),
            "joint_max_speed_rpm": tuple(float(v) for v in arm.rm_algo_get_joint_max_speed()),
        }

        self.remote_ik_initialized = True
        remote_ik_config = {
            "initialized": True,
            "dt_sec": float(dt_sec),
            "tool_or_work": int(tool_or_work),
            "error_weight": tuple(error_weight),
            "dq_weight": tuple(dq_weight),
            "limit_holdon": False,
            "j3_hard_min_deg": float(j3_hard_min_deg),
            "j3_limit_set_return_code": int(limit_ret),
            "algorithm_limits_before": limits_before,
            "algorithm_limits_after": limits_after,
        }
        self.configuration_snapshot["remote_ik"] = remote_ik_config
        return remote_ik_config

    def solve_remote_ik(
        self,
        target_pose: Pose3D,
        q_in_deg: tuple[float, ...],
    ) -> RemoteIkResult:
        arm = self._require_arm()

        if not self.remote_ik_initialized:
            raise Api2AlgorithmError("Remote IK has not been initialized")

        if len(q_in_deg) != 6 or not all(math.isfinite(value) for value in q_in_deg):
            raise ValueError("Remote IK q_in must contain 6 finite joint angles")

        matrix = self._pose_to_rm_matrix(target_pose)
        q_out_array = (ctypes.c_float * 6)()

        started = time.perf_counter()
        return_code = int(
            arm.rm_algo_ik_remote(
                matrix,
                list(q_in_deg),
                q_out_array,
            )
        )
        duration_ms = (time.perf_counter() - started) * 1000.0

        q_out = tuple(float(q_out_array[index]) for index in range(6))
        return RemoteIkResult(
            return_code=return_code,
            q_out_deg=q_out,
            duration_ms=duration_ms,
        )

    def check_joint_position_limit(self, q_deg: tuple[float, ...]) -> int:
        self._validate_joint_vector(q_deg)
        values = (ctypes.c_float * 8)(*q_deg, 0.0, 0.0)
        return int(self.module.rm_algo_ikine_check_joint_position_limit(values))

    def check_joint_velocity_limit(
        self,
        *,
        dt_sec: float,
        q_ref_deg: tuple[float, ...],
        q_out_deg: tuple[float, ...],
    ) -> int:
        self._validate_joint_vector(q_ref_deg)
        self._validate_joint_vector(q_out_deg)

        if not math.isfinite(dt_sec) or dt_sec <= 0.0:
            raise ValueError("velocity limit dt_sec must be finite and greater than zero")

        q_ref = (ctypes.c_float * 8)(*q_ref_deg, 0.0, 0.0)
        q_out = (ctypes.c_float * 8)(*q_out_deg, 0.0, 0.0)
        return int(
            self.module.rm_algo_ikine_check_joint_velocity_limit(
                ctypes.c_float(dt_sec),
                q_ref,
                q_out,
            )
        )

    def check_self_collision(self, q_deg: tuple[float, ...]) -> int:
        arm = self._require_arm()
        self._validate_joint_vector(q_deg)
        return int(arm.rm_algo_safety_robot_self_collision_detection(list(q_deg)))

    def forward_kinematics(self, joints_deg: tuple[float, ...]) -> Pose3D:
        arm = self._require_arm()
        self._validate_joint_vector(joints_deg)
        result = arm.rm_algo_forward_kinematics(list(joints_deg), flag=0)
        return pose_from_fk_result(result)

    def disconnect(self) -> None:
        arm = self.arm

        if arm is None:
            return

        if self.restore_udp_config_on_shutdown and self._previous_udp_config is not None:
            try:
                restored = self._push_config_from_dict(self._previous_udp_config)
                arm.rm_set_realtime_push(restored)
            except Exception:
                pass

        try:
            arm.rm_delete_robot_arm()
        finally:
            self.arm = None
            self.handle = None
            self._callback = None
            self.remote_ik_initialized = False

    def _pose_to_rm_matrix(self, pose: Pose3D) -> Any:
        pose = validate_pose(pose)
        rotation = quaternion_to_matrix(pose.orientation)
        data = [
            [rotation[0][0], rotation[0][1], rotation[0][2], pose.position[0]],
            [rotation[1][0], rotation[1][1], rotation[1][2], pose.position[1]],
            [rotation[2][0], rotation[2][1], rotation[2][2], pose.position[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]
        return self.module.rm_Mat_t(row=4, col=4, data=data)

    @staticmethod
    def _validate_joint_vector(values: tuple[float, ...]) -> None:
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("RM65 joint vector must contain exactly 6 finite values")

    def _require_arm(self) -> Any:
        if self.arm is None:
            raise Api2ConnectionError("robot is not connected")
        return self.arm

    def _push_config_from_dict(self, value: dict[str, Any]) -> Any:
        custom_value = value.get("custom_config", {})

        if not isinstance(custom_value, dict):
            custom_value = {}

        custom = self.module.rm_udp_custom_config_t(
            joint_speed=int(custom_value.get("joint_speed", -1)),
            lift_state=int(custom_value.get("lift_state", -1)),
            expand_state=int(custom_value.get("expand_state", -1)),
            arm_current_status=int(custom_value.get("arm_current_status", -1)),
            hand_state=int(custom_value.get("hand_state", -1)),
            aloha_state=int(custom_value.get("aloha_state", -1)),
            plus_base=int(custom_value.get("plus_base", -1)),
            plus_state=int(custom_value.get("plus_state", -1)),
        )

        return self.module.rm_realtime_push_config_t(
            cycle=int(value.get("cycle", 1)),
            enable=bool(value.get("enable", False)),
            port=int(value.get("port", self.udp_port)),
            force_coordinate=int(value.get("force_coordinate", self.force_coordinate)),
            ip=str(value.get("ip", self.udp_target_ip)),
            custom_config=custom,
        )
