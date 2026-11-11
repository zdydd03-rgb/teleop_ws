from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .canfd_control_core import validate_joint_vector
from .rm_api2_adapter import RmApi2ReadOnlyAdapter


@dataclass(frozen=True)
class MotionApiResult:
    return_code: int
    duration_ms: float


class RmApi2MotionAdapter(RmApi2ReadOnlyAdapter):
    """显式包含运动输出的API2适配层，只供高跟随节点使用。"""

    def connect(self) -> dict:
        configuration = super().connect()
        configuration["read_only"] = False
        configuration["motion_adapter"] = "rm_movej_canfd"
        self.configuration_snapshot = configuration
        return configuration

    def send_joint_canfd(
        self,
        joints_deg: tuple[float, ...],
        *,
        follow: bool,
        trajectory_mode: int,
        radio: int,
    ) -> MotionApiResult:
        joints = validate_joint_vector(joints_deg, "CANFD command joints")
        if trajectory_mode not in (0, 1, 2):
            raise ValueError("trajectory_mode must be 0, 1 or 2")
        if not 0 <= int(radio) <= 999:
            raise ValueError("radio must be in [0, 999]")

        arm = self._require_arm()
        started = time.perf_counter()
        return_code = int(
            arm.rm_movej_canfd(
                list(joints),
                bool(follow),
                0.0,
                int(trajectory_mode),
                int(radio),
            )
        )
        return MotionApiResult(
            return_code=return_code,
            duration_ms=(time.perf_counter() - started) * 1000.0,
        )

    def write_o7_hil_speed_once(self, emit) -> None:
        """Real persistent speed configuration write; caller owns the lifetime lease."""
        start_ns = time.time_ns()
        emit({"event": "MODBUS_START", "timestamp_ns": start_ns,
              "session": "existing_rm65_api2_handle", "operation": "write_o7_speed",
              "slave_id": 39, "address": 14, "num_registers": 7, "port": 1})
        started = time.perf_counter()
        try:
            arm = self._require_arm()
            params = self.module.rm_peripheral_read_write_params_t(
                port=1, address=14, device=39, num=7,
            )
            code = int(arm.rm_write_registers(params, [0, 25] * 7))
        except Exception as exc:
            ended = time.perf_counter()
            emit({"event": "MODBUS_EXCEPTION", "timestamp_ns": time.time_ns(),
                  "error": repr(exc), "modbus_start_timestamp": started,
                  "modbus_end_timestamp": ended, "duration_ms": (ended - started) * 1000,
                  "modbus_duration_ms": (ended - started) * 1000,
                  "return_code": None, "modbus_return_code": None})
        else:
            ended = time.perf_counter()
            emit({"event": "MODBUS_FINISH", "timestamp_ns": time.time_ns(),
                  "modbus_start_timestamp": started, "modbus_end_timestamp": ended,
                  "duration_ms": (ended - started) * 1000,
                  "modbus_duration_ms": (ended - started) * 1000,
                  "return_code": code, "modbus_return_code": code})

    def initialize_o7_modbus(self):
        """Existing handle only. Caller must own the inactive control window."""
        arm = self._require_arm()
        started = time.perf_counter()
        code = int(arm.rm_set_modbus_mode(1, 115200, 1))
        config = arm.rm_get_tool_rs485_mode() if code == 0 else None
        okay = (code == 0 and isinstance(config, dict) and config.get('return_code') == 0
                and all(config.get(k) == v for k, v in
                        (('mode', 1), ('baudrate', 115200), ('timeout', 1))))
        return dict(success=okay, return_code=code, config=config,
                    duration_ms=(time.perf_counter() - started) * 1000)

    def write_o7_hil_positions(self, positions) -> int:
        if (len(positions) != 7
                or any(type(v) is not int or not 0 <= v <= 255 for v in positions)):
            raise ValueError("O7 positions require seven integers in [0,255]")
        arm = self._require_arm()
        params = self.module.rm_peripheral_read_write_params_t(port=1, address=0, device=39, num=7)
        data = [byte for value in positions for byte in (value >> 8, value & 255)]
        return int(arm.rm_write_registers(params, data))

    def slow_stop(self) -> MotionApiResult:
        arm = self._require_arm()
        started = time.perf_counter()
        return_code = int(arm.rm_set_arm_slow_stop())
        return MotionApiResult(
            return_code=return_code,
            duration_ms=(time.perf_counter() - started) * 1000.0,
        )

    def hard_stop(self) -> MotionApiResult:
        arm = self._require_arm()
        started = time.perf_counter()
        return_code = int(arm.rm_set_arm_stop())
        return MotionApiResult(
            return_code=return_code,
            duration_ms=(time.perf_counter() - started) * 1000.0,
        )
