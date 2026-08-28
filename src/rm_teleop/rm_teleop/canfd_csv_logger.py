from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Iterable

from .target_math import Pose3D


class CanfdCsvLogger:
    FIELDNAMES = [
        "local_time", "source_seq", "state", "reason", "input_active",
        "motion_permitted", "motion_active", "fault_latched",
        "ik_return_code", "ik_output_valid", "ik_duration_ms",
        "ik_position_limit", "ik_velocity_limit", "ik_collision",
        "command_attempted", "command_sent", "movej_canfd_return_code",
        "command_duration_ms", "command_interval_sec", "stop_command_sent",
        "stop_return_code", "target_age_sec", "feedback_age_sec",
        "target_input_motion_state", "target_still_hold_active",
        "target_settling_active", "settling_remaining_translation_m",
        "settling_remaining_rotation_deg",
        "motion_window_translation_m", "motion_window_rotation_deg",
        "max_ik_step_deg", "max_command_step_deg", "max_tracking_error_deg",
        "j3_actual_deg", "j3_ik_deg", "j3_command_deg",
        "ik_fk_position_error_m", "ik_fk_orientation_error_deg",
        "q_actual_deg", "q_ik_deg", "q_command_deg", "command_step_deg",
        "tracking_error_deg", "target_position_m", "target_quaternion_xyzw",
        "raw_target_position_m", "raw_target_quaternion_xyzw",
        "raw_pico_position_m", "raw_pico_quaternion_xyzw",
        "filtered_pico_position_m", "filtered_pico_quaternion_xyzw",
    ]

    def __init__(self, directory: str) -> None:
        self.path: Path | None = None
        self._file = None
        self._writer: csv.DictWriter | None = None
        if directory.strip() == "":
            return
        base = Path(directory).expanduser().resolve()
        base.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = base / f"rm65_canfd_high_follow_{stamp}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()

    def write(self, **values) -> None:
        if self._writer is None or self._file is None:
            return
        row = {name: values.get(name, "") for name in self.FIELDNAMES}
        row["local_time"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
        for name in (
            "q_actual_deg", "q_ik_deg", "q_command_deg", "command_step_deg",
            "tracking_error_deg", "target_position_m", "target_quaternion_xyzw",
            "raw_target_position_m", "raw_target_quaternion_xyzw",
            "raw_pico_position_m", "raw_pico_quaternion_xyzw",
            "filtered_pico_position_m", "filtered_pico_quaternion_xyzw",
        ):
            if name in values:
                row[name] = self._format_vector(values[name])
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None

    @staticmethod
    def _format_vector(values: Iterable[float]) -> str:
        return "[" + ",".join(f"{float(value):.9f}" for value in values) + "]"
