#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from pico_rm65_teleop_interfaces.msg import (
    PicoTeleopState,
    RmIkDryRunState,
)


FIELDS = [
    "wall_time",
    "elapsed_sec",
    "source_seq",
    "input_active",
    "ik_initialized",
    "ik_attempted",
    "ik_success",
    "output_valid",
    "fault_latched",
    "motion_command_sent",
    "state",
    "reason",
    "ik_return_code",
    "joint_position_limit_result",
    "joint_velocity_limit_result",
    "collision_result",
    "target_age_sec",
    "feedback_age_sec",
    "ik_duration_ms",
    "q_in_deg",
    "q_out_deg",
    "delta_q_deg",
    "max_abs_delta_q_deg",
    "j3_in_deg",
    "j3_out_deg",
    "fk_position_error_m",
    "fk_orientation_error_deg",
]


def join_values(values) -> str:
    return ";".join(f"{float(value):.9f}" for value in values)


def all_finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class AcceptanceNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65_ik_acceptance_test")

        self.args = args
        self.phase = "WAIT_FALSE"

        self.start_time = time.monotonic()
        self.rising_edge_time: Optional[float] = None
        self.record_start_time: Optional[float] = None

        self.last_pico_receive_time: Optional[float] = None
        self.last_ik_receive_time: Optional[float] = None
        self.last_ik_message: Optional[RmIkDryRunState] = None

        self.rows: List[Dict[str, object]] = []

        self.finish_reason = ""
        self.done = False
        self.exit_code = 2

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            PicoTeleopState,
            "/pico/teleop_state",
            self.on_pico_state,
            qos,
        )

        self.create_subscription(
            RmIkDryRunState,
            "/rm65/ik_dry_run_state",
            self.on_ik_state,
            qos,
        )

        self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            "自动空跑验收已启动。先保持手柄连接。"
        )
        self.get_logger().info(
            "脚本观察到 hand_teleop_enabled=false 后，"
            "请把右手放稳，然后断开手柄。"
        )
        self.get_logger().info(
            f"Remote IK激活后自动记录 {args.duration:.1f} 秒，"
            "测试期间不需要操作终端。"
        )

    def on_pico_state(self, msg: PicoTeleopState) -> None:
        now = time.monotonic()
        self.last_pico_receive_time = now

        enabled = bool(msg.hand_teleop_enabled)

        if self.phase == "WAIT_FALSE":
            if not enabled:
                self.phase = "WAIT_TRUE"
                self.get_logger().info(
                    "已观察到 hand_teleop_enabled=false。"
                )
                self.get_logger().info(
                    "现在把右手放稳，然后断开手柄。"
                )
            return

        if self.phase == "WAIT_TRUE":
            if enabled:
                self.phase = "WAIT_IK_ACTIVE"
                self.rising_edge_time = now
                self.get_logger().info(
                    "检测到使能上升沿，等待Remote IK开始计算。"
                )
            return

        if self.phase == "WAIT_IK_ACTIVE":
            if not enabled:
                self.phase = "WAIT_TRUE"
                self.rising_edge_time = None
                self.get_logger().warning(
                    "IK尚未激活时使能已经消失，"
                    "重新等待下一次上升沿。"
                )
            return

        if self.phase == "RECORDING" and not enabled:
            self.finish(
                "pico_disabled_before_window_completed"
            )

    def on_ik_state(self, msg: RmIkDryRunState) -> None:
        now = time.monotonic()

        self.last_ik_receive_time = now
        self.last_ik_message = msg

        if self.phase == "WAIT_IK_ACTIVE":
            if msg.input_active and msg.ik_attempted:
                self.phase = "RECORDING"
                self.record_start_time = now
                self.rows.clear()

                self.get_logger().info(
                    "Remote IK已开始计算。"
                )
                self.get_logger().info(
                    f"正在自动记录 {self.args.duration:.1f} 秒，"
                    "请保持右手稳定，不要操作终端。"
                )

                self.append_row(msg, now)
                self.check_immediate_failure(msg)
            return

        if self.phase == "RECORDING":
            self.append_row(msg, now)
            self.check_immediate_failure(msg)

    def append_row(
        self,
        msg: RmIkDryRunState,
        now: float,
    ) -> None:
        if self.record_start_time is None:
            return

        self.rows.append(
            {
                "wall_time": datetime.now().isoformat(
                    timespec="milliseconds"
                ),
                "elapsed_sec": (
                    now - self.record_start_time
                ),
                "source_seq": int(msg.source_seq),
                "input_active": bool(msg.input_active),
                "ik_initialized": bool(msg.ik_initialized),
                "ik_attempted": bool(msg.ik_attempted),
                "ik_success": bool(msg.ik_success),
                "output_valid": bool(msg.output_valid),
                "fault_latched": bool(msg.fault_latched),
                "motion_command_sent": bool(
                    msg.motion_command_sent
                ),
                "state": str(msg.state),
                "reason": str(msg.reason),
                "ik_return_code": int(
                    msg.ik_return_code
                ),
                "joint_position_limit_result": int(
                    msg.joint_position_limit_result
                ),
                "joint_velocity_limit_result": int(
                    msg.joint_velocity_limit_result
                ),
                "collision_result": int(
                    msg.collision_result
                ),
                "target_age_sec": float(
                    msg.target_age_sec
                ),
                "feedback_age_sec": float(
                    msg.feedback_age_sec
                ),
                "ik_duration_ms": float(
                    msg.ik_duration_ms
                ),
                "q_in_deg": join_values(
                    msg.q_in_deg
                ),
                "q_out_deg": join_values(
                    msg.q_out_deg
                ),
                "delta_q_deg": join_values(
                    msg.delta_q_deg
                ),
                "max_abs_delta_q_deg": float(
                    msg.max_abs_delta_q_deg
                ),
                "j3_in_deg": float(
                    msg.j3_in_deg
                ),
                "j3_out_deg": float(
                    msg.j3_out_deg
                ),
                "fk_position_error_m": float(
                    msg.fk_position_error_m
                ),
                "fk_orientation_error_deg": float(
                    msg.fk_orientation_error_deg
                ),
                "_receive_time": now,
                "_joint_arrays": (
                    list(msg.q_in_deg)
                    + list(msg.q_out_deg)
                    + list(msg.delta_q_deg)
                ),
            }
        )

    def check_immediate_failure(
        self,
        msg: RmIkDryRunState,
    ) -> None:
        if self.phase != "RECORDING":
            return

        if msg.motion_command_sent:
            self.finish(
                "motion_command_sent_became_true"
            )
            return

        if msg.fault_latched:
            self.finish(
                f"fault_latched:{msg.reason}"
            )
            return

        if not msg.ik_initialized:
            self.finish(
                "ik_initialized_became_false"
            )
            return

        if not msg.input_active:
            self.finish(
                f"ik_input_became_inactive:{msg.reason}"
            )
            return

        # 普通IK无解允许少量出现，最终按比例统计。
        # 其他原因均视为安全拒绝并立即结束。
        if msg.reason not in {"ok", "ik_failed"}:
            self.finish(
                f"safety_rejection:{msg.reason}"
            )
            return

        joint_values = (
            list(msg.q_in_deg)
            + list(msg.q_out_deg)
            + list(msg.delta_q_deg)
        )

        if not all_finite(joint_values):
            self.finish(
                "nonfinite_joint_value"
            )

    def on_timer(self) -> None:
        if self.done:
            return

        now = time.monotonic()

        if self.phase in {
            "WAIT_FALSE",
            "WAIT_TRUE",
            "WAIT_IK_ACTIVE",
        }:
            if (
                now - self.start_time
                > self.args.activation_timeout
            ):
                self.finish("activation_timeout")
                return

        if self.phase == "WAIT_FALSE":
            if (
                self.last_pico_receive_time is None
                and now - self.start_time > 5.0
            ):
                self.finish("no_pico_messages")
            return

        if self.phase == "WAIT_IK_ACTIVE":
            if self.rising_edge_time is not None:
                if (
                    now - self.rising_edge_time
                    > self.args.ik_active_timeout
                ):
                    detail = ""

                    if self.last_ik_message is not None:
                        detail = (
                            f":{self.last_ik_message.state}"
                            f":{self.last_ik_message.reason}"
                        )

                    self.finish(
                        "ik_did_not_enter_active"
                        + detail
                    )
            return

        if self.phase != "RECORDING":
            return

        if (
            self.last_pico_receive_time is None
            or now - self.last_pico_receive_time > 0.5
        ):
            self.finish("pico_message_timeout")
            return

        if (
            self.last_ik_receive_time is None
            or now - self.last_ik_receive_time > 0.2
        ):
            self.finish("ik_state_message_timeout")
            return

        assert self.record_start_time is not None

        if (
            now - self.record_start_time
            >= self.args.duration
        ):
            self.finish("window_completed")

    def finish(self, reason: str) -> None:
        if self.done:
            return

        self.finish_reason = reason
        self.done = True

        self.write_report()

        if rclpy.ok():
            rclpy.shutdown()

    def write_report(self) -> None:
        output_path = Path(
            self.args.output
        ).expanduser().resolve()

        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        with output_path.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as file:
            writer = csv.DictWriter(
                file,
                fieldnames=FIELDS,
            )
            writer.writeheader()

            for row in self.rows:
                writer.writerow(
                    {
                        field: row.get(field, "")
                        for field in FIELDS
                    }
                )

        rows = self.rows
        sample_count = len(rows)

        receive_times = [
            float(row["_receive_time"])
            for row in rows
        ]

        elapsed = (
            float(rows[-1]["elapsed_sec"])
            if rows
            else 0.0
        )

        if (
            len(receive_times) > 1
            and receive_times[-1] > receive_times[0]
        ):
            measured_rate = (
                len(receive_times) - 1
            ) / (
                receive_times[-1]
                - receive_times[0]
            )
        else:
            measured_rate = 0.0

        attempts = sum(
            bool(row["ik_attempted"])
            for row in rows
        )

        successes = sum(
            bool(row["ik_success"])
            for row in rows
        )

        valid_outputs = sum(
            bool(row["output_valid"])
            for row in rows
        )

        success_ratio = (
            successes / attempts
            if attempts
            else 0.0
        )

        valid_ratio = (
            valid_outputs / sample_count
            if sample_count
            else 0.0
        )

        max_joint_step = max(
            (
                float(row["max_abs_delta_q_deg"])
                for row in rows
            ),
            default=math.inf,
        )

        min_j3_out = min(
            (
                float(row["j3_out_deg"])
                for row in rows
                if row["ik_success"]
            ),
            default=-math.inf,
        )

        max_target_age = max(
            (
                float(row["target_age_sec"])
                for row in rows
            ),
            default=math.inf,
        )

        max_feedback_age = max(
            (
                float(row["feedback_age_sec"])
                for row in rows
            ),
            default=math.inf,
        )

        max_ik_duration = max(
            (
                float(row["ik_duration_ms"])
                for row in rows
                if row["ik_attempted"]
            ),
            default=math.inf,
        )

        max_fk_position_error = max(
            (
                float(row["fk_position_error_m"])
                for row in rows
                if row["output_valid"]
            ),
            default=math.inf,
        )

        max_fk_orientation_error = max(
            (
                float(
                    row["fk_orientation_error_deg"]
                )
                for row in rows
                if row["output_valid"]
            ),
            default=math.inf,
        )

        critical_reasons = [
            str(row["reason"])
            for row in rows
            if row["reason"]
            not in {"ok", "ik_failed"}
        ]

        ordinary_failures = sum(
            row["reason"] == "ik_failed"
            for row in rows
        )

        bad_position_limits = sum(
            bool(row["ik_success"])
            and int(
                row["joint_position_limit_result"]
            )
            != 0
            for row in rows
        )

        bad_velocity_limits = sum(
            bool(row["ik_success"])
            and int(
                row["joint_velocity_limit_result"]
            )
            != 0
            for row in rows
        )

        bad_collisions = sum(
            bool(row["ik_success"])
            and int(row["collision_result"]) != 0
            for row in rows
        )

        all_arrays_finite = all(
            all_finite(row["_joint_arrays"])
            for row in rows
        )

        minimum_samples = max(
            20,
            int(
                self.args.duration
                * self.args.min_rate_hz
                * 0.95
            ),
        )

        checks = [
            (
                "record_window_completed",
                self.finish_reason
                == "window_completed",
            ),
            (
                "sample_count",
                sample_count >= minimum_samples,
            ),
            (
                "message_rate_hz",
                measured_rate
                >= self.args.min_rate_hz,
            ),
            (
                "success_ratio",
                success_ratio
                >= self.args.min_success_ratio,
            ),
            (
                "valid_ratio",
                valid_ratio
                >= self.args.min_valid_ratio,
            ),
            (
                "motion_command_sent",
                not any(
                    row["motion_command_sent"]
                    for row in rows
                ),
            ),
            (
                "fault_latched",
                not any(
                    row["fault_latched"]
                    for row in rows
                ),
            ),
            (
                "critical_reasons",
                not critical_reasons,
            ),
            (
                "ordinary_failures",
                ordinary_failures
                <= self.args.max_ordinary_failures,
            ),
            (
                "max_joint_step",
                max_joint_step
                <= self.args.max_joint_step_deg,
            ),
            (
                "j3_out_min",
                min_j3_out
                > self.args.j3_hard_min_deg,
            ),
            (
                "position_limit",
                bad_position_limits == 0,
            ),
            (
                "velocity_limit",
                bad_velocity_limits == 0,
            ),
            (
                "collision",
                bad_collisions == 0,
            ),
            (
                "all_finite",
                all_arrays_finite,
            ),
            (
                "target_age",
                max_target_age
                <= self.args.max_target_age,
            ),
            (
                "feedback_age",
                max_feedback_age
                <= self.args.max_feedback_age,
            ),
            (
                "ik_duration",
                max_ik_duration
                <= self.args.max_ik_duration_ms,
            ),
            (
                "fk_position_error",
                max_fk_position_error
                <= self.args.max_fk_position_error,
            ),
            (
                "fk_orientation_error",
                max_fk_orientation_error
                <= self.args.max_fk_orientation_error,
            ),
        ]

        passed = all(
            check_passed
            for _, check_passed in checks
        )

        print()
        print("=" * 72)
        print("RM65 Remote IK 自动空跑验收")
        print("=" * 72)

        print(
            f"finish_reason={self.finish_reason}"
        )

        print(
            f"samples={sample_count}, "
            f"elapsed={elapsed:.3f}s, "
            f"rate={measured_rate:.2f}Hz"
        )

        print(
            f"attempts={attempts}, "
            f"successes={successes}, "
            f"valid={valid_outputs}"
        )

        print(
            f"success_ratio={success_ratio:.4f}, "
            f"valid_ratio={valid_ratio:.4f}"
        )

        print(
            f"max_step={max_joint_step:.6f}deg, "
            f"min_j3_out={min_j3_out:.6f}deg"
        )

        print(
            f"max_target_age="
            f"{max_target_age:.6f}s, "
            f"max_feedback_age="
            f"{max_feedback_age:.6f}s"
        )

        print(
            f"max_ik_duration="
            f"{max_ik_duration:.6f}ms"
        )

        print(
            f"max_fk_position_error="
            f"{max_fk_position_error:.9f}m"
        )

        print(
            f"max_fk_orientation_error="
            f"{max_fk_orientation_error:.9f}deg"
        )

        if critical_reasons:
            print(
                "critical_reasons="
                + ",".join(critical_reasons[:10])
            )

        for name, check_passed in checks:
            result = (
                "PASS"
                if check_passed
                else "FAIL"
            )
            print(f"[{result}] {name}")

        print(f"CSV={output_path}")
        print(
            f"RESULT="
            f"{'PASS' if passed else 'FAIL'}"
        )
        print("=" * 72)

        self.exit_code = 0 if passed else 1


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "RM65 Remote IK自动空跑验收脚本"
        )
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--activation-timeout",
        type=float,
        default=90.0,
    )

    parser.add_argument(
        "--ik-active-timeout",
        type=float,
        default=3.0,
    )

    parser.add_argument(
        "--min-rate-hz",
        type=float,
        default=80.0,
    )

    parser.add_argument(
        "--min-success-ratio",
        type=float,
        default=0.99,
    )

    parser.add_argument(
        "--min-valid-ratio",
        type=float,
        default=0.99,
    )

    parser.add_argument(
        "--max-ordinary-failures",
        type=int,
        default=2,
    )

    parser.add_argument(
        "--max-joint-step-deg",
        type=float,
        default=1.0,
    )

    parser.add_argument(
        "--j3-hard-min-deg",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--max-target-age",
        type=float,
        default=0.10,
    )

    parser.add_argument(
        "--max-feedback-age",
        type=float,
        default=0.05,
    )

    parser.add_argument(
        "--max-ik-duration-ms",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--max-fk-position-error",
        type=float,
        default=0.001,
    )

    parser.add_argument(
        "--max-fk-orientation-error",
        type=float,
        default=0.2,
    )

    parser.add_argument(
        "--output",
        default=(
            "ik_acceptance_"
            + datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )
            + ".csv"
        ),
    )

    return parser


def main() -> int:
    args = create_parser().parse_args()

    rclpy.init()

    node = AcceptanceNode(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("user_interrupt")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return node.exit_code


if __name__ == "__main__":
    raise SystemExit(main())