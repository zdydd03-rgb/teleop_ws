# Workspace-owned PICO + O7 overlay; the original ARM control implementation is unchanged.
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("rm_teleop"))
    config_file = package_share / "config" / "high_follow.yaml"
    o7_share = Path(get_package_share_directory("pico_o7"))
    from pico_o7.paths import solver_python

    allow_motion = LaunchConfiguration("allow_motion")
    motion_confirmation = LaunchConfiguration("motion_confirmation")
    position_scale = LaunchConfiguration("position_scale")
    translation_speed = LaunchConfiguration("translation_speed_mps")
    rotation_speed = LaunchConfiguration("rotation_speed_deg_s")
    motion_gate_enabled = LaunchConfiguration("motion_gate_enabled")
    input_filter_tau = LaunchConfiguration("input_filter_time_constant_sec")
    motion_window = LaunchConfiguration("motion_detection_window_sec")
    motion_start_translation = LaunchConfiguration("motion_start_translation_m")
    motion_start_rotation = LaunchConfiguration("motion_start_rotation_deg")
    motion_stop_translation = LaunchConfiguration("motion_stop_translation_m")
    motion_stop_rotation = LaunchConfiguration("motion_stop_rotation_deg")
    motion_stop_hold = LaunchConfiguration("motion_stop_hold_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("o7_solver_python", default_value=str(solver_python())),
            DeclareLaunchArgument("enable_o7_read_diag", default_value="false"),
            DeclareLaunchArgument("enable_o7_retargeting", default_value="false"),
            DeclareLaunchArgument("o7_device_mapping_path", default_value=str(o7_share / "config/o7_device_linear.json")),
            DeclareLaunchArgument("enable_o7_index2_calibration", default_value="false"),
            DeclareLaunchArgument("o7_index2_plan_path", default_value=str(o7_share / "calibration/index2_plan.example.json")),
            DeclareLaunchArgument("enable_o7_single_session_hil", default_value="false"),
            DeclareLaunchArgument("enable_o7_stream_hil", default_value="false"),
            DeclareLaunchArgument("enable_o7_auto_stream_hil", default_value="false"),
            DeclareLaunchArgument("o7_auto_stream_hz", default_value="10.0"),
            DeclareLaunchArgument("o7_auto_stream_timeout_sec", default_value="60.0"),
            DeclareLaunchArgument("o7_auto_stream_open_pose_confirmed", default_value="false"),
            DeclareLaunchArgument("o7_stream_hz", default_value="1.0"),
            DeclareLaunchArgument("o7_stream_duration_sec", default_value="10.0"),
            DeclareLaunchArgument("o7_stream_joint", default_value="2"),
            DeclareLaunchArgument("o7_stream_open_pose_confirmed", default_value="false"),
            DeclareLaunchArgument("allow_motion", default_value="false"),
            DeclareLaunchArgument("motion_confirmation", default_value=""),
            DeclareLaunchArgument("position_scale", default_value="0.75"),
            DeclareLaunchArgument("translation_speed_mps", default_value="0.08"),
            DeclareLaunchArgument("rotation_speed_deg_s", default_value="12.0"),
            DeclareLaunchArgument("motion_gate_enabled", default_value="true"),
            DeclareLaunchArgument("input_filter_time_constant_sec", default_value="0.060"),
            DeclareLaunchArgument("motion_detection_window_sec", default_value="0.40"),
            DeclareLaunchArgument("motion_start_translation_m", default_value="0.004"),
            DeclareLaunchArgument("motion_start_rotation_deg", default_value="2.0"),
            DeclareLaunchArgument("motion_stop_translation_m", default_value="0.0035"),
            DeclareLaunchArgument("motion_stop_rotation_deg", default_value="1.5"),
            DeclareLaunchArgument("motion_stop_hold_sec", default_value="0.30"),
            Node(
                package="pico_o7",
                executable="pico_hand_receiver_node",
                name="pico_receiver_node",
                output="screen",
                parameters=[str(config_file)],
            ),
            Node(
                package="pico_o7",
                executable="shared_session_node",
                name="rm65_canfd_high_follow_node",
                output="screen",
                parameters=[
                    str(config_file),
                    {
                        "enable_o7_read_diag": ParameterValue(LaunchConfiguration("enable_o7_read_diag"), value_type=bool),
                        "enable_o7_retargeting": ParameterValue(LaunchConfiguration("enable_o7_retargeting"), value_type=bool),
                        "o7_device_mapping_path": LaunchConfiguration("o7_device_mapping_path"),
                        "o7_solver_python": LaunchConfiguration("o7_solver_python"),
                        "enable_o7_index2_calibration": ParameterValue(LaunchConfiguration("enable_o7_index2_calibration"), value_type=bool),
                        "o7_index2_plan_path": LaunchConfiguration("o7_index2_plan_path"),
                        "allow_motion": ParameterValue(
                            allow_motion,
                            value_type=bool,
                        ),
                        "motion_confirmation": motion_confirmation,
                        **{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                           for name, kind in (("enable_o7_auto_stream_hil", bool),
                                              ("o7_auto_stream_hz", float),
                                              ("o7_auto_stream_timeout_sec", float),
                                              ("o7_auto_stream_open_pose_confirmed", bool),
                                              ("enable_o7_stream_hil", bool),
                                              ("o7_stream_hz", float),
                                              ("o7_stream_duration_sec", float),
                                              ("o7_stream_joint", int),
                                              ("o7_stream_open_pose_confirmed", bool))},
                        "enable_o7_single_session_hil": ParameterValue(
                            LaunchConfiguration("enable_o7_single_session_hil"), value_type=bool),
                    },
                ],
            ),
            Node(
                package="rm_teleop",
                executable="target_generator_node",
                name="target_generator_node",
                output="screen",
                parameters=[
                    str(config_file),
                    {
                        "position_scale": ParameterValue(
                            position_scale,
                            value_type=float,
                        ),
                        "max_translation_speed_mps": ParameterValue(
                            translation_speed,
                            value_type=float,
                        ),
                        "max_rotation_speed_deg_s": ParameterValue(
                            rotation_speed,
                            value_type=float,
                        ),
                        "motion_gate_enabled": ParameterValue(
                            motion_gate_enabled,
                            value_type=bool,
                        ),
                        "input_filter_time_constant_sec": ParameterValue(
                            input_filter_tau,
                            value_type=float,
                        ),
                        "motion_detection_window_sec": ParameterValue(
                            motion_window,
                            value_type=float,
                        ),
                        "motion_start_translation_m": ParameterValue(
                            motion_start_translation,
                            value_type=float,
                        ),
                        "motion_start_rotation_deg": ParameterValue(
                            motion_start_rotation,
                            value_type=float,
                        ),
                        "motion_stop_translation_m": ParameterValue(
                            motion_stop_translation,
                            value_type=float,
                        ),
                        "motion_stop_rotation_deg": ParameterValue(
                            motion_stop_rotation,
                            value_type=float,
                        ),
                        "motion_stop_hold_sec": ParameterValue(
                            motion_stop_hold,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
