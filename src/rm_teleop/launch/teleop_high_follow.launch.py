from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("rm_teleop"))
    config_file = package_share / "config" / "high_follow.yaml"

    allow_motion = LaunchConfiguration("allow_motion")
    motion_confirmation = LaunchConfiguration("motion_confirmation")
    position_scale = LaunchConfiguration("position_scale")
    translation_speed = LaunchConfiguration("translation_speed_mps")
    rotation_speed = LaunchConfiguration("rotation_speed_deg_s")
    rotation_fallback = IfElseSubstitution(EqualsSubstitution(rotation_speed, ""), "20.0", rotation_speed)
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
            DeclareLaunchArgument("allow_motion", default_value="true"),
            DeclareLaunchArgument("motion_confirmation", default_value=""),
            DeclareLaunchArgument("position_scale", default_value="0.75"),
            DeclareLaunchArgument("translation_speed_mps", default_value="0.08"),
            DeclareLaunchArgument("rotation_speed_deg_s", default_value=""),
            *[DeclareLaunchArgument(f"rotation_{axis}_speed_deg_s", default_value=IfElseSubstitution(
                EqualsSubstitution(rotation_speed, ""), default, rotation_speed))
              for axis, default in (("rx", "70.0"), ("ry", "8.0"), ("rz", "8.0"))],
            DeclareLaunchArgument("motion_gate_enabled", default_value="true"),
            DeclareLaunchArgument("input_filter_time_constant_sec", default_value="0.060"),
            DeclareLaunchArgument("motion_detection_window_sec", default_value="0.40"),
            DeclareLaunchArgument("motion_start_translation_m", default_value="0.004"),
            DeclareLaunchArgument("motion_start_rotation_deg", default_value="2.0"),
            DeclareLaunchArgument("motion_stop_translation_m", default_value="0.0035"),
            DeclareLaunchArgument("motion_stop_rotation_deg", default_value="1.5"),
            DeclareLaunchArgument("motion_stop_hold_sec", default_value="0.30"),
            Node(
                package="rm_teleop",
                executable="pico_receiver_node",
                name="pico_receiver_node",
                output="screen",
                parameters=[str(config_file)],
            ),
            Node(
                package="rm_teleop",
                executable="rm65_canfd_high_follow_node",
                name="rm65_canfd_high_follow_node",
                output="screen",
                parameters=[
                    str(config_file),
                    {
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
                            rotation_fallback,
                            value_type=float,
                        ),
                        **{f"max_rotation_{axis}_speed_deg_s": ParameterValue(
                            LaunchConfiguration(f"rotation_{axis}_speed_deg_s"), value_type=float)
                           for axis in ("rx", "ry", "rz")},
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
