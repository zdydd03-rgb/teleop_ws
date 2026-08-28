from pathlib import Path


def root() -> Path:
    return Path(__file__).resolve().parents[1]


def test_only_one_launch_and_one_config_exist():
    launch_files = sorted((root() / "launch").glob("*.launch.py"))
    config_files = sorted((root() / "config").glob("*.yaml"))

    assert [path.name for path in launch_files] == ["teleop_high_follow.launch.py"]
    assert [path.name for path in config_files] == ["high_follow.yaml"]


def test_only_high_follow_console_path_is_installed():
    setup_text = (root() / "setup.py").read_text(encoding="utf-8")

    assert "rm65_canfd_high_follow_node" in setup_text
    assert "rm65_canfd_slow_node" not in setup_text
    assert "rm65_state_node" not in setup_text


def test_launch_exposes_speed_controls_and_defaults_locked():
    launch_text = (
        root() / "launch" / "teleop_high_follow.launch.py"
    ).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("allow_motion", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("position_scale", default_value="0.25")' in launch_text
    assert 'DeclareLaunchArgument("translation_speed_mps", default_value="0.05")' in launch_text
    assert 'DeclareLaunchArgument("rotation_speed_deg_s", default_value="12.0")' in launch_text
    assert 'DeclareLaunchArgument("motion_gate_enabled", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("input_filter_time_constant_sec", default_value="0.060")' in launch_text
    assert 'executable="rm65_canfd_high_follow_node"' in launch_text


def test_high_follow_configuration_is_200_hz_and_motion_locked():
    config = (root() / "config" / "high_follow.yaml").read_text(encoding="utf-8")

    assert "control_hz: 200.0" in config
    assert "publish_hz: 200.0" in config
    assert "ik_dt_sec: 0.005" in config
    assert "allow_motion: false" in config
    assert 'motion_confirmation: ""' in config
    assert "max_translation_speed_mps: 0.05" in config
    assert "max_rotation_speed_deg_s: 12.0" in config
    assert "motion_gate_enabled: true" in config
    assert "input_filter_time_constant_sec: 0.060" in config
    assert "motion_stop_translation_m: 0.0035" in config
    assert "motion_stop_rotation_deg: 1.5" in config


def test_direct_remote_ik_canfd_stream_has_no_slow_path_imports():
    source = (
        root() / "rm_teleop" / "rm65_canfd_high_follow_node.py"
    ).read_text(encoding="utf-8")

    assert "q_command = tuple(q_ik)" in source
    assert "follow=True" in source
    assert "rm65_canfd_slow" not in source
    assert "ik_dry_run" not in source


def test_target_interface_and_csv_expose_input_stabilization_diagnostics():
    target_msg = (
        root().parents[0]
        / "pico_rm65_teleop_interfaces"
        / "msg"
        / "RmTeleopTarget.msg"
    ).read_text(encoding="utf-8")
    csv_source = (
        root() / "rm_teleop" / "canfd_csv_logger.py"
    ).read_text(encoding="utf-8")

    for field in (
        "raw_pico_pose",
        "filtered_pico_pose",
        "input_motion_state",
        "still_hold_active",
        "settling_active",
        "settling_remaining_translation_m",
        "settling_remaining_rotation_deg",
        "motion_window_translation_m",
        "motion_window_rotation_deg",
    ):
        assert field in target_msg

    for field in (
        "target_input_motion_state",
        "target_still_hold_active",
        "target_settling_active",
        "settling_remaining_translation_m",
        "settling_remaining_rotation_deg",
        "raw_target_position_m",
        "raw_pico_position_m",
        "filtered_pico_position_m",
    ):
        assert field in csv_source
