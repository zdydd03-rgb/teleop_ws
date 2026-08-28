from pathlib import Path


MOTION_CALLS = (
    ".rm_movej_canfd(",
    ".rm_set_arm_slow_stop(",
    ".rm_set_arm_stop(",
)


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def test_sdk_motion_calls_are_isolated_to_motion_adapter():
    package_dir = package_root() / "rm_teleop"
    allowed = package_dir / "rm_api2_motion_adapter.py"

    for path in package_dir.glob("*.py"):
        content = path.read_text(encoding="utf-8")
        for call in MOTION_CALLS:
            if path == allowed:
                continue
            assert call not in content, f"SDK motion call {call} leaked into {path}"


def test_motion_adapter_contains_only_expected_motion_calls():
    content = (
        package_root() / "rm_teleop" / "rm_api2_motion_adapter.py"
    ).read_text(encoding="utf-8")

    for call in MOTION_CALLS:
        assert call in content
    for forbidden in (
        ".rm_movej(",
        ".rm_movel(",
        ".rm_movec(",
        ".rm_set_joint_enable(",
    ):
        assert forbidden not in content


def test_normal_disable_stops_stream_without_stop_api():
    source = (
        package_root() / "rm_teleop" / "rm65_canfd_high_follow_node.py"
    ).read_text(encoding="utf-8")

    assert "def _end_normal_session" in source
    assert 'normal_disable_policy", value="stop_stream_only"' in source
    assert "adapter.hard_stop()" in source
