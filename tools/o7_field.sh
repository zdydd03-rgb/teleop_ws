#!/usr/bin/env bash
# All project code/configs now live in this workspace. No extra SDK clients.
set -e
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-help}
if [ "$#" -gt 0 ]; then shift; fi
case "$mode" in
  hand-start|combined-start|hand-status|dry-run|tests) ;;
  help) printf '%s\n' 'Modes: hand-start, combined-start, hand-status, dry-run, tests'; exit 0 ;;
  *) printf '%s\n' "Unknown mode: $mode" >&2; exit 2 ;;
esac
cd "$workspace"
if [ "$mode" = dry-run ]; then
  exec env -u PYTHONPATH -u PYTHONHOME PYTHONPATH="$workspace/src/pico_o7" \
    "$workspace/.o7_solver_venv/bin/python" -m pico_o7.retarget_dry_run "$@"
fi
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
export ROS_LOG_DIR="$workspace/log/pico_o7"
case "$mode" in
  hand-start) exec ros2 launch pico_o7 teleop.launch.py allow_motion:=false enable_o7_retargeting:=true "$@" ;;
  combined-start) exec ros2 launch pico_o7 teleop.launch.py allow_motion:=true motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED enable_o7_retargeting:=true "$@" ;;
  hand-status) exec ros2 run pico_o7 o7_session_tools hand-status "$@" ;;
  tests) exec python3 -m pytest src/rm_teleop/test src/pico_o7/test tools -q "$@" ;;
esac
