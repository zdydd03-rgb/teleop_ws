"""Workspace/package paths, also usable in the isolated solver interpreter."""
import os
from pathlib import Path


def share_dir():
    override=os.environ.get('PICO_O7_SHARE')
    if override:return Path(override).expanduser().resolve()
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('pico_o7'))
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1]


def workspace_root():
    for ancestor in Path(__file__).resolve().parents:
        if (ancestor/'src/rm_teleop/package.xml').is_file():return ancestor
    from ament_index_python.packages import get_package_prefix
    return Path(get_package_prefix('pico_o7')).parent.parent


def solver_python():
    return workspace_root()/'.o7_solver_venv/bin/python'
