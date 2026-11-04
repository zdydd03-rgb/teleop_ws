#!/usr/bin/env python3
"""Create the workspace-local solver environment; never imports ROS/robot SDK."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python',help='Python 3.11 executable for a fresh environment')
    parser.add_argument('--copy-installed-from',type=Path,help='Offline seed from an existing compatible venv; copied, never linked')
    parser.add_argument('--install',action='store_true',help='Explicitly install the pinned requirements using pip')
    args=parser.parse_args()
    target=ROOT/'.o7_solver_venv'
    if target.exists():parser.error(str(target)+' already exists; refusing to overwrite')
    env=dict(os.environ);env.pop('PYTHONPATH',None);env.pop('PYTHONHOME',None)
    python=args.python
    if args.copy_installed_from:
        python=python or subprocess.check_output([str(args.copy_installed_from/'bin/python'),'-c','import sys;print(sys._base_executable)'],env=env,text=True).strip()
    python=python or shutil.which('python3.11')
    if not python:parser.error('Specify --python /path/to/python3.11')
    version=subprocess.check_output([python,'-c','import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")'],env=env,text=True).strip()
    if version!='3.11':parser.error('The current pinned dependencies require Python 3.11')
    subprocess.run([python,'-m','venv','--copies',str(target)],env=env,check=True)
    (target/'COLCON_IGNORE').touch()
    if args.copy_installed_from:
        source=args.copy_installed_from/'lib/python3.11/site-packages'
        shutil.copytree(source,target/'lib/python3.11/site-packages',dirs_exist_ok=True)
    if args.install:
        subprocess.run([str(target/'bin/python'),'-m','pip','install','-r',str(ROOT/'src/pico_o7/requirements.lock')],env=env,check=True)
    subprocess.run([str(target/'bin/python'),'-m','pip','check'],env=env,check=True)
    (target/'workspace_environment.json').write_text(json.dumps(dict(
        python_base=python,seed=str(args.copy_installed_from) if args.copy_installed_from else None,
        runtime_dependency_on_seed=False),indent=2))
    print('Solver interpreter:',target/'bin/python')


if __name__=='__main__':main()
