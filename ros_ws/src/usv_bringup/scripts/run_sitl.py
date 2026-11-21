#!/usr/bin/env python3
"""Start the existing local ArduRover binary in an isolated workspace directory."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,os
from pathlib import Path
root=WORKSPACE
p=argparse.ArgumentParser();p.add_argument('--instance',type=int,default=10);p.add_argument('--binary',type=Path,default=Path(os.environ.get('ARDUPILOT_BIN',str(root.parents[1]/'ardupilot/build/sitl/bin/ardurover'))));p.add_argument('--wipe',action='store_true');p.add_argument('--profile',choices=['baseline','platform15'],default='baseline');a=p.parse_args()
if not 0<=a.instance<=99:p.error('Instance must be in [0,99]')
if not a.binary.is_file():p.error('ArduRover binary not found; pass --binary or ARDUPILOT_BIN')
work=root/'log/ardupilot'/a.profile/f'instance_{a.instance}';work.mkdir(parents=True,exist_ok=True)
args=[str(a.binary.resolve()),'--model','JSON','--sim-address','127.0.0.1','--sim-port-out',str(9002+10*a.instance),'--instance',str(a.instance),'--home','-33.724223,150.679736,0,90','--defaults',str(root/'src/usv_ardupilot/config/boat.parm')+(','+str(root/'src/usv_ardupilot/config/platform15.parm') if a.profile=='platform15' else ''),'--speedup','1','--rate','100']
if a.wipe:args.append('--wipe')
print(f'JSON bridge port {9002+10*a.instance}; MAVLink TCP port {5760+10*a.instance}',flush=True)
os.chdir(work);os.execv(args[0],args)
