#!/usr/bin/env python3
"""Stop only processes carrying an explicitly named test Gazebo partition."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import os,signal,time,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('partition');p.add_argument('--exclude-pid',action='append',type=int,default=[]);a=p.parse_args()
if not a.partition.startswith('usv_'):raise SystemExit('Expected a project test partition')
key=('GZ_PARTITION='+a.partition).encode();pids=[]
for f in Path('/proc').glob('[0-9]*'):
 try:
  if int(f.name) not in [os.getpid(),os.getppid(),*a.exclude_pid] and f.stat().st_uid==os.getuid() and key in (f/'environ').read_bytes().split(b'\0'):pids.append(int(f.name))
 except OSError:pass
for sig in [signal.SIGTERM,signal.SIGKILL]:
 for pid in pids:
  try:
   if key in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0'):os.kill(pid,sig)
  except (ProcessLookupError,FileNotFoundError):pass
 if sig==signal.SIGTERM:time.sleep(1)
print('Stopped',len(pids),'processes in',a.partition)
