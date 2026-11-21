#!/usr/bin/env python3
"""Smoke-test the real coupled launcher, including bounded child cleanup."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import os,time,json,signal,subprocess,argparse
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);parser.add_argument('--kill-controller',action='store_true');parser.add_argument('--control',choices=['ros','ardupilot'],default='ros');args=parser.parse_args();stem='launcher_smoke' if args.control=='ros' else 'launcher_ardupilot_smoke'
if args.kill_controller:stem='launcher_controller_exit'
w=WORKSPACE;out=args.output or w/'log/platform_validation';out.mkdir(parents=True,exist_ok=True)
os.environ.update(GZ_PARTITION='usv_performance_launcher_test',GZ_IP='127.0.0.1',ROS_DOMAIN_ID='90')
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
node=Node();rows=[];node.subscribe(Double_V,'/usv/propulsion/state',lambda m:rows.append(list(m.data)))
started_epoch=time.time()
f=(out/(stem+'.log')).open('w');p=subprocess.Popen(['bash',str(tool_path('run_platform.sh')),'--control',args.control,'--wave-gain','0','headless:=True'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children=set();start=time.monotonic();injected=False;automatic_exit=False
def descend(pid):
 try:ids=[int(x) for x in Path(f'/proc/{pid}/task/{pid}/children').read_text().split()]
 except FileNotFoundError:return
 for child in ids:children.add(child);descend(child)
try:
 while time.monotonic()-start<60 and p.poll() is None:
  descend(p.pid)
  if rows and rows[-1][0]>5:
   rf=w/'log/platform-ready-usv_performance_launcher_test.json'
   if args.control!='ros' or (rf.exists() and rf.stat().st_mtime>=started_epoch):break
  time.sleep(.1)
 if args.kill_controller:
  descend(p.pid)
  for child in children:
   try:command=Path(f'/proc/{child}/cmdline').read_bytes()
   except FileNotFoundError:continue
   if b'/boat_control.py' in command:os.kill(child,signal.SIGTERM);injected=True;break
  if injected:
   try:p.wait(timeout=15);automatic_exit=True
   except subprocess.TimeoutExpired:pass
finally:
 if p.poll() is None:
  os.kill(p.pid,signal.SIGINT)
  try:p.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 f.close()
live=[]
for child in children:
 try:
  status=Path(f'/proc/{child}/status').read_text()
  if '\nState:\tZ' not in status:live.append(child)
 except FileNotFoundError:pass
ready_file=w/'log/platform-ready-usv_performance_launcher_test.json'
checks={'startup_ready':bool(ready_file.exists() and ready_file.stat().st_mtime>=started_epoch and json.loads(ready_file.read_text())['ready']) if args.control=='ros' else True,'physics_stream':bool(rows and rows[-1][0]>5),'no_uncommanded_propulsion':bool(rows and max(abs(r[1]) for r in rows)<1e-9),'bounded_cleanup':p.returncode in [0,130],'no_live_children':not live}
if args.kill_controller:checks.update(injected_controller_exit=injected,stack_exits_automatically=automatic_exit)
r={'passed':all(checks.values()),'checks':checks,'exit_code':p.returncode,'observed_children':len(children),'remaining_children':live};(out/(stem+'.json')).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
