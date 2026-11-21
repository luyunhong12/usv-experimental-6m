#!/usr/bin/env python3
"""Interrupt a private suite and verify it leaves no matching native processes."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,json,os,signal,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--kill-child',action='store_true');a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);w=WORKSPACE
spec={'cancel_probe':{'kind':'cancellation','stages':[{'name':'running','duration':120,'mode':'velocity','speed':1.5}]}};(out/'spec.json').write_text(json.dumps(spec))
with (out/'suite.log').open('w') as log:
 proc=subprocess.Popen(['python3',str(tool_path('run_performance_suite.py')),'--spec',str(out/'spec.json'),'--output',str(out),'--workers','1'],stdout=log,stderr=subprocess.STDOUT)
 begin=time.monotonic()
 while time.monotonic()-begin<45 and proc.poll() is None and not (out/'cancel_probe/startup.json').exists():time.sleep(.2)
 was_ready=(out/'cancel_probe/startup.json').exists()
 if a.kill_child:
  injected=False
  children=set()
  for task in Path(f'/proc/{proc.pid}/task').glob('*/children'):
   try:children.update(task.read_text().split())
   except FileNotFoundError:pass
  for child in children:
   if b'validate_vessel_performance.py' in Path(f'/proc/{child}/cmdline').read_bytes():os.kill(int(child),signal.SIGKILL);injected=True;break
  if not injected:proc.terminate()
 else:proc.terminate()
 try:proc.wait(timeout=25)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
meta=json.loads((out/'cancel_probe/source_versions.json').read_text());key=('GZ_PARTITION='+meta['partition']).encode();remaining=[]
for item in Path('/proc').glob('[0-9]*'):
 try:
  if item.stat().st_uid==os.getuid() and key in (item/'environ').read_bytes().split(b'\0'):remaining.append(int(item.name))
 except OSError:pass
result_file=out/'cancel_probe/result.json';result=json.loads(result_file.read_text()) if result_file.exists() else {}
checks={'started_ready':was_ready,'no_partition_processes_remain':not remaining,'interruption_recorded':result.get('failure')=='Interrupted signal 15','cancelled_run_not_passed':result.get('passed') is False}
if a.kill_child:
 checks['interruption_recorded']='END cancel_probe -9' in (out/'suite.log').read_text()
 checks['cancelled_run_not_passed']=result.get('passed') is not True
r={'passed':all(checks.values()),'checks':checks,'remaining_pids':remaining,'suite_exit_code':proc.returncode};(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
