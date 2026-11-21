#!/usr/bin/env python3
"""Isolated physics trials with bounded cancellation and preserved evidence."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,fcntl,json,os,signal,subprocess,sys,threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--skip-case',action='append',default=[]);p.add_argument('--workers',type=int,default=4);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
cases=json.loads(a.spec.read_text());active={};guard=threading.Lock();stop=threading.Event()
def run(case):
 if case in a.skip_case or stop.is_set():return
 with (a.output/(case+'.lock')).open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  result=a.output/case/'result.json'
  if stop.is_set() or (result.exists() and json.loads(result.read_text()).get('passed')):return
  print('START',case,flush=True)
  with (a.output/(case+'_console.log')).open('w') as log:
   with guard:
    if stop.is_set():return
    proc=subprocess.Popen([sys.executable,str(tool_path('validate_vessel_performance.py')),'--case',case,'--spec',str(a.spec.resolve()),'--output',str((a.output/case).resolve())],stdout=log,stderr=subprocess.STDOUT)
    active[case]=proc
   code=proc.wait()
   # Also clean up after a native abort, which bypasses the child's finally/atexit.
   metadata=a.output/case/'source_versions.json'
   if metadata.exists():
    partition=json.loads(metadata.read_text())['partition']
    subprocess.run([sys.executable,str(tool_path('stop_test_partition.py')),partition],stdout=subprocess.DEVNULL)
   with guard:active.pop(case,None)
  print('END',case,code,flush=True)
def interrupt(signum,frame):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupt)
priority=[k for k in ['clock_independent','neutral_rudder','curve_0_100','endurance_10min'] if k in cases];ordered=priority+[k for k in cases if k not in priority]
pool=ThreadPoolExecutor(max_workers=a.workers)
try:list(pool.map(run,ordered))
except BaseException:
 stop.set();signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
 with guard:
  for proc in active.values():
   if proc.poll() is None:proc.terminate()
 raise
finally:pool.shutdown(wait=True,cancel_futures=True)
results={case:json.loads((a.output/case/'result.json').read_text()) if (a.output/case/'result.json').exists() else {'passed':False,'failure':'no result'} for case in cases}
summary={'passed':all(r['passed'] for r in results.values()),'cases':results};(a.output/'summary.json').write_text(json.dumps(summary,indent=2));raise SystemExit(0 if summary['passed'] else 1)
