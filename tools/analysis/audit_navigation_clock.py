#!/usr/bin/env python3
"""Audit recorded ArduPilot disarm causes and bridge time steps after navigation."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import sys,json,argparse
from pathlib import Path
w=WORKSPACE;sys.path.insert(0,str(w/'../dependencies/vendor/python'))
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('label');a=p.parse_args()
if not a.label.isidentifier():p.error('Invalid label')
out=w/'log/platform_validation'/a.label
m=mavutil.mavlink_connection(str(out/'flight_control.BIN'));arm=[]
while True:
 d=m.recv_match(type='ARM')
 if d is None:break
 arm.append(d.to_dict())
t=json.loads((out/'json_frames.jsonl.summary.json').read_text())
checks={'armed_event_recorded':any(x['ArmState']==1 for x in arm),'no_cpu_failsafe_disarm':not any(x['ArmState']==0 and x['Method']==6 for x in arm),'bounded_steps':0<t['max_time_step_s']<=.020001,'physics_frames':t['frames_sent']>100}
r={'passed':all(checks.values()),'checks':checks,'arm_events':arm,'timing':t,'basis':'ArduPilot AP_Arming::Method::CPUFAILSAFE=6, local pinned source'}
(out/'flight_log_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
