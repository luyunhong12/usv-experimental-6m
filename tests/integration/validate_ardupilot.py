#!/usr/bin/env python3
"""Local SITL handshake, normal arming and manual forward/reverse/turn check."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import os,sys,time,json,signal,subprocess,math,argparse,shutil
import xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
root=WORKSPACE;sys.path.insert(0,str(root/'../dependencies/vendor/python'))
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--profile',choices=['baseline','platform15'],default='baseline');p.add_argument('--label');a=p.parse_args()
if a.label and not a.label.isidentifier():p.error('Invalid label')
os.environ['GZ_PARTITION']='usv_ardupilot_validation';os.environ['GZ_IP']='127.0.0.1'
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
out=root/('log/platform_validation/'+a.label if a.label else 'log/integration_upgrade/ardupilot');out.mkdir(parents=True,exist_ok=True)
node=Node();rows=[];node.subscribe(Double_V,'/usv/propulsion/state',lambda m:rows.append(list(m.data)))
world=out/'world.sdf';subprocess.run(['python3',str(tool_path('prepare_sydney.py')),'--output',str(world)]+(['--profile','speed15_concept'] if a.profile=='platform15' else []),check=True,stdout=subprocess.DEVNULL)
tree=E.parse(world)
for inc in tree.findall('world/include'):
 if inc.findtext('name')=='usv':
  snapshot=out/'boat.sdf';shutil.copy2(inc.findtext('uri'),snapshot);inc.find('uri').text=str(snapshot)
tree.write(world,encoding='utf-8',xml_declaration=True)
for filename in ['boat.parm','platform15.parm']:shutil.copy2(root/'src/usv_ardupilot/config'/filename,out/filename)
procs=[];files=[]
def start(args,name):
 f=(out/name).open('w');files.append(f);p=subprocess.Popen(args,cwd=out,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
start(['bash',str(tool_path('run_hull_trial.sh')),'world:='+str(world)],'gazebo.log')
start(['bash',str(tool_path('run_ardupilot.sh')),'--port','9422','--log',str(out/'json_frames.jsonl')],'bridge.log')
# Instance 42 has independent TCP/UDP ports and EEPROM/log directory.
start(['python3',str(tool_path('run_sitl.py')),'--instance','42','--profile',a.profile,'--wipe'],'sitl.log')
status=[];attitudes=[];heartbeats=[];servo=[];phase_start=None;armed=False;conn=None;begin=time.monotonic();last_rc=last_hb=last_arm=0.;result={}
try:
 while time.monotonic()-begin<240:
  if any(p.poll() is not None for p in procs):break
  if conn is None:
   try:conn=mavutil.mavlink_connection('tcp:127.0.0.1:6180',source_system=255,source_component=190)
   except (ConnectionError,OSError):time.sleep(.2);continue
  m=conn.recv_match(blocking=False)
  if m:
   kind=m.get_type()
   if kind=='STATUSTEXT':status.append(m.text)
   elif kind=='HEARTBEAT':
    heartbeats.append(m.to_dict());armed=bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
   elif kind=='ATTITUDE':attitudes.append(m.to_dict())
   elif kind=='SERVO_OUTPUT_RAW':servo.append(m.to_dict())
  now=time.monotonic();sim=rows[-1][0] if rows else 0
  if now-last_hb>1:
   conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0);last_hb=now
  if heartbeats and now-last_arm>4 and phase_start is None:
   conn.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,0)
   conn.mav.request_data_stream_send(1,1,mavutil.mavlink.MAV_DATA_STREAM_ALL,10,1)
   conn.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0);last_arm=now
  if armed and rows and phase_start is None:phase_start=sim
  elapsed=sim-phase_start if phase_start is not None else -1
  if now-last_rc>.1:
   thr=1800 if 0<=elapsed<12 or 28<=elapsed<48 else 1200 if 12<=elapsed<28 else 1500
   steer=1800 if 28<=elapsed<48 else 1500
   conn.mav.rc_channels_override_send(1,1,steer,1500,thr,1500,65535,65535,65535,65535);last_rc=now
  if elapsed>=55:break
  time.sleep(.003)
finally:
 if conn:
  conn.mav.rc_channels_override_send(1,1,1500,1500,1500,1500,65535,65535,65535,65535)
  conn.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
 for p in reversed(procs):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in files:f.close()
np.savetxt(out/'propulsion.csv',np.array(rows),delimiter=',')
(out/'mavlink.json').write_text(json.dumps(dict(status=status,attitude=attitudes,heartbeat=heartbeats,servo=servo),indent=2))
checks={'heartbeat':bool(heartbeats),'normal_arming':phase_start is not None,'attitude_telemetry':bool(attitudes),'servo_telemetry':bool(servo)}
if phase_start is not None and rows:
 d=np.array(rows);t=d[:,0]-phase_start;fwd=d[(t>8)&(t<11)];rev=d[(t>24)&(t<27)];turn=d[(t>44)&(t<47)]
 checks['completed']=bool(t[-1]>=54)
 checks['forward']=bool(len(fwd) and np.mean(fwd[:,10])>.2)
 checks['reverse']=bool(len(rev) and np.mean(rev[:,10])<-.2)
 checks['right_turn']=bool(len(turn) and np.unwrap(d[:,14])[np.where(t>44)[0][0]]<-.2)
 result.update(phase_start_sim_s=phase_start,forward_speed_m_s=float(np.mean(fwd[:,10])) if len(fwd) else None,reverse_speed_m_s=float(np.mean(rev[:,10])) if len(rev) else None)
result.update(passed=all(checks.values()),checks=checks,status_tail=status[-15:]);(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
