#!/usr/bin/env python3
"""Actual local ArduRover AUTO waypoints, LOITER, RTL and GCS-loss test.
Independent instance/partition; normal arming; never connects to real hardware.
"""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import os,sys,time,json,signal,subprocess,math,argparse,socket,struct
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as E
import shutil
root=WORKSPACE;sys.path.insert(0,str(root/'../dependencies/vendor/python'))
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--label',default='navigation');p.add_argument('--failsafe-only',action='store_true');a=p.parse_args()
if not a.label.isidentifier():p.error('Invalid label')
os.environ['GZ_PARTITION']='usv_platform_'+a.label;os.environ['GZ_IP']='127.0.0.1'
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.vector3d_pb2 import Vector3d
out=root/'log/platform_validation'/a.label;out.mkdir(parents=True,exist_ok=True)
rc_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
node=Node();rows=[];energy=[];node.subscribe(Double_V,'/usv/propulsion/state',lambda m:rows.append(list(m.data)));node.subscribe(Double_V,'/usv/energy/truth',lambda m:energy.append(list(m.data)))
flow=node.advertise('/vrx/current/velocity',Vector3d)
world=out/'world.sdf';subprocess.run(['python3',str(tool_path('prepare_sydney.py')),'--profile','speed15_concept','--output',str(world)],check=True,stdout=subprocess.DEVNULL)
# Capture the exact boat and flight-control defaults before launching.
tree=E.parse(world)
for inc in tree.findall('world/include'):
 if inc.findtext('name')=='usv':
  snapshot=out/'boat.sdf';shutil.copy2(inc.findtext('uri'),snapshot);inc.find('uri').text=str(snapshot)
tree.write(world,encoding='utf-8',xml_declaration=True)
for parameter_file in ['boat.parm','platform15.parm']:shutil.copy2(root/'src/usv_ardupilot/config'/parameter_file,out/parameter_file)
procs=[];files=[]
def start(args,name):
 f=(out/name).open('w');files.append(f);p=subprocess.Popen(args,cwd=out,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
start(['bash',str(tool_path('run_hull_trial.sh')),'world:='+str(world)],'gazebo.log')
start(['bash',str(tool_path('run_ardupilot.sh')),'--port','9432','--log',str(out/'json_frames.jsonl')],'bridge.log')
start(['python3',str(tool_path('run_sitl.py')),'--instance','43','--profile','platform15','--wipe'],'sitl.log')
conn=None;status=[];events=[];track=[];home=None;waypoints=None;reached=set();requested=set();ack=False;armed=False;mode=None;gps=None;phase='init';phase_t=0.;last_hb=last_arm=last_upload=last_rc=0.;begin=time.monotonic();checks={};rc_health=[];loiter_center=None;loiter_errors=[];home_errors=[];phase_start=None
M=mavutil.mavlink

def event(kind,**data):
 events.append(dict(time=rows[-1][0] if rows else 0,event=kind,**data));print(events[-1],flush=True)
def setmode(value):conn.mav.set_mode_send(1,M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,value)
def transition(value,sim):
 global phase,phase_t
 phase=value;phase_t=sim;event(value)
def offset(lat,lon,east,north):return lat+north/111319.49,lon+east/(111319.49*math.cos(math.radians(lat)))
def send_item(seq):
 lat,lon=waypoints[seq]
 conn.mav.mission_item_int_send(1,1,seq,M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,M.MAV_CMD_NAV_WAYPOINT,0,1,0,3,0,0,int(lat*1e7),int(lon*1e7),0)
try:
 while time.monotonic()-begin<660:
  if any(p.poll() is not None for p in procs):event('process_exit');break
  if conn is None:
   try:conn=mavutil.mavlink_connection('tcp:127.0.0.1:6190',source_system=255,source_component=190)
   except (ConnectionError,OSError):time.sleep(.2);continue
  now=time.monotonic();sim=rows[-1][0] if rows else 0
  for _ in range(60):
   m=conn.recv_match(blocking=False)
   if m is None:break
   kind=m.get_type()
   if kind=='STATUSTEXT':
    status.append(m.text);print('SITL:',m.text,flush=True)
   elif kind=='HEARTBEAT':armed=bool(m.base_mode&M.MAV_MODE_FLAG_SAFETY_ARMED);mode=m.custom_mode
   elif kind=='GLOBAL_POSITION_INT':gps=(m.lat*1e-7,m.lon*1e-7)
   elif kind=='SYS_STATUS' and phase=='gcs_loss':rc_health.append(bool(m.onboard_control_sensors_health & M.MAV_SYS_STATUS_SENSOR_RC_RECEIVER))
   elif kind in ['MISSION_REQUEST_INT','MISSION_REQUEST'] and waypoints is not None and m.seq<len(waypoints):
    requested.add(m.seq);send_item(m.seq)
   elif kind=='MISSION_ACK':ack=bool(waypoints is not None and requested==set(range(len(waypoints))) and m.type==M.MAV_MISSION_ACCEPTED)
   elif kind=='MISSION_ITEM_REACHED':reached.add(m.seq);event('waypoint_reached',seq=m.seq)
  if phase!='gcs_loss' and now-last_hb>1:
   conn.mav.heartbeat_send(M.MAV_TYPE_GCS,M.MAV_AUTOPILOT_INVALID,0,0,0);last_hb=now
  # Keep an independent neutral RC link alive when removing ONLY GCS heartbeat.
  # Native SITL RC UDP is independent: MAVLink RC overrides also refresh GCS last-seen.
  if now-last_rc>.2:
   rc_socket.sendto(struct.pack('<8H',1500,1500,1500,1500,1800,1000,1000,1800),('127.0.0.1',5931));last_rc=now
  if phase=='init':
   if now-last_arm>4:
    setmode(15);conn.mav.request_data_stream_send(1,1,M.MAV_DATA_STREAM_ALL,10,1)
    if mode==15:conn.mav.command_long_send(1,1,M.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
    last_arm=now
   if armed and gps and rows:
    checks['normal_arming']=True;home=rows[-1][12:14];phase_start=sim
    if a.failsafe_only:
     lat,lon=offset(*gps,40,0)
     conn.mav.mission_item_int_send(1,1,0,M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,M.MAV_CMD_NAV_WAYPOINT,2,0,0,0,0,0,int(lat*1e7),int(lon*1e7),0)
     transition('outbound',sim);continue
    waypoints=[gps,offset(*gps,70,0),offset(*gps,70,40),offset(*gps,20,40)]
    conn.mav.mission_clear_all_send(1,1);transition('upload',sim)
  elif phase=='upload':
   if now-last_upload>5:conn.mav.mission_count_send(1,1,len(waypoints));last_upload=now
   if ack:
    checks['mission_upload']=True;conn.mav.mission_set_current_send(1,1,1);setmode(10);transition('start_auto',sim)
  elif phase=='start_auto':
   if mode==10:transition('auto',sim)
   elif now-last_arm>2:setmode(10);last_arm=now
   if sim-phase_t>45:event('auto_mode_rejected');break
  elif phase=='auto':
   if 3 in reached:
    checks['auto_waypoints']=bool({1,2,3}.issubset(reached) and armed);loiter_center=rows[-1][12:14];setmode(5);transition('loiter',sim)
   elif sim-phase_t>240:event('auto_timeout');break
  elif phase=='loiter':
   flow.publish(Vector3d(x=.15,y=.1,z=0))
   if sim-phase_t>15:loiter_errors.append(float(np.linalg.norm(np.array(rows[-1][12:14])-loiter_center)))
   if sim-phase_t>60:
    checks['loiter_mode']=mode==5;checks['loiter_radius']=bool(loiter_errors and max(loiter_errors)<12)
    flow.publish(Vector3d(x=0,y=0,z=0));setmode(11);transition('rtl',sim)
  elif phase=='rtl':
   dist=float(np.linalg.norm(np.array(rows[-1][12:14])-home));home_errors.append(dist)
   if dist<6 and sim-phase_t>5:
    checks['rtl_mode']=mode==11;checks['rtl_home']=True
    # Head away again in GUIDED, then stop only GCS heartbeats. Keep JSON physics alive.
    setmode(15);lat,lon=offset(*gps,40,0)
    conn.mav.mission_item_int_send(1,1,0,M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,M.MAV_CMD_NAV_WAYPOINT,2,0,0,0,0,0,int(lat*1e7),int(lon*1e7),0)
    transition('outbound',sim)
   elif sim-phase_t>180:event('rtl_timeout');break
  elif phase=='outbound':
   if sim-phase_t>12:checks['guided_mode']=mode==15;checks['moving_before_loss']=abs(rows[-1][10])>.3;transition('gcs_loss',sim)
  elif phase=='gcs_loss':
   if sim-phase_t>18:
    checks['gcs_failsafe_hold']=mode==4;checks['gcs_failsafe_stops_command']=abs(rows[-1][1])<.01;checks['failsafe_status_reported']=any(s.endswith(' Failsafe') for s in status);checks['independent_rc_link_healthy']=bool(rc_health and all(rc_health))
    transition('complete',sim);break
  if rows:track.append([sim,rows[-1][12],rows[-1][13],rows[-1][10],mode if mode is not None else -1])
  time.sleep(.01)
except KeyboardInterrupt:
 event('interrupted');phase='interrupted'
finally:
 if conn:
  setmode(4);conn.mav.rc_channels_override_send(1,1,1500,1500,1500,1500,65535,65535,65535,65535)
  conn.mav.command_long_send(1,1,M.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
 for p in reversed(procs):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in files:f.close()
 rc_socket.close()
if not a.failsafe_only:
 logs=root/'log/ardupilot/platform15/instance_43/logs'
 if list(logs.glob('*.BIN')):shutil.copy2(max(logs.glob('*.BIN'),key=lambda p:p.stat().st_mtime),out/'flight_control.BIN')
checks['completed']=phase=='complete'
if not a.failsafe_only and home is not None and track:
 tr=np.array(track);points=np.array([[0.,0.],[70.,0.],[70.,40.],[20.,40.]])+home
 def distance_to_segment(points,start,end):
  v=end-start;q=np.clip((points-start)@v/np.dot(v,v),0,1);return np.linalg.norm(points-(start+q[:,None]*v),axis=1)
 auto=tr[tr[:,4]==10,1:3]
 if len(auto):
  path_error=np.min(np.array([distance_to_segment(auto,points[i],points[i+1]) for i in range(3)]),axis=0)
  checks['auto_path_corridor']=bool(np.max(path_error)<15)
  events.append({'event':'auto_path_audit','max_distance_to_route_m':float(np.max(path_error))})
checks['finite']=bool(rows and np.isfinite(rows).all())
r=dict(passed=all(checks.values()),checks=checks,final_phase=phase,events=events,waypoints_lat_lon=waypoints,home_xy=home,loiter_max_error_m=max(loiter_errors) if loiter_errors else None,rtl_min_distance_m=min(home_errors) if home_errors else None,max_speed_km_h=max((r[10] for r in rows),default=0)*3.6,status_tail=status[-30:],rc_health_samples_during_gcs_loss=rc_health,rc_source='native SITL UDP 5931; no MAVLink RC overrides during experiment')
for name,data in [('propulsion',rows),('energy',energy),('track',track)]:np.savetxt(out/(name+'.csv'),np.array(data),delimiter=',')
(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));
if not a.failsafe_only and (out/'flight_control.BIN').exists():
 audit=subprocess.run(['python3',str(tool_path('audit_navigation_clock.py')),a.label]);r['passed']=r['passed'] and audit.returncode==0
 (out/'result.json').write_text(json.dumps(r,indent=2))
raise SystemExit(0 if r['passed'] else 1)
