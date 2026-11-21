#!/usr/bin/env python3
"""ROS-controlled refined assembly: steering, real shaft spin, collision monitoring."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import os,time,json,subprocess,signal,math,threading,argparse
from pathlib import Path
import xml.etree.ElementTree as E
import shutil,numpy as np
from scipy.spatial.transform import Rotation
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);args=parser.parse_args()
w=WORKSPACE;out=args.output or w/'../artifacts/runs/船体三项优化_2026-11-20/assembly';out.mkdir(parents=True,exist_ok=True)
os.environ.update(GZ_PARTITION='usv_refined_assembly',GZ_IP='127.0.0.1',ROS_DOMAIN_ID='76')
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.model_pb2 import Model
from gz.msgs10.contacts_pb2 import Contacts
import rclpy
from std_msgs.msg import Float64
node=Node();latest={};poses={};contacts=[];joint={};rows=[]
def st(m,key):latest[key]=list(m.data)
for k in ['actuators/state','propulsion/state','energy/truth']:node.subscribe(Double_V,'/usv/'+k,lambda m,k=k:st(m,k))
def poses_cb(m):
 for p in m.pose:
  if p.name in ['azimuth','propeller']:poses[p.name]=p
node.subscribe(Pose_V,'/world/usv_sydney/dynamic_pose/info',poses_cb)
def joint_cb(m):
 for j in m.joint:joint[j.name]=j.axis1.position
node.subscribe(Model,'/usv/joint_state',joint_cb)
world=out/'world.sdf';subprocess.run(['python3',str(tool_path('prepare_sydney.py')),'--profile','speed15_concept','--wave-gain','0','--output',str(world)],check=True,stdout=subprocess.DEVNULL)
t=E.parse(world);inc=t.find("world/include[name='usv']");shutil.copy2(inc.findtext('uri'),out/'boat.sdf');inc.find('uri').text=str(out/'boat.sdf');t.write(world,encoding='utf-8',xml_declaration=True)
callbacks=[]
for sensor in E.parse(out/'boat.sdf').findall("model/link/sensor[@type='contact']"):
 topic=sensor.findtext('topic')
 def cb(m,key=topic):
  if len(m.contact):contacts.append({'topic':key,'count':len(m.contact)})
 callbacks.append(cb);node.subscribe(Contacts,topic,cb)
rclpy.init();ros=rclpy.create_node('refined_assembly_validator');th=ros.create_publisher(Float64,'/usv/actuators/throttle',10);steer=ros.create_publisher(Float64,'/usv/actuators/steering',10)
procs=[];logs=[]
for name,args in [('gazebo',['bash',str(tool_path('run_hull_trial.sh')),'world:='+str(world)]),('ros',['bash',str(tool_path('run_interfaces.sh')),'speed_scale:=4.3'])]:
 f=(out/(name+'.log')).open('w');logs.append(f);procs.append(subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
stages=[('center',0,0,8),('left_idle',0,-30,12),('right_idle',0,30,14),('center_return',0,0,12),('spin_forward',.35,0,12),('spin_reverse',-.35,0,14),('ahead',.35,0,14),('left_underway',.35,-15,14),('right_underway',.35,15,16),('stop',0,0,30)]
phase=0;start_sim=None;wall=time.monotonic();failure=None
try:
 while time.monotonic()-wall<330:
  if any(p.poll() is not None for p in procs):raise RuntimeError('Child process exited')
  rclpy.spin_once(ros,timeout_sec=.005)
  if all(k in latest for k in ['actuators/state','propulsion/state','energy/truth']) and all(k in poses for k in ['azimuth','propeller']) and 'propeller_spin' in joint:
   a=latest['actuators/state'];p=latest['propulsion/state'];sim=a[0]
   if start_sim is None:start_sim=sim
   name,cmd,deg,duration=stages[phase];th.publish(Float64(data=float(cmd)));steer.publish(Float64(data=math.radians(deg)))
   headings=[]
   for key in ['azimuth','propeller']:
    q=poses[key].orientation;v=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([1,0,0]);headings.append(math.atan2(v[1],v[0]))
   rows.append([sim,phase,deg,math.degrees(a[1]),*np.rad2deg(headings),joint['propeller_spin'],p[2],p[10],p[14],p[15],p[16],a[4],a[5],p[12],p[13]])
   if sim-start_sim>duration:
    print(name,'complete at',sim,flush=True);phase+=1;start_sim=sim
    if phase==len(stages):break
  time.sleep(.02)
except (Exception,KeyboardInterrupt) as e:failure=str(e) or type(e).__name__
finally:
 for _ in range(5):th.publish(Float64(data=0.));steer.publish(Float64(data=0.));rclpy.spin_once(ros,timeout_sec=.02)
 for p in reversed(procs):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for f in logs:f.close()
 ros.destroy_node();rclpy.shutdown()
d=np.array(rows);checks={'completed':phase==len(stages),'finite':bool(rows and np.isfinite(d).all()),'no_self_collision_contacts':not contacts,'no_process_failure':failure is None};metrics={}
if len(d):
 for i,(name,cmd,deg,duration) in enumerate(stages):
  part=d[d[:,1]==i]
  if not len(part):continue
  tail=part[part[:,0]>part[-1,0]-3]
  metrics[name]={'actual_angle_deg':float(np.mean(tail[:,3])),'mesh_joint_error_deg':float(np.max(abs(tail[:,4:6]-tail[:,3,None]))),'rpm':float(np.mean(tail[:,7])),'shaft_rotation_rad':float(np.ptp(part[:,6])),'tail_heading_change_rad':float(np.unwrap(tail[:,9])[-1]-np.unwrap(tail[:,9])[0])}
  if i in [0,1,2,3,9]:checks[name+'_angle']=abs(metrics[name]['actual_angle_deg']-deg)<1
  checks[name+'_mesh_follows_joint']=metrics[name]['mesh_joint_error_deg']<.5
 for name,sign in [('spin_forward',1),('spin_reverse',-1)]:
  x=metrics.get(name,{});checks[name]=x.get('rpm',0)*sign>50 and x.get('shaft_rotation_rad',0)>2*math.pi
 for name,sign in [('left_underway',1),('right_underway',-1)]:checks[name]=metrics.get(name,{}).get('tail_heading_change_rad',0)*sign>.03
 checks['attitude_bounded']=float(np.max(abs(d[:,10:12])))<math.radians(20)
r={'passed':all(checks.values()),'checks':checks,'stages':metrics,'contacts':contacts[:100],'failure':failure,'control':'ROS direct actuator topics through boat_control.py; no ArduPilot'}
np.savetxt(out/'assembly.csv',d,delimiter=',',header='sim_s,stage,target_deg,actual_deg,azimuth_deg,propeller_axis_deg,shaft_rad,rpm,forward_speed,yaw,roll,pitch,servo_nm,servo_w,x,y',comments='');(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
