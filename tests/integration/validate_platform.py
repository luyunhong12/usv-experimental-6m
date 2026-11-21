#!/usr/bin/env python3
"""Bounded engineering-envelope tests; strict nominal target, honest saturation under uncertainty."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--case',choices=['nominal','drag_low','drag_high','weather','endurance','command_loss','sensor_loss','payload'],required=True);p.add_argument('--output',type=Path);a=p.parse_args()
w=WORKSPACE;out=a.output if a.output else w/'log/platform_validation'/a.case;out.mkdir(parents=True,exist_ok=True)
os.environ.update(GZ_PARTITION='usv_platform_'+a.case,GZ_IP='127.0.0.1',ROS_DOMAIN_ID='74')
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.vector3d_pb2 import Vector3d
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
node=Node();rows={k:[] for k in ['propulsion/state','energy/truth','actuators/state','environment/state','fuel_mass/state']}
for topic in rows:node.subscribe(Double_V,'/usv/'+topic,lambda m,k=topic:rows[k].append(list(m.data)))
flow=node.advertise('/vrx/current/velocity',Vector3d);wind=node.advertise('/vrx/wind/velocity',Vector3d)
world=out/'world.sdf';subprocess.run(['python3',str(tool_path('prepare_sydney.py')),'--profile','speed15_concept','--payload-kg',str(100 if a.case=='payload' else 0),'--wave-gain',str(.3 if a.case=='weather' else 0),'--output',str(world)],check=True,stdout=subprocess.DEVNULL)
tree=E.parse(world);inc=tree.find("world/include[name='usv']");bt=E.parse(inc.findtext('uri'));bm=bt.find('model');scale={'drag_low':.75,'drag_high':1.25}.get(a.case,1.)
h=bm.find("plugin[@name='usv::HullHydrostatics']")
for tag in ['linear_damping','quadratic_damping','angular_damping','angular_quadratic_damping']:
 el=h.find(tag);el.text=' '.join(str(float(x)*scale) for x in el.text.split())
bf=out/'boat.sdf';bt.write(bf,encoding='utf-8',xml_declaration=True);inc.find('uri').text=str(bf);tree.write(world,encoding='utf-8',xml_declaration=True)
rclpy.init();ros=rclpy.create_node('platform_validator');pub=ros.create_publisher(Twist,'/usv/cmd_vel',10);control=[];ros.create_subscription(Float64MultiArray,'/usv/control/state',lambda m:control.append(list(m.data)),10)
procs=[];files=[]
for name,args in [('gazebo',['bash',str(tool_path('run_hull_trial.sh')),'world:='+str(world)]),('interfaces',['bash',str(tool_path('run_interfaces.sh')),'speed_scale:=4.3'])]:
 f=(out/(name+'.log')).open('w');files.append(f);procs.append(subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
duration=180 if a.case=='endurance' else 65;begin=time.monotonic();failure=None;bridge_pid=None;bridge_resumed=False
try:
 while time.monotonic()-begin<duration*3+60:
  if any(x.poll() is not None for x in procs):failure='child exited';break
  d=rows['propulsion/state'];t=d[-1][0] if d else 0
  if t>=duration:break
  if a.case=='sensor_loss' and t>20 and bridge_pid is None:
   # Pause only this launch's ROS parameter bridge, preserving Gazebo physics and controller.
   children=Path(f'/proc/{procs[1].pid}/task/{procs[1].pid}/children').read_text().split()
   for child in children:
    command=Path(f'/proc/{child}/cmdline').read_bytes().split(b'\0')[0]
    if command.endswith(b'/parameter_bridge'):bridge_pid=int(child);os.kill(bridge_pid,signal.SIGSTOP);break
   if bridge_pid is None:raise RuntimeError('Cannot locate owned parameter bridge for sensor dropout test')
  if bridge_pid and t>30 and not bridge_resumed:os.kill(bridge_pid,signal.SIGCONT);bridge_resumed=True
  if t>3 and not(a.case=='command_loss' and t>20):
   msg=Twist();msg.linear.x=15/3.6;msg.angular.z=.08 if a.case=='endurance' and t>15 else 0.;pub.publish(msg)
  if a.case=='weather' and t>5:flow.publish(Vector3d(x=.2,y=.15,z=0));wind.publish(Vector3d(x=0,y=8,z=0))
  rclpy.spin_once(ros,timeout_sec=.01);time.sleep(.02)
finally:
 if bridge_pid and not bridge_resumed:
  try:os.kill(bridge_pid,signal.SIGCONT)
  except ProcessLookupError:pass
 for x in reversed(procs):
  if x.poll() is None:
   os.killpg(x.pid,signal.SIGINT)
   try:x.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(x.pid,signal.SIGKILL);x.wait()
 for f in files:f.close()
 ros.destroy_node();rclpy.shutdown()
for k,v in rows.items():np.savetxt(out/(k.replace('/','_')+'.csv'),v,delimiter=',')
np.savetxt(out/'controller.csv',control,delimiter=',')
d=np.array(rows['propulsion/state']);e=np.array(rows['energy/truth']);c=np.array(control)
checks={'completed':bool(len(d) and d[-1,0]>=duration-.2),'finite':all(len(v)>0 and np.isfinite(v).all() for v in rows.values()),'processes_alive':failure is None};r={'case':a.case,'drag_scale':scale,'checks':checks}
if len(d) and len(e):
 tail=d[d[:,0]>duration-15];speed=float(np.mean(tail[:,10]))*3.6
 checks.update(attitude_bounded=bool(np.max(abs(d[:,15:17]))<math.radians(20)),power_limited=bool(e[:,7].max()<=40000.01),energy_balance=bool(np.max(abs(e[:,21]))<1e-6),soc_decreases=bool(e[-1,1]<e[0,1]))
 r.update(tail_speed_km_h=speed,max_roll_pitch_deg=np.rad2deg(np.max(abs(d[:,15:17]),axis=0)).tolist(),peak_power_w=float(e[:,7].max()),final_soc=float(e[-1,1]),max_temperature_c=float(e[:,4].max()),final_sim_s=float(d[-1,0]),saturation_fraction=float(np.mean(c[:,7])) if len(c) else None)
 if a.case in ['nominal','drag_low','payload']:checks['target_speed']=abs(speed-15)<.5
 elif a.case in ['drag_high','weather']:checks['bounded_forward_operation']=8<speed<17
 elif a.case=='endurance':checks.update(sustained_operation=10<speed<17,turns=bool(abs(np.unwrap(d[:,14])[-1]-d[0,14])>2*math.pi))
 elif a.case=='sensor_loss':
  gap=d[(d[:,0]>24)&(d[:,0]<29)];checks.update(dropout_injected=bridge_resumed,stops_when_navigation_stale=bool(len(gap) and np.max(abs(gap[:,1]))<1e-9),recovers_with_fresh_commands=abs(speed-15)<.5)
 elif a.case=='command_loss':
  after=d[d[:,0]>23];checks.update(was_moving=bool(np.max(d[d[:,0]<20,10])>3),stops_command=bool(len(after) and np.max(abs(after[:,1]))<1e-9),coasts_to_rest=abs(speed)<.5)
 r['heading_error_tail_deg']=float(np.max(abs(np.rad2deg(tail[:,14])))) if a.case!='endurance' else None
r['passed']=all(checks.values());(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
