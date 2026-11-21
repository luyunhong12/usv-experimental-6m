#!/usr/bin/env python3
"""Bounded integration test; writes telemetry, image and explicit pass/fail evidence."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,os,time,signal,subprocess,json,math,threading
from pathlib import Path
import xml.etree.ElementTree as E
p=argparse.ArgumentParser();p.add_argument('--case',choices=['equilibrium','disturbed','steering_contact','steering_limits'],default='equilibrium');args=p.parse_args()
W=WORKSPACE;out=W/'log/hull_validation'/args.case;out.mkdir(parents=True,exist_ok=True)
os.environ['GZ_PARTITION']='usv_validate_'+args.case
os.environ.setdefault('GZ_IP','127.0.0.1')
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.image_pb2 import Image
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double
import numpy as np
from PIL import Image as PILImage
world=E.parse(W/'src/usv_simulation/worlds/usv_hull_trial.sdf');ww=world.getroot().find('world');ww.set('name','trial_'+args.case)
pose=ww.find('include/pose');q=list(map(float,pose.text.split()))
if args.case=='disturbed':q[2]+=.08;q[3]+=math.radians(5);q[4]+=math.radians(2)
pose.text=' '.join(map(str,q))
# Static inspection camera is test instrumentation, not a vessel sensor.
cam=E.SubElement(ww,'model',name='inspection_camera');E.SubElement(cam,'static').text='true';E.SubElement(cam,'pose').text='-8 -9 5 0 0.36 0.84';l=E.SubElement(cam,'link',name='camera');s=E.SubElement(l,'sensor',name='camera',type='camera');E.SubElement(s,'always_on').text='true';E.SubElement(s,'update_rate').text='1';E.SubElement(s,'topic').text='/usv/inspection/image';c=E.SubElement(s,'camera');E.SubElement(c,'horizontal_fov').text='1.0';im=E.SubElement(c,'image');E.SubElement(im,'width').text='1200';E.SubElement(im,'height').text='800';E.SubElement(im,'format').text='R8G8B8';cl=E.SubElement(c,'clip');E.SubElement(cl,'near').text='.1';E.SubElement(cl,'far').text='1000'
if args.case=='steering_contact':
 ball=E.SubElement(ww,'model',name='contact_probe');E.SubElement(ball,'pose').text='0 0 2.5 0 0 0';bl=E.SubElement(ball,'link',name='ball');bi=E.SubElement(bl,'inertial');E.SubElement(bi,'mass').text='1';ii=E.SubElement(bi,'inertia')
 for k in ['ixx','iyy','izz']:E.SubElement(ii,k).text='.004'
 for kind in ['collision','visual']:
  g=E.SubElement(E.SubElement(bl,kind,name=kind),'geometry');E.SubElement(E.SubElement(g,'sphere'),'radius').text='.1'
 pp=E.SubElement(ball,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher');E.SubElement(pp,'publish_model_pose').text='true';E.SubElement(pp,'use_pose_vector_msg').text='true'
f=out/('trial_'+args.case+'.sdf');world.write(f,encoding='utf-8',xml_declaration=True)
rows=[];joints=[];probe=[];latest={};lock=threading.Lock();node=Node()
def hydro(m):
 with lock:rows.append(list(m.data))
def picture(m):
 with lock:latest['image']=(m.width,m.height,m.step,bytes(m.data),m.pixel_format_type)
def joint(m):
 for j in m.joint:
  if j.name=='steering':
   with lock:joints.append([j.axis1.position,j.axis1.velocity])
def ballpose(m):
 for p in m.pose:
  if p.name=='contact_probe':
   with lock:probe.append(p.position.z)
node.subscribe(Model,'/usv/joint_state',joint);node.subscribe(Pose_V,'/model/contact_probe/pose',ballpose)
command=node.advertise('/usv/steering/angle',Double)
node.subscribe(Double_V,'/usv/hydrostatics',hydro);node.subscribe(Image,'/usv/inspection/image',picture)
log=(out/'launch.log').open('w');proc=subprocess.Popen(['bash',str(tool_path('run_hull_trial.sh')),'world:='+str(f)],cwd=W,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 start=time.monotonic();commanded=False;limit_stage=0
 while time.monotonic()-start<55:
  if proc.poll() is not None:break
  if 'core dumped' in (out/'launch.log').read_text(errors='replace'):break
  with lock:done=bool(rows and rows[-1][0]>=30 and 'image' in latest)
  if args.case=='steering_contact' and not commanded and rows and rows[-1][0]>5:
   msg=Double();msg.data=.3;command.publish(msg);commanded=True
  if args.case=='steering_limits' and rows:
   schedule=[(5,-math.radians(35)),(12,math.radians(35)),(24,0.)]
   if limit_stage<len(schedule) and rows[-1][0]>=schedule[limit_stage][0]:
    msg=Double();msg.data=schedule[limit_stage][1];command.publish(msg);limit_stage+=1
  if done:break
  time.sleep(.2)
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=6)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close()
with lock:data=np.array(rows);image=latest.get('image')
if image:
 width,height,step,raw,fmt=image
 if len(raw)==height*step and step>=width*3:
  a=np.frombuffer(raw,dtype=np.uint8).reshape(height,step)[:,:width*3].reshape(height,width,3);PILImage.fromarray(a).save(out/'gazebo_view.png')
np.savetxt(out/'telemetry.csv',data,delimiter=',',header='time,volume,cb_x,cb_y,cb_z,buoyancy_N,x,y,z,roll,pitch,yaw',comments='')
expected=json.loads((W/'../datasets/reference/hydrostatic_trial.json').read_text());target=np.array(expected['equilibrium_pose_z_roll_pitch']);result={'case':args.case,'samples':len(data),'render_captured':bool(image),'passed':False}
if len(data):
 final=data[-1];tail=data[data[:,0]>max(0,final[0]-5)]
 result.update(final_sim_time_s=float(final[0]),final_z_roll_pitch=final[[8,9,10]].tolist(),target_z_roll_pitch=target.tolist(),final_displacement_m3=float(final[1]),tail_max_pose_error=np.max(np.abs(tail[:,[8,9,10]]-target),axis=0).tolist(),finite=bool(np.isfinite(data).all()))
 result['passed']=bool(result['finite'] and final[0]>=25 and np.all(np.array(result['tail_max_pose_error'])<[.01,.01,.01]) and abs(final[1]-expected['displaced_volume_m3'])<.02)
if args.case=='steering_contact':
 result['final_steering_rad']=joints[-1][0] if joints else None
 result['final_probe_z_m']=probe[-1] if probe else None
 result['max_steering_rad']=max(abs(j[0]) for j in joints) if joints else None
 result['max_steering_velocity_rad_s']=max(abs(j[1]) for j in joints) if joints else None
 result['passed']=bool(result['passed'] and joints and abs(joints[-1][0]-.3)<.03 and max(abs(j[0]) for j in joints)<math.radians(35.1) and probe and .5<probe[-1]<1.5 and max(abs(j[1]) for j in joints)<.21 and abs(final[1]-(expected['displaced_volume_m3']+.001))<.0001)
 (out/'joint_probe.json').write_text(json.dumps({'joints_angle_velocity':joints,'probe_z':probe}))
if args.case=='steering_limits':
 result['steering_min_max_rad']=[min(j[0] for j in joints),max(j[0] for j in joints)] if joints else None
 result['max_steering_velocity_rad_s']=max(abs(j[1]) for j in joints) if joints else None
 result['final_steering_rad']=joints[-1][0] if joints else None
 result['passed']=bool(result['passed'] and joints and result['steering_min_max_rad'][0]<-.59 and result['steering_min_max_rad'][1]>.59 and max(abs(j[0]) for j in joints)<math.radians(35.1) and result['max_steering_velocity_rad_s']<.21 and abs(joints[-1][0])<.03)
 (out/'joints.json').write_text(json.dumps(joints))
(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
