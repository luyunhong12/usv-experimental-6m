#!/usr/bin/env python3
"""Fuel mass / COM / full inertia integration test against an independent static reference."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,os,time,signal,subprocess,json
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from fuel_equilibrium import solve,properties,tank
from scipy.spatial.transform import Rotation
from energy_fields import TRUTH
p=argparse.ArgumentParser();p.add_argument('--case',choices=['full','half','low','burn','reset'],required=True);a=p.parse_args()
W=WORKSPACE;out=W/'log/fuel_mass_validation'/a.case;out.mkdir(parents=True,exist_ok=True)
os.environ['GZ_PARTITION']='usv_fuel_mass_'+a.case+'_'+str(os.getpid());os.environ.setdefault('GZ_IP','127.0.0.1')
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
litres={'full':60,'half':30,'low':6,'burn':30,'reset':30}[a.case]
model=E.parse(W/'src/usv_description/models/usv_azimuth_trial.sdf');model.find(".//plugin[@name='usv::AzimuthPropulsion']/energy/initial_fuel_l").text=str(litres);mf=out/'model.sdf';model.write(mf,encoding='utf-8',xml_declaration=True)
world=E.parse(W/'src/usv_simulation/worlds/usv_hull_trial.sdf');world.find('.//include/uri').text=str(mf);wf=out/'world.sdf';world.write(wf,encoding='utf-8',xml_declaration=True)
mass=[];energy=[];hydro=[];n=Node()
n.subscribe(Double_V,'/usv/fuel_mass/state',lambda m:mass.append(list(m.data)))
n.subscribe(Double_V,'/usv/energy/truth',lambda m:energy.append(list(m.data)))
n.subscribe(Double_V,'/usv/hydrostatics',lambda m:hydro.append(list(m.data)))
log=(out/'launch.log').open('w');proc=subprocess.Popen(['bash',str(tool_path('run_hull_trial.sh')),'world:='+str(wf)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 start=time.monotonic();gen=None;paused_ok=None;reset_sent=False;before_reset=None;reset_success=False
 while time.monotonic()-start<90 and proc.poll() is None:
  if mass and mass[-1][0]>=40 and (a.case!='reset' or reset_sent):break
  if mass and a.case in ['burn','reset']:
   if gen is None:gen=n.advertise('/usv/energy/generator_enable',Boolean)
   gen.publish(Boolean(data=not reset_sent))
  if mass and a.case=='reset':
   sim=mass[-1][0]
   if sim>=15 and paused_ok is None:
    ok,reply=n.request('/world/usv_hull_trial/control',WorldControl(pause=True),WorldControl,Boolean,2000)
    time.sleep(.2);saved=mass[-1].copy();time.sleep(.4);paused_ok=bool(ok and reply.data and mass[-1]==saved)
    n.request('/world/usv_hull_trial/control',WorldControl(pause=False),WorldControl,Boolean,2000)
   if sim>=24 and not reset_sent:
    before_reset=mass[-1].copy();req=WorldControl(pause=False);req.reset.all=True
    ok,reply=n.request('/world/usv_hull_trial/control',req,WorldControl,Boolean,2000);reset_success=bool(ok and reply.data);reset_sent=True
  time.sleep(.05)
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=6)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close()
 for topic in ['fuel_mass/state','energy/truth','hydrostatics']:n.unsubscribe('/usv/'+topic)
d=np.array(mass);e=np.array(energy);h=np.array(hydro)
fields='time,fuel_l,hull_mass,cgx,cgy,cgz,ixx,iyy,izz,ixy,ixz,iyz,ship_mass,ship_cgx,ship_cgy,ship_cgz,ship_ixx,ship_iyy,ship_izz,ship_ixy,ship_ixz,ship_iyz'
for name,data,header in [('mass',d,fields),('energy',e,','.join(TRUTH)),('hydro',h,'time,volume,cbx,cby,cbz,buoyancy,x,y,z,roll,pitch,yaw')]:np.savetxt(out/(name+'.csv'),data,delimiter=',',header=header,comments='')
r={'case':a.case,'passed':False,'samples':len(d)}
if len(d) and len(h) and len(e) and d[-1,0]>=39:
 expected=solve(float(d[-1,1]));(out/'expected.json').write_text(json.dumps(expected,indent=2));hp,sp=properties(float(d[-1,1]),Rotation.from_euler('xyz',h[-1,9:12]).as_matrix());I=np.array(hp['inertia_kg_m2']);want=I[[0,1,2,0,0,1],[0,1,2,1,2,2]]
 dry=tank['dry_hull']['mass_kg'];tail=h[h[:,0]>35];masserr=float(np.max(abs(d[:,2]-(dry+.84*d[:,1]))));cgerr=float(np.max(abs(d[-1,3:6]-hp['cg_m'])));ierr=float(np.max(abs(d[-1,6:12]-want)));poseerr=np.max(abs(tail[:,8:11]-expected['pose_z_roll_pitch']),axis=0)
 checks={'finite':bool(np.isfinite(d).all() and np.isfinite(h).all()),'mass_tracks_fuel':masserr<1e-7,'com_matches_independent_geometry':cgerr<2e-7,'full_tensor_matches_independent_geometry':ierr<1e-4,'floating_equilibrium':bool(np.all(poseerr<[.002,.002,.002])),'displacement_matches_mass':abs(h[-1,1]-d[-1,12]/1000)<.002,'energy_fuel_agrees':abs(d[-1,1]-e[-1,5])<.00001}
 if a.case=='reset':checks.update(pause_preserves_fuel=bool(paused_ok),reset_restores_mass=bool(reset_success and before_reset and before_reset[1]<29.999 and abs(d[-1,1]-30)<1e-10 and d[-1,2]>before_reset[2]),reset_time_rewinds=bool(np.any(np.diff(d[:,0])<0)),reset_turns_generator_off=bool(e[-1,11]==0))
 if a.case=='burn':checks.update(fuel_consumed=bool(d[0,1]-d[-1,1]>.001),native_mass_decreases=bool(d[0,2]-d[-1,2]>.00084),com_changes=bool(np.linalg.norm(d[0,3:6]-d[-1,3:6])>1e-7),inertia_changes=bool(np.linalg.norm(d[0,6:12]-d[-1,6:12])>1e-5))
 checks={k:bool(v) for k,v in checks.items()}
 r.update(passed=all(checks.values()),checks=checks,final_fuel_l=float(d[-1,1]),final_ship_mass_kg=float(d[-1,12]),fuel_consumed_l=float(d[0,1]-d[-1,1]),native_hull_mass_change_kg=float(d[-1,2]-d[0,2]),max_mass_error_kg=masserr,final_cg_error_m=cgerr,final_inertia_error_kg_m2=ierr,tail_max_pose_error=poseerr.tolist(),final_z_roll_pitch=h[-1,8:11].tolist(),final_sim_s=float(d[-1,0]))
(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
