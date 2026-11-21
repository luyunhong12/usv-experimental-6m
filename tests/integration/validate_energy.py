#!/usr/bin/env python3
"""Serial real-Gazebo tests; fixture overrides affect only a generated test model."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,os,time,signal,subprocess,json,math
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from energy_fields import TRUTH,SENSORS
p=argparse.ArgumentParser();p.add_argument('--case',choices=['drive','hybrid','charge_sensor','low_soc','fuel_out'],required=True);p.add_argument('--bridge',action='store_true');a=p.parse_args()
if a.bridge and a.case!='charge_sensor':p.error('--bridge requires --case charge_sensor')
W=WORKSPACE;out=W/'log/energy_validation'/a.case;out.mkdir(parents=True,exist_ok=True)
os.environ['GZ_PARTITION']='usv_energy_'+a.case+'_'+str(os.getpid())
(out/'partition.txt').write_text(os.environ['GZ_PARTITION'])
os.environ.setdefault('GZ_IP','127.0.0.1')
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_v_pb2 import Double_V
model=E.parse(W/'src/usv_description/models/usv_azimuth_trial.sdf');config=model.find(".//plugin[@name='usv::AzimuthPropulsion']/energy")
if a.case=='low_soc':config.find('initial_soc').text='0.020005'
if a.case=='fuel_out':config.find('initial_fuel_l').text='1.0009'
modelpath=out/'model.sdf';model.write(modelpath,encoding='utf-8',xml_declaration=True)
world=E.parse(W/'src/usv_simulation/worlds/usv_hull_trial.sdf');world.find('.//include/uri').text=str(modelpath);worldpath=out/'world.sdf';world.write(worldpath,encoding='utf-8',xml_declaration=True)
rows=[];sensors=[];motion=[];n=Node()
n.subscribe(Double_V,'/usv/energy/truth',lambda m:rows.append(list(m.data)))
n.subscribe(Double_V,'/usv/energy/sensors',lambda m:sensors.append(list(m.data)))
n.subscribe(Double_V,'/usv/propulsion/state',lambda m:motion.append(list(m.data)))
t=g=c=ss=None
log=(out/'launch.log').open('w');proc=subprocess.Popen(['bash',str(tool_path('run_hull_trial.sh')),'world:='+str(worldpath)],cwd=W,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
bridge_proc=None;bridge_log=None
try:
 start=time.monotonic()
 while time.monotonic()-start<85 and proc.poll() is None:
  sim=rows[-1][0] if rows else 0
  if sim>=40:break
  if not rows:time.sleep(.04);continue
  if a.bridge and bridge_proc is None:
   bridge_log=(out/'bridge_validation.log').open('w');bridge_proc=subprocess.Popen(['python3',str(tool_path('validate_energy_bridge.py'))],stdout=bridge_log,stderr=subprocess.STDOUT)
  if t is None:
   t=n.advertise('/usv/propulsion/throttle',Double);g=n.advertise('/usv/energy/generator_enable',Boolean);c=n.advertise('/usv/energy/cooling_ok',Boolean);ss=n.advertise('/usv/energy/sensors_ok',Boolean)
  throttle=.8 if a.case=='drive' and 5<sim<30 or a.case=='hybrid' and sim>20 or a.case=='low_soc' else 0
  t.publish(Double(data=throttle));g.publish(Boolean(data=a.case in ['hybrid','charge_sensor','fuel_out']))
  c.publish(Boolean(data=not(a.case=='hybrid' and sim>=34)));ss.publish(Boolean(data=not(a.case=='charge_sensor' and 24<=sim<32)))
  time.sleep(.04)
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=6)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close()
 if bridge_proc is not None:
  try:bridge_proc.wait(timeout=10)
  except subprocess.TimeoutExpired:bridge_proc.terminate();bridge_proc.wait(timeout=5)
  bridge_log.close()
d=np.array(rows);se=np.array(sensors);mo=np.array(motion)
for name,data,fields in [('truth',d,TRUTH),('sensors',se,SENSORS),('motion',mo,[])]:np.savetxt(out/(name+'.csv'),data,delimiter=',',header=','.join(fields),comments='')
r={'case':a.case,'passed':False,'samples':len(d)}
if len(d) and len(se) and len(mo) and d[-1,0]>=39:
 idx={v:i for i,v in enumerate(TRUTH)}
 def col(k):return d[:,idx[k]]
 checks={'finite':bool(np.isfinite(d).all() and np.isfinite(mo).all()),'dc_balance':bool(np.max(abs(col('balance_w')))<1e-6),'charger_limit':bool(np.max(col('charger_dc_w')/col('voltage_v'))<=25.0001),'motor_limit':bool(np.max(col('motor_w'))<=6000.001),'state_bounds':bool(np.min(col('soc'))>=0 and np.max(col('soc'))<=1 and np.min(col('fuel_l'))>=0),'valid_sensor_received':bool(np.any(se[:,2]==1))}
 if a.case=='drive':checks.update(consumption=bool(d[-1,1]<d[0,1]-.0005),no_fuel_burn=bool(d[-1,5]==d[0,5]),boat_moves=bool(np.max(mo[:,10])>.5))
 if a.case=='hybrid':
  running=d[(d[:,0]>26)&(d[:,0]<32)]
  checks.update(generator_runs=bool(len(running) and np.all(running[:,11]==3)),charger_not_infinite=bool(len(running) and np.min(running[:,3])>10),fuel_consumed=bool(d[-1,5]<d[0,5]),cooling_stops=bool(np.all(d[d[:,0]>36,11]==4) and np.max(d[d[:,0]>36,9])==0))
 if a.case=='charge_sensor':
  failed=se[(se[:,0]>26)&(se[:,0]<31)];recovered=se[se[:,0]>35]
  checks.update(charges=bool(d[-1,1]>d[0,1]),sensor_dropout=bool(len(failed) and np.all(failed[:,2]==0) and np.min(failed[:,8])>1),sensor_recovers=bool(len(recovered) and np.all(recovered[:,2]==1)))
 if a.case=='low_soc':checks.update(propulsion_cut=bool(np.all(d[d[:,0]>5,13]==1) and np.max(d[d[:,0]>5,7])==0),boat_not_driven=bool(np.max(abs(mo[:,10]))<.02),low_soc_alarm=bool(np.all(col('low_soc')==1)))
 if a.case=='fuel_out':checks.update(pickup_stops=bool(np.all(d[d[:,0]>35,11]==4) and np.max(d[d[:,0]>35,9])==0),fuel_alarm=bool(np.all(col('low_fuel')==1)),reserve_retained=bool(d[-1,5]>=1-1e-10))
 r.update(passed=all(checks.values()),checks=checks,final_soc=float(d[-1,1]),soc_change=float(d[-1,1]-d[0,1]),fuel_used_l=float(d[0,5]-d[-1,5]),max_motor_w=float(np.max(col('motor_w'))),max_charger_w=float(np.max(col('charger_dc_w'))),max_balance_error_w=float(np.max(abs(col('balance_w')))),final_generator_state=int(d[-1,11]),final_sim_s=float(d[-1,0]))
if a.bridge:
 bridge_result=json.loads((out/'bridge_result.json').read_text()) if (out/'bridge_result.json').exists() else {'passed':False}
 r['bridge_passed']=bridge_result['passed'];r['passed']=r['passed'] and r['bridge_passed']
(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
