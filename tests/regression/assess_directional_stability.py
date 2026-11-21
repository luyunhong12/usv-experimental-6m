#!/usr/bin/env python3
"""Physical acceptance independent of the generic runtime checks.
Thresholds are engineering regression bounds, not real-vessel certification.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();o=a.output
spec=json.loads((Path(__file__).resolve().parents[1]/'scenarios/directional_stability.json').read_text())
results={}
for name in spec:
 d=o/name
 if not (d/'result.json').exists():results[name]={'passed':False,'failure':'Missing run result'};continue
 run=json.loads((d/'result.json').read_text());stages=json.loads((d/'stages.json').read_text())
 data=np.loadtxt(d/'propulsion_state.csv',delimiter=',');act=np.loadtxt(d/'actuators_state.csv',delimiter=',');metrics={};checks={'runtime':run['passed']}
 # stages.json stores stage definitions with absolute simulation bounds.
 if spec[name].get('kind') in ['neutral','recovery']:
  stage=next(s for s in stages if s['name']=='neutral');end=stage['end_sim'];start=stage['start_sim']
  tail=data[(data[:,0]>end-15)&(data[:,0]<end)];servo=act[(act[:,0]>end-15)&(act[:,0]<end)]
  rate=float(np.polyfit(tail[:,0],np.unwrap(tail[:,14]),1)[0]);slip=float(np.degrees(np.arctan2(tail[:,11],tail[:,10])).mean())
  metrics={'tail_yaw_rate_rad_s':rate,'tail_speed_m_s':float(tail[:,10].mean()),'tail_sideslip_deg':slip,'tail_servo_max_deg':float(np.degrees(abs(servo[:,1])).max()),'tail_command_min_max':[float(tail[:,1].min()),float(tail[:,1].max())]}
  checks.update(passive_rate_bounded=abs(rate)<.003,sideslip_bounded=abs(slip)<1.,speed_retained=metrics['tail_speed_m_s']>3.8,actual_zero_rudder=metrics['tail_servo_max_deg']<.1,full_direct_throttle=min(tail[:,1])>.999)
  health=json.loads((d/'health.json').read_text());checks['no_heading_control']=all(h['command_mode'] in ['idle','direct'] for h in health)
  app=np.loadtxt(d/'hydrodynamics_appendages.csv',delimiter=',');checks['appendages_dissipative']=float(app[:,-1].min())>=-1e-8
  if spec[name].get('kind')=='recovery':
   kick=next(s for s in stages if s['name']=='kick');kd=data[(data[:,0]>kick['start_sim'])&(data[:,0]<kick['end_sim'])]
   kickrate=float(np.polyfit(kd[:,0],np.unwrap(kd[:,14]),1)[0]);metrics['kick_yaw_rate_rad_s']=kickrate
   checks['perturbation_was_effective']=abs(kickrate)>.015;checks['yaw_rate_recovered']=abs(rate)<abs(kickrate)*.1
 checks={k:bool(v) for k,v in checks.items()}
 results[name]={'passed':all(checks.values()),'checks':checks,'metrics':metrics}
nom=results.get('nominal',{}).get('metrics',{});comparisons={}
if nom:
 for n in ['step_1ms','step_2ms','step_4ms','heading_90']:
  m=results[n].get('metrics',{})
  comparisons[n]=bool(m and abs(m['tail_yaw_rate_rad_s']-nom['tail_yaw_rate_rad_s'])<.0002 and abs(m['tail_speed_m_s']-nom['tail_speed_m_s'])<.03)
 for n in ['kick_left','kick_right']:
  m=results[n].get('metrics',{});comparisons[n+'_returns_to_same_trim']=bool(m and abs(m['tail_yaw_rate_rad_s']-nom['tail_yaw_rate_rad_s'])<.0003)
result={'passed':all(x['passed'] for x in results.values()) and all(comparisons.values()),'criteria':{'tail_window_s':15,'max_abs_yaw_rate_rad_s':.003,'max_abs_sideslip_deg':1.,'min_forward_speed_m_s':3.8,'max_actual_rudder_deg':.1,'step_and_heading_rate_difference_rad_s':.0002,'step_and_heading_speed_difference_m_s':.03},'cases':results,'comparisons':comparisons,'limitation':'Bounds allow small physical trim drift; zero commanded rudder does not guarantee mathematically zero yaw rate. No heading controller is used in neutral/recovery cases.'}
(o/'physical_assessment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
