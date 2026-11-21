#!/usr/bin/env python3
"""Independent physical metrics for direct-command manoeuvres, no fitted limits."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np

def speed_rate(x):
    h=np.unwrap(x[:,14]);return float(np.polyfit(x[:,0]-x[0,0],h,1)[0])

def path_distance(x):
    return float(np.linalg.norm(np.diff(x[:,12:14],axis=0),axis=1).sum())

def stage_metrics(x):
    tail=x[x[:,0]>x[-1,0]-15]
    return {'duration_s':float(x[-1,0]-x[0,0]),'tail_surge_m_s':float(tail[:,10].mean()),
            'tail_speed_std_m_s':float(tail[:,10].std()),'tail_yaw_deg_s':math.degrees(speed_rate(tail)),
            'heading_change_deg':float(np.degrees(np.unwrap(x[:,14])[-1]-x[0,14])),
            'distance_m':path_distance(x)}

def turning_metrics(x):
    tail=x[x[:,0]>x[-1,0]-20];delta=np.unwrap(tail[:,14]);theta=abs(delta[-1]-delta[0])
    radius=path_distance(tail)/theta if theta>1e-6 else None
    q=x[:,12:14]-x[0,12:14];h=x[0,14]
    along=q[:,0]*np.cos(h)+q[:,1]*np.sin(h);across=-q[:,0]*np.sin(h)+q[:,1]*np.cos(h)
    dh=np.abs(np.unwrap(x[:,14])-x[0,14]);out={'radius_m':radius}
    for angle,label in [(90,'advance_90_m'),(180,'tactical_diameter_m')]:
        hit=np.flatnonzero(dh>=np.radians(angle))
        out[label]=float(along[hit[0]] if angle==90 else abs(across[hit[0]])) if len(hit) else None
    return out

def stop_metrics(x,reverse=False):
    if reverse:
        hit=np.flatnonzero(x[:,10]<=0)
    else:
        horizontal=np.linalg.norm(np.gradient(x[:,12:14],x[:,0],axis=0),axis=1)
        hit=np.array([i for i in np.flatnonzero(horizontal<.1) if x[-1,0]-x[i,0]>=1 and np.all(horizontal[(x[:,0]>=x[i,0])&(x[:,0]<=x[i,0]+1)]<.1)],dtype=int)
    if not len(hit):return {'stop_time_s':None,'stop_distance_m':None}
    i=hit[0]
    return {'stop_time_s':float(x[i,0]-x[0,0]),'stop_distance_m':path_distance(x[:i+1]),
            'surge_at_detection_m_s':float(x[i,10]),'sway_at_detection_m_s':float(x[i,11]),
            'definition':'first nonpositive surge (remaining lateral motion reported)' if reverse else 'horizontal speed <0.1 m/s sustained for 1 s'}

def assess_case(p,c):
    result=json.loads((p/'result.json').read_text());stages=json.loads((p/'stages.json').read_text())
    x=np.loadtxt(p/'propulsion_state.csv',delimiter=',',ndmin=2);a=np.loadtxt(p/'actuators_state.csv',delimiter=',',ndmin=2)
    health=json.loads((p/'health.json').read_text());L=c['length_reference_m']
    checks={'runtime':result['passed'],'direct_only':bool(health) and all(h['command_mode'] in ['direct','idle'] and h.get('heading_reference_rad') is None for h in health),
            'finite':bool(np.isfinite(x).all() and np.isfinite(a).all()),
            'attitude':bool(np.max(abs(x[:,15:17]))<np.radians(c['max_roll_pitch_deg'])),
            'servo_rate':bool(np.max(abs(a[:,2]))<=c['max_servo_rate_rad_s'])}
    metrics={};overshoots=[];reference=None
    for i,s in enumerate(stages):
        v=x[(x[:,0]>=s['start_sim'])&(x[:,0]<=s['end_sim'])]
        if len(v)<5:checks[s['name']+'_samples']=False;continue
        m=stage_metrics(v);name=s['name'];metrics[name]=m
        at=a[(a[:,0]>s['end_sim']-min(3,(s['end_sim']-s['start_sim'])/3))&(a[:,0]<=s['end_sim'])]
        # Short heading-triggered executions may reverse before reaching angle;
        # rate is still checked, but steady angle checks belong to held stages.
        if 'until_heading_deg' not in s:
            checks[name+'_servo']=bool(len(at) and np.max(abs(np.degrees(at[:,1])-s.get('steering_deg',0)))<c['max_servo_error_deg'])
        if name in ['neutral','recover']:
            checks[name+'_yaw']=abs(m['tail_yaw_deg_s'])<c['max_neutral_yaw_deg_s']
            checks[name+'_steady_speed']=m['tail_speed_std_m_s']<c['max_tail_speed_std_m_s']
            checks[name+'_forward']=m['tail_surge_m_s']>.3
        if name in ['turn','astern_turn']:
            m.update(turning_metrics(v));direction=-np.sign(s['steering_deg'])*np.sign(s['throttle'])
            checks[name+'_direction']=m['heading_change_deg']*direction>90
            checks[name+'_radius']=m['radius_m'] is not None and c['min_turn_radius_lengths']*L<m['radius_m']<c['max_turn_radius_lengths']*L
            checks[name+'_steady_speed']=m['tail_speed_std_m_s']<c['max_tail_speed_std_m_s']
            checks[name+'_half_circle']=m['tactical_diameter_m'] is not None
        if name in ['coast','reverse_brake']:
            m.update(stop_metrics(v,name=='reverse_brake'))
            key='max_reverse_brake_stop_distance_lengths' if name=='reverse_brake' else 'max_coast_stop_distance_lengths'
            checks[name+'_stop']=m['stop_distance_m'] is not None and m['stop_distance_m']<c[key]*L
        if name=='reverse':
            checks[name+'_astern']=m['tail_surge_m_s']<-.2
            checks[name+'_bounded_yaw']=abs(m['tail_yaw_deg_s'])<c['max_reverse_yaw_deg_s']
        if 'until_heading_deg' in s:
            checks[name+'_trigger']=bool(s.get('heading_triggered'))
            if reference is None:reference=v[0,14]
            if i and 'until_heading_deg' in stages[i-1]:
                previous=stages[i-1]['until_heading_deg'];h=np.degrees(np.unwrap(v[:,14])-reference)
                overshoot=float(np.max(h*np.sign(previous))-abs(previous));overshoots.append(max(0,overshoot))
    if overshoots:
        metrics['zigzag_overshoots_deg']=overshoots
        checks['zigzag_overshoots']=len(overshoots)==3 and max(overshoots)<c['max_zigzag_overshoot_deg']
    return {'passed':all(checks.values()),'checks':checks,'metrics':metrics}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--criteria',type=Path,required=True);ap.add_argument('--spec',type=Path,required=True);args=ap.parse_args()
    c=json.loads(args.criteria.read_text());spec=json.loads(args.spec.read_text());cases={}
    for name in spec:
        try:cases[name]=assess_case(args.output/name,c)
        except Exception as e:cases[name]={'passed':False,'failure':str(e)}
    comparisons={}
    for label in ['055','100']:
        try:
            l=cases['turn_'+label+'_left']['metrics']['turn']['radius_m'];r=cases['turn_'+label+'_right']['metrics']['turn']['radius_m']
            difference=abs(l-r)/((l+r)/2)
            comparisons['turn_'+label]={'relative_radius_difference':difference,'passed':difference<c['max_port_starboard_radius_relative_difference']}
        except (KeyError,TypeError):comparisons['turn_'+label]={'passed':False}
    try:
        u=[cases['neutral_'+k]['metrics']['neutral']['tail_surge_m_s'] for k in ['035','065','100']]
        comparisons['speed_monotonic']={'passed':u[0]<u[1]<u[2],'speeds_m_s':u}
    except KeyError:comparisons['speed_monotonic']={'passed':False}
    out={'passed':all(v['passed'] for v in [*cases.values(),*comparisons.values()]),'criteria':c,'criteria_sha256':hashlib.sha256(args.criteria.read_bytes()).hexdigest(),'spec_sha256':hashlib.sha256(args.spec.read_bytes()).hexdigest(),'cases':cases,'comparisons':comparisons}
    (args.output/'maneuver_assessment.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v['passed'] for k,v in cases.items()}));raise SystemExit(0 if out['passed'] else 1)
if __name__=='__main__':main()
