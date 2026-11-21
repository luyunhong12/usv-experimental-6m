#!/usr/bin/env python3
"""Archive only completed acceptance runs; preserve exploratory runs separately in log."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,json,shutil,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
w=WORKSPACE;p=argparse.ArgumentParser();p.add_argument('--navigation',required=True);a=p.parse_args()
source=w/'log/platform_validation';out=w/'../artifacts/validation/十五公里平台验证';out.mkdir(parents=True,exist_ok=True)
names=['nominal','drag_low','drag_high','weather','command_loss','sensor_loss','payload','endurance','design','energy_duration','geometry','gcs_isolation','clock_steps','manual_control',a.navigation]
r={}
for name in names:
 folder=source/name;result=json.loads((folder/'result.json').read_text())
 if not result.get('passed'):raise RuntimeError('Not passed: '+name)
 if name==a.navigation:
  if result.get('final_phase')!='complete' or not result['checks'].get('auto_path_corridor'):raise RuntimeError('Navigation requires complete end-to-end run and route audit')
  if not json.loads((folder/'flight_log_audit.json').read_text())['passed']:raise RuntimeError('Flight log audit failed')
  timing=json.loads((folder/'json_frames.jsonl.summary.json').read_text())
  if timing['max_time_step_s']>.020001:raise RuntimeError('Unbounded flight-control clock steps')
 target='navigation' if name==a.navigation else name
 if target=='navigation' and (out/target).exists():shutil.rmtree(out/target)
 shutil.copytree(folder,out/target,dirs_exist_ok=True,ignore=shutil.ignore_patterns('energy_duration'))
 r[target]=result
for name in ['unit_tests.log','test_results.log','launcher_smoke.log','launcher_smoke.json','launcher_ardupilot_smoke.log','launcher_ardupilot_smoke.json','build.log','build_ardupilot.log','software_versions.json','physics_linkage.txt']:
 if (source/name).exists():shutil.copy2(source/name,out/name)
shutil.copy2(w/'../datasets/reference/platform15_propeller_sweep_raw.json',out/'geometry/sweep_raw.json')
for name in ['usv_description','usv_energy','usv_ardupilot']:shutil.copytree(w/'src'/name/'config',out/'config'/name,dirs_exist_ok=True)
(out/'summary.json').write_text(json.dumps(r,indent=2,ensure_ascii=False))
fig,ax=plt.subplots(2,2,figsize=(11,8),layout='constrained')
for name in ['nominal','drag_low','drag_high','weather','payload']:
 d=np.loadtxt(out/name/'propulsion_state.csv',delimiter=',');ax[0,0].plot(d[:,0],d[:,10]*3.6,label=name)
ax[0,0].axhline(15,color='gray',ls='--');ax[0,0].set(xlabel='Simulation time (s)',ylabel='Forward speed (km/h)');ax[0,0].legend(fontsize=8)
d=np.loadtxt(out/'navigation/track.csv',delimiter=',');h=r['navigation']['home_xy'];
for mode,label in [(10,'AUTO'),(5,'LOITER'),(11,'RTL'),(15,'GUIDED'),(4,'HOLD')]:
 v=d[d[:,4]==mode];ax[0,1].scatter(v[:,1]-h[0],v[:,2]-h[1],s=1,label=label)
ax[0,1].plot([0,70,70,20],[0,0,40,40],'k--',lw=.8,label='Mission route');ax[0,1].scatter([0],[0],marker='*',label='Home');ax[0,1].set(xlabel='East from home (m)',ylabel='North from home (m)',title='ArduRover AUTO / LOITER / RTL');ax[0,1].axis('equal');ax[0,1].legend(loc='upper right',fontsize=7,ncol=2)
for name in ['command_loss','sensor_loss']:
 d=np.loadtxt(out/name/'propulsion_state.csv',delimiter=',');ax[1,0].plot(d[:,0],d[:,1],label=name)
ax[1,0].axvspan(20,30,color='gray',alpha=.15,label='Sensor bridge paused');ax[1,0].set(xlabel='Simulation time (s)',ylabel='Actual accepted throttle command');ax[1,0].legend(fontsize=8)
d=np.genfromtxt(out/'energy_duration/energy.csv',delimiter=',',names=True)
for gen,label in [(0,'Generator off'),(1,'Generator on')]:
 v=d[d['generator']==gen];ax[1,1].plot(v['time_s']/60,v['motor_w']/1000,label=label)
ax[1,1].set(xlabel='Energy-model time (min)',ylabel='Available requested motor input (kW)',title='Energy-only calculation, not voyage range');ax[1,1].legend(fontsize=8)
for item in ax.flat:item.grid(alpha=.25)
fig.suptitle('15 km/h engineering simulation platform: acceptance evidence');fig.savefig(out/'overview.png',dpi=160);plt.close(fig)
lines=['# 平台验收结果','', '| 工况 | 结果 | 关键数据 |','|---|---|---|']
for name,result in r.items():
 value=f"末段速度 {result['tail_speed_km_h']:.3f} km/h" if 'tail_speed_km_h' in result else '见 result.json'
 lines.append(f'| {name} | 通过 | {value} |')
lines+=['','全部数据属于工程仿真/几何计算，不是实船测量。标称/低阻/配重要求 15±0.5 km/h；高阻和环境压力测试记录真实降速，不能视作全部海况保持 15 km/h。','', '动力学 CSV 字段延续 ../六项整改验证/csv_schemas.json；navigation/track.csv 为 time,x,y,body_u,ArduPilot_mode，navigation/propulsion.csv 同 propulsion_state，energy.csv 同 energy_truth；energy_duration/energy.csv 自带表头。navigation 的各事件时间和任务点详见 result.json。', '', 'energy_duration 为同一 C++ 能源模型的独立两小时积分，不是两小时整船运动。endurance 是 180 s 六自由度持续转弯测试。geometry 是 5112 个离散外表面扫碰，不是制造认证。', '', '按 source_sha256.json 校验代码、按 data_sha256.json 校验本归档。运行快照含原始绝对路径，仅用于溯源，迁移须重新生成。navigation 包含同一次自动航点、定点、返航和 GCS 失联验收，并记录实际航线偏差。GCS 失联另由 gcs_isolation 在遥控链路保持正常时独立交叉检查。失败及中断探索保留在 log/platform_validation/navigation*，不计为通过。']
(out/'README.md').write_text('\n'.join(lines)+'\n')
files=[]
for folder in [PROJECT_ROOT/'tools',*w.glob('src/usv_*')]:
 for f in folder.rglob('*'):
  if f.is_file() and f.suffix in {'.py','.cc','.hh','.json','.xml','.txt','.sh','.md','.parm'} and '__pycache__' not in f.parts:files.append(f)
(out/'source_sha256.json').write_text(json.dumps({str(f.resolve().relative_to(PROJECT_ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(files)},indent=2,ensure_ascii=False))
(out/'data_sha256.json').write_text(json.dumps({str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(out.rglob('*')) if f.is_file() and f.name!='data_sha256.json'},indent=2,ensure_ascii=False))
print('Archived',len(r),'passed groups to',out)
