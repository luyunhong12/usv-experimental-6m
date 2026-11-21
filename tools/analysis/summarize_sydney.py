#!/usr/bin/env python3
"""Archive measured Sydney manoeuvres; do not label failures as passes."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
from pathlib import Path
import json
import shutil
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=WORKSPACE
out=root/'../artifacts/validation/悉尼场景验证'
out.mkdir(parents=True,exist_ok=True)
results=[]
fig,axes=plt.subplots(1,2,figsize=(12,5))
for case in ['forward','reverse','left','right','reversal']:
    source=root/'log/sydney_validation'/case
    result=json.loads((source/'result.json').read_text())
    if not result['passed']:raise RuntimeError(f'{case} did not pass')
    shutil.copytree(source,out/case,dirs_exist_ok=True)
    results.append(result)
    data=np.loadtxt(source/'telemetry.csv',delimiter=',',skiprows=1)
    axes[0].plot(data[:,12]+480,data[:,13]-180,label=case)
    axes[1].plot(data[:,0],data[:,10],label=case)
axes[0].set(xlabel='East displacement (m)',ylabel='North displacement (m)',title='Sydney: vessel trajectories')
axes[0].axis('equal')
axes[1].set(xlabel='Simulation time (s)',ylabel='Body forward speed (m/s)',title='Actual simulated speed')
for ax in axes:ax.grid(True);ax.legend()
fig.tight_layout();fig.savefig(out/'manoeuvres.png',dpi=160);plt.close(fig)
shutil.copy(root/'../datasets/reference/sydney_scene.json',out/'scene.json')
shutil.copy(root/'../dependencies/vendor/fuel/SOURCE.json',out/'asset_source.json')
if (root/'log/sydney_validation/gui').exists():
    shutil.copytree(root/'log/sydney_validation/gui',out/'gui',dirs_exist_ok=True)
    shutil.copy(root/'log/sydney_gui.log',out/'gui/launch_snapshot.log')
paths=['../tools/modeling/prepare_sydney.py','../ros_ws/src/usv_bringup/scripts/run_sydney.sh','../tests/integration/validate_propulsion.py',
       'src/usv_simulation/worlds/usv_sydney.sdf','src/usv_description/models/usv_azimuth_trial.sdf',
       'src/usv_description/config/propulsion.json','src/usv_description/config/trial_load.json',
       'src/usv_energy/config/energy.json',
       'src/usv_description/models/coast_waves_calm/model.sdf']
hashes={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in paths}
(out/'results.json').write_text(json.dumps({'cases':results,'source_sha256':hashes},indent=2)+'\n')
table=['| 工况 | 24–27 秒平均纵向速度 m/s | 约 27 秒航向变化 ° | 结果 |',
       '|---|---:|---:|---|']
labels={'forward':'前进','reverse':'倒车','left':'左转','right':'右转','reversal':'前进切倒车'}
for r in results:
    table.append(f"| {labels[r['case']]} | {r['driven_speed_m_s']:.4f} | {r['driven_yaw_deg']:.3f} | 通过 |")
(out/'结果表.md').write_text('\n'.join(table)+'\n')
print('\n'.join(table))
