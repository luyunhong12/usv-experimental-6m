#!/usr/bin/env python3

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
from pathlib import Path
import os,json
os.environ.setdefault('MPLCONFIGDIR','/tmp/usv_matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
w=WORKSPACE;o=w/'../artifacts/validation/WAMV差距完善_2026-11-20'
fig,ax=plt.subplots(1,2,figsize=(12,4));d=np.loadtxt(o/'maneuvers/assembly.csv',delimiter=',',skiprows=1);ax[0].plot(d[:,0],d[:,8]);ax[0].set(xlabel='Simulation time [s]',ylabel='Forward speed [m/s]',title='ROS forward / reverse / turns / coasting');ax[0].grid(alpha=.3)
for stage,label in [(7,'Left 15 deg'),(8,'Right 15 deg'),(9,'Coast')]:
 x=d[d[:,1]==stage];ax[1].plot(x[:,14]+480,x[:,15]-180,label=label)
ax[1].set(xlabel='East offset [m]',ylabel='North offset [m]',title='Measured track');ax[1].axis('equal');ax[1].legend();fig.tight_layout();fig.savefig(o/'maneuvers.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for folder,label in [('wave3','3 s'),('wave5','5 s'),('wave8','8 s')]:
 d=np.loadtxt(o/folder/'wave_response.csv',delimiter=',',skiprows=1);axes[0].plot(d[:,0],d[:,6]-d[d[:,0]>25,6].mean(),label=label);axes[1].plot(d[:,0],np.rad2deg(d[:,7]));axes[2].plot(d[:,0],np.rad2deg(d[:,8]))
for ax,label in zip(axes,['Heave [m]','Roll [deg]','Pitch [deg]']):ax.set_ylabel(label);ax.grid(alpha=.3)
axes[0].legend();axes[2].set_xlabel('Simulation time [s]');fig.suptitle('Measured response across VRX wave periods (gain 0.3)');fig.tight_layout();fig.savefig(o/'period_response.png',dpi=160);plt.close(fig)
base=json.loads((o/'wave5/result.json').read_text())['metrics'];fine=json.loads((o/'wave_dt2/result.json').read_text())['metrics'];errors={k:abs(fine[k]-base[k])/base[k] for k in ['heave_peak_to_peak_m','roll_peak_to_peak_deg','pitch_peak_to_peak_deg']};(o/'step_convergence.json').write_text(json.dumps({'passed':max(errors.values())<.01,'relative_peak_to_peak_difference':errors,'scope':'4 vs 2 ms, 65 s diagonal-wave trial; not universal convergence proof'},indent=2))
cases=['calm','wave3','wave5','wave8','wave_dt2','added0','added05','added2','maneuvers','speed','speed_enu','sensors','stationkeeping','wayfinding','navigation'];checks={c:json.loads((o/c/'result.json').read_text())['passed'] for c in cases};checks['step_convergence']=max(errors.values())<.01
manifest=json.loads((w/'src/usv_description/config/pbr_visuals.json').read_text());export=json.loads((w/'../datasets/reference/boat_export.json').read_text());checks['pbr_triangles_preserved']=all(sum(v['triangles'] for v in manifest['groups'][g])==export['groups'][g]['triangles'] for g in manifest['groups']);checks['pbr_assets_present']=all(Path(v[k]).is_file() for group in manifest['groups'].values() for v in group for k in ['mesh','albedo','normal_map','roughness_map']);checks['gate_geometry_matches_score']=json.loads((o/'navigation/geometry_score_check.json').read_text())['passed'];checks['eight_numerical_tests']='8 tests, 0 errors, 0 failures, 0 skipped' in (o/'unit_test_results.log').read_text();checks['maneuver_metrics']=json.loads((o/'maneuver_metrics.json').read_text())['passed']
summary={'passed':all(checks.values()),'checks':checks,'limitations':['Engineering parameters, not measured vessel calibration','Task lifecycle/scoring interfaces verified; autonomous task solving is separate','Added mass is conservative constant diagonal force-level approximation','Underwater rendering, CFD wake, radiation memory and slamming remain outside this change']};(o/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2));raise SystemExit(0 if summary['passed'] else 1)
