#!/usr/bin/env python3
"""Separate proven navigation stages from an explicitly failed mixed-link failsafe attribution.
Never converts the original full-run failure to a pass; preserve it verbatim.
"""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,json,shutil
from pathlib import Path
import numpy as np
w=WORKSPACE;p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();src=w/'log/platform_validation'/a.run;out=w/'log/platform_validation/navigation_core'
r=json.loads((src/'result.json').read_text());required=['normal_arming','mission_upload','auto_waypoints','loiter_mode','loiter_radius','rtl_mode','rtl_home','finite']
checks={k:r['checks'].get(k,False) for k in required}
d=np.loadtxt(src/'propulsion.csv',delimiter=',');e=np.loadtxt(src/'energy.csv',delimiter=',')
checks['attitude_bounded']=bool(np.max(abs(d[:,15:17]))<np.deg2rad(20));checks['power_limited']=bool(np.max(e[:,7])<=40000.01)
shutil.copytree(src,out,dirs_exist_ok=True);shutil.copy2(src/'result.json',out/'full_attempt_result.json')
r.update(passed=all(checks.values()),checks=checks,source_run=a.run,scope='Navigation stages only. Original full attempt did NOT pass GCS attribution: literal GCS-only status-text assertion failed; upstream can print Radio for the GCS bit. Preserve full_attempt_result.json and require independent gcs_isolation result.',original_full_attempt_passed=False)
(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
