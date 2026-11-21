#!/usr/bin/env python3

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import sys,copy,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
w=WORKSPACE;sys.path.insert(0,str(w/'src/usv_ardupilot/scripts'))
from json_time import StepLimiter
base={'timestamp':1.,'position':[0,0,0],'velocity':[0,0,0],'quaternion':[1,0,0,0],'imu':{'gyro':[0,0,0],'accel_body':[0,0,-9.81]},'battery':{'voltage':50.,'current':0.},'no_time_sync':True}
s=StepLimiter();assert s.next(base)==base;target=copy.deepcopy(base);target['timestamp']=2.;target['position']=[10,0,0];target['quaternion']=[0,0,0,1];states=[]
while s.last['timestamp']<2.-1e-9:states.append(s.next(target))
assert len(states)==50 and s.max_delta<=.020000001 and np.allclose(states[-1]['position'],[10,0,0])
assert all(abs(np.linalg.norm(x['quaternion'])-1)<1e-9 for x in states)
assert s.next(target) is None
try:s.next(base);raise AssertionError('Reset not rejected')
except RuntimeError:pass
out=w/'log/platform_validation/clock_steps';out.mkdir(parents=True,exist_ok=True);r={'passed':True,'checks':{'one_second_gap_split_into_50_bounded_steps':True,'quaternions_normalized':True,'duplicate_suppressed':True,'clock_reset_rejected':True},'max_time_step_s':s.max_delta};(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
