"""Evaluate contact trajectories; a sphere may roll on a sloping inverted keel."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

def evaluate(rows,cases,hull):
 result={}
 for idx,(name,rot,xy,lo,hi) in enumerate(cases):
  a=np.array(rows[name]);ok=len(a)>100
  z=float(a[-1,2]) if len(a) else None
  spread=float(np.ptp(a[-50:,2])) if len(a)>50 else None
  entry={'samples':len(a),'final_z_m':z,'last_50_z_range_m':spread,'expected_contact_surface_range_m':[lo,hi],'passed':bool(ok and lo-.03<z<hi+.05 and spread<.01)}
  if name=='bottom' and ok:
   # Fixed body transformed by known test pose. Probe radius is 0.025 m.
   local=(a[-50:]-[idx*20.,0,0])@Rotation.from_euler('xyz',rot).as_matrix()
   mesh=trimesh.Trimesh(hull['vertices'],hull['faces'],process=False)
   distance=trimesh.proximity.closest_point_naive(mesh,local)[1]
   entry['criterion']='Rolling contact: last 50 samples sphere-centre distance to hull within radius +/- 5 mm; not rest on keel ridge'
   entry['surface_distance_min_max_m']=[float(distance.min()),float(distance.max())]
   entry['passed']=bool(np.all(np.abs(distance-.025)<.005))
  result[name]=entry
 return result
