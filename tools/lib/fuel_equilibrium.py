"""Independent static reference for fuel-linked hull and attitude-dependent liquid COM."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import json
from pathlib import Path
import numpy as np
from scipy.optimize import root
from scipy.spatial.transform import Rotation
from fuel_mass import configuration,tank_properties,combine,W
cfg=json.loads((W/'src/usv_description/config/trial_load.json').read_text());orig=json.loads((W/'../datasets/reference/boat_export.json').read_text())['origins'];tank=configuration();triangles=np.loadtxt(W/'src/usv_description/config/hull_triangles.txt').reshape(-1,3,3)
def displaced(q):
 z,roll,pitch=q;R=Rotation.from_euler('xyz',[roll,pitch,0]).as_matrix();tri=triangles@R.T+[0,0,z];volume=0;first=np.zeros(3)
 for t in tri:
  poly=[]
  for a,b in zip(t,np.roll(t,-1,axis=0)):
   if a[2]<=0:poly.append(a)
   if (a[2]<=0)!=(b[2]<=0):poly.append(a+(b-a)*(-a[2])/(b[2]-a[2]))
  for j in range(1,len(poly)-1):
   a,b,c=poly[0],poly[j],poly[j+1];dv=np.dot(a,np.cross(b,c))/6;volume+=dv;first+=dv*(a+b+c)/4
 return volume,first/volume,R
others=[]
for name in ['azimuth','propeller']:
 others.append(dict(mass_kg=cfg[name+'_mass_kg'],cg_m=(np.array(orig[name])+cfg[name+'_cg_m']).tolist(),inertia_kg_m2=cfg[name+'_inertia_matrix_kg_m2']))
def properties(litres,R):
 hull=combine(tank['dry_hull'],tank_properties(tank,litres,R.T@np.array([0.,0.,1.])));ship=hull
 for p in others:ship=combine(ship,p)
 return hull,ship
def solve(litres):
 def residual(q):
  v,cb,R=displaced(q);h,ship=properties(litres,R);cg=R@np.array(ship['cg_m'])+[0,0,q[0]]
  return [v-ship['mass_kg']/cfg['fluid_density_kg_m3'],cb[0]-cg[0],cb[1]-cg[1]]
 sol=root(residual,[-.239,.027,-.003]);assert sol.success and np.linalg.norm(residual(sol.x))<1e-8
 R=Rotation.from_euler('xyz',[sol.x[1],sol.x[2],0]).as_matrix();hull,ship=properties(litres,R)
 return dict(fuel_l=litres,pose_z_roll_pitch=sol.x.tolist(),hull=hull,ship=ship,displacement_m3=ship['mass_kg']/cfg['fluid_density_kg_m3'])
if __name__=='__main__':
 print(json.dumps({str(l):solve(l) for l in [60,30,6,1,0]},indent=2))
