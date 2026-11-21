"""Aggregate explicitly supplied component properties; never infer from appearance."""

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

def aggregate(parts):
    masses=[];centers=[];inertias=[]
    for p in parts:
        m=p.get('mass_kg')
        if m is None:
            rho=p.get('density_kg_m3')
            volume=p.get('material_volume_m3')
            if volume is None and p.get('plate_area_m2') is not None and p.get('thickness_m') is not None:
                volume=p['plate_area_m2']*p['thickness_m']
            if rho is None or volume is None:raise ValueError(f"Missing mass basis: {p['name']}")
            if rho<=0 or volume<=0:raise ValueError('Density and material volume must be positive')
            m=rho*volume
        c=np.asarray(p.get('cg_m'),dtype=float);i=np.asarray(p.get('inertia_at_cg_kg_m2'),dtype=float)
        if not np.isfinite(m) or m<=0 or c.shape!=(3,) or not np.isfinite(c).all():raise ValueError('Invalid mass / CG')
        if i.shape!=(3,3) or not np.isfinite(i).all() or not np.allclose(i,i.T):raise ValueError('Missing or asymmetric inertia')
        ev=np.linalg.eigvalsh(i)
        if ev.min()<=0 or 2*ev.max()>ev.sum()+1e-9:raise ValueError('Nonphysical component inertia')
        # CG and inertia must already be expressed in common model FLU axes.
        masses.append(m);centers.append(c);inertias.append(i)
    if not masses:raise ValueError('No components')
    mass=sum(masses);cg=sum(m*c for m,c in zip(masses,centers))/mass
    inertia=np.zeros((3,3))
    for m,c,i in zip(masses,centers,inertias):
        d=c-cg;inertia+=i+m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    return {'mass_kg':mass,'cg_m':cg.tolist(),'inertia_at_cg_kg_m2':inertia.tolist()}

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('input');parser.add_argument('--output',required=True);args=parser.parse_args()
    data=json.loads(Path(args.input).read_text())
    if data.get('coverage_complete') is not True:raise SystemExit('Component coverage is incomplete; refusing to publish whole-boat mass properties')
    result=aggregate(data['parts']);result['status']=data.get('status','UNVALIDATED');Path(args.output).write_text(json.dumps(result,indent=2))
