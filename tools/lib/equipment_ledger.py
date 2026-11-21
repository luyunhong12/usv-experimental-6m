"""Authoritative dry-component aggregation and auxiliary electrical load budget."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import json,math
from pathlib import Path
import numpy as np
from mass_properties import aggregate
ROOT=WORKSPACE;CFG=ROOT/'src/usv_description/config'
def load():return json.loads((CFG/'component_mass_ledger.json').read_text())
def rows(profile='baseline',mass_case='nominal'):
 if profile not in ['baseline','speed15_concept'] or mass_case not in ['nominal','light','heavy']:raise ValueError('Unknown ledger scenario')
 data=load();items=[];ids=set();scale=json.loads((CFG/'profiles/speed15_concept.json').read_text())['propeller_geometry_scale']
 origins=json.loads((ROOT/'../datasets/reference/boat_export.json').read_text())['origins']
 for row in data['parts']:
  if row['id'] in ids:raise ValueError('Duplicate component id')
  ids.add(row['id'])
  if row['link'] not in ['hull','azimuth','propeller'] or not row['profiles'] or not set(row['profiles'])<=set(['baseline','speed15_concept']):raise ValueError('Unknown link/profile in ledger')
  if row['power']['class'] not in ['passive','auxiliary','energy_storage','propulsion_dynamic','steering_dynamic','generator_output_dynamic','starter_storage_dynamic','energy_conversion_dynamic']:raise ValueError('Unknown power accounting class')
  if profile not in row['profiles']:continue
  r=dict(row);nominal=r['mass_kg'];lo,hi=r['mass_range_kg']
  if not 0<lo<=nominal<=hi:raise ValueError('Invalid uncertainty range')
  r['mass_kg']={'nominal':nominal,'light':lo,'heavy':hi}[mass_case]
  I=np.array(r['inertia_at_cg_kg_m2'])*r['mass_kg']/nominal
  if profile=='speed15_concept' and r['link']=='propeller':
   r['mass_kg']*=scale**3;I*=scale**5;r['cg_m']=(np.array(origins['propeller'])+scale*(np.array(r['cg_m'])-origins['propeller'])).tolist()
  r['inertia_at_cg_kg_m2']=I.tolist();items.append(r)
 return items
def dry_links(profile='baseline',mass_case='nominal'):
 items=rows(profile,mass_case);out={}
 for link in ['hull','azimuth','propeller']:
  p=aggregate([r for r in items if r['link']==link]);out[link]={'mass_kg':p['mass_kg'],'cg_m':p['cg_m'],'inertia_kg_m2':p['inertia_at_cg_kg_m2']}
 return out
def auxiliary_w(mode='normal',profile='baseline'):
 total=0.
 for r in rows(profile):
  p=r['power']
  if p['class']!='auxiliary':continue
  idle,rated,duty=p['idle_w'],p['rated_w'],p['duty'][mode]
  if not 0<=idle<=rated or not 0<=duty<=1:raise ValueError('Invalid electrical duty')
  total+=idle+(rated-idle)*duty
 if not math.isfinite(total):raise ValueError('Nonfinite auxiliary budget')
 return total
