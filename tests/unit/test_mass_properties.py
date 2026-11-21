
# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import unittest
import numpy as np
from mass_properties import aggregate
class MassProperties(unittest.TestCase):
 def test_parallel_axis(self):
  p=[{'name':str(x),'mass_kg':2,'cg_m':[x,0,0],'inertia_at_cg_kg_m2':np.eye(3).tolist()} for x in [-1,1]]
  r=aggregate(p);self.assertEqual(r['mass_kg'],4);np.testing.assert_allclose(r['cg_m'],[0,0,0]);np.testing.assert_allclose(r['inertia_at_cg_kg_m2'],np.diag([2,6,6]))
 def test_plate_mass(self):
  r=aggregate([{'name':'test plate','density_kg_m3':1000,'plate_area_m2':2,'thickness_m':.005,'cg_m':[0,0,0],'inertia_at_cg_kg_m2':np.eye(3).tolist()}]);self.assertEqual(r['mass_kg'],10)
 def test_missing_and_invalid(self):
  with self.assertRaises(ValueError):aggregate([{'name':'unknown'}])
  with self.assertRaises(ValueError):aggregate([{'name':'bad','mass_kg':1,'cg_m':[0,0,0],'inertia_at_cg_kg_m2':np.diag([1,1,5]).tolist()}])
if __name__=='__main__':unittest.main()
