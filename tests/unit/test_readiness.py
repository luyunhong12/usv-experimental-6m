
# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import unittest
from readiness import ReadinessWindow
class TestReadiness(unittest.TestCase):
 def test_window(self):
  g=ReadinessWindow();self.assertFalse(g.update({'ready':True,'sim_s':1},0))
  self.assertFalse(g.update({'ready':True,'sim_s':1.5},.5));self.assertTrue(g.update({'ready':True,'sim_s':2},1.1))
 def test_missing_and_stale(self):
  g=ReadinessWindow();g.update({'ready':True,'sim_s':1},0)
  self.assertFalse(g.update({'ready':True,'sim_s':4},3));self.assertFalse(g.update({'ready':False,'sim_s':4.2},3.2))
 def test_paused_and_reset(self):
  g=ReadinessWindow();g.update({'ready':True,'sim_s':10},0)
  g.update({'ready':True,'sim_s':10},.5);self.assertFalse(g.update({'ready':True,'sim_s':10},1.1))
  self.assertFalse(g.update({'ready':True,'sim_s':0},1.2))
if __name__=='__main__':unittest.main()
