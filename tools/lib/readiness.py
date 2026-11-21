"""A continuous readiness window; stale/missing health must restart the window."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
class ReadinessWindow:
 def __init__(self,hold_s=1.):self.hold_s=hold_s;self.start=None;self.sim_start=None;self.last_received=None;self.last_health=None
 def update(self,health,wall):
  self.fresh(wall)
  self.last_received=wall;self.last_health=health
  if not health.get('ready'):
   self.start=None;self.sim_start=None;return False
  if self.start is None:self.start=wall;self.sim_start=health['sim_s']
  if health['sim_s']<self.sim_start:self.start=wall;self.sim_start=health['sim_s']
  return wall-self.start>=self.hold_s and health['sim_s']-self.sim_start>=.3
 def fresh(self,wall):
  ok=self.last_received is not None and wall-self.last_received<1.
  if not ok:self.start=None;self.sim_start=None
  return ok
