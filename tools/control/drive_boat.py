#!/usr/bin/env python3
"""Maintain throttle heartbeat; steering is physical azimuth angle in FLU."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse, math, os, time
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--throttle',type=float,default=0)
p.add_argument('--steering-deg',type=float,default=0)
p.add_argument('--duration',type=float,default=30)
p.add_argument("--name",default="usv")
a=p.parse_args()
if not a.name.isidentifier():p.error("Invalid boat name")
prefix="/"+a.name
if not all(map(math.isfinite,[a.throttle,a.steering_deg,a.duration])) or not -1<=a.throttle<=1 or not -35<=a.steering_deg<=35 or a.duration<=0:p.error('throttle [-1,1], steering [-35,35], duration > 0 required')
os.environ.setdefault('GZ_PARTITION','usv_hull_trial')
os.environ.setdefault('GZ_IP','127.0.0.1')
from gz.transport13 import Node
from usv_control.ownership import ControlLease
lease=ControlLease(a.name,"drive_boat")
from gz.msgs10.double_pb2 import Double
n=Node();t=n.advertise(prefix+'/propulsion/throttle',Double);s=n.advertise(prefix+'/steering/angle',Double)
def send(throttle,angle):
 t.publish(Double(data=throttle));s.publish(Double(data=math.radians(angle)))
try:
 start=time.monotonic()
 while time.monotonic()-start<a.duration:
  send(a.throttle,a.steering_deg);time.sleep(.05)
except KeyboardInterrupt:pass
finally:
 for _ in range(10):send(0,0);time.sleep(.05)

lease.close()
