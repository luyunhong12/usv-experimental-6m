#!/usr/bin/env python3
"""Bounded, observable startup check for the real ROS control chain."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,json,time,os
from pathlib import Path
import rclpy
from std_msgs.msg import String
from readiness import ReadinessWindow
p=argparse.ArgumentParser();p.add_argument('--timeout',type=float,default=60);p.add_argument('--boat',default='usv');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
rclpy.init();n=rclpy.create_node('usv_wait_ready');gate=ReadinessWindow();received=[];ready=False

def cb(msg):
 global ready
 h=json.loads(msg.data)
 if h.get('gz_partition')!=os.environ.get('GZ_PARTITION','default'):return
 received.append(h);ready=gate.update(h,time.monotonic())
n.create_subscription(String,f'/{a.boat}/control/health',cb,10);start=time.monotonic()
try:
 while time.monotonic()-start<a.timeout:
  rclpy.spin_once(n,timeout_sec=.1)
  if ready and gate.fresh(time.monotonic()):break
  gate.fresh(time.monotonic())
finally:n.destroy_node();rclpy.shutdown()
result={'ready':bool(ready and gate.fresh(time.monotonic())),'wall_s':time.monotonic()-start,'samples':len(received),'last_health':received[-1] if received else None,'reason':'ready' if ready else 'timeout waiting for fresh energy, actuators, physics, IMU and GPS'}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);raise SystemExit(0 if result['ready'] else 1)
