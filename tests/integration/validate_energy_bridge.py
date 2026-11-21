#!/usr/bin/env python3
"""Run alongside charge_sensor test; verify ROS conversion and invalid readings."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import os,time,subprocess,json,math
from pathlib import Path
W=WORKSPACE;out=W/'log/energy_validation/charge_sensor'
os.environ['GZ_PARTITION']=(out/'partition.txt').read_text().strip();os.environ['GZ_IP']='127.0.0.1';os.environ['ROS_DOMAIN_ID']='177'
import rclpy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
rclpy.init();node=rclpy.create_node('test_energy_bridge');battery=[];fuel=[];diagnostic=[]
node.create_subscription(BatteryState,'/usv/battery',lambda m:battery.append(m),10)
node.create_subscription(Float64,'/usv/fuel_l',lambda m:fuel.append(m.data),10)
node.create_subscription(DiagnosticArray,'/usv/energy/diagnostics',lambda m:diagnostic.extend(m.status),10)
log=(out/'ros_bridge.log').open('w');proc=subprocess.Popen(['python3',str(tool_path('energy_console.py')),'--ros','--duration','36'],stdout=log,stderr=subprocess.STDOUT)
try:
 start=time.monotonic()
 while time.monotonic()-start<38:rclpy.spin_once(node,timeout_sec=.2)
finally:
 if proc.poll() is None:proc.terminate()
 proc.wait(timeout=5);log.close();node.destroy_node();rclpy.shutdown()
valid=[m for m in battery if math.isfinite(m.voltage)]
checks={'clean_exit':proc.returncode==0,'battery_received':len(valid)>2,'fuel_received':len(fuel)>2,'charging_sign':any(m.current>.1 for m in valid),'soc_range':all(0<=m.percentage<=1 for m in valid),'invalid_nan':any(not math.isfinite(m.voltage) for m in battery),'invalid_diagnostic':any(m.level==DiagnosticStatus.ERROR for m in diagnostic),'valid_after_dropout':any(math.isfinite(m.voltage) and m.header.stamp.sec>=33 for m in battery)}
r={'passed':all(checks.values()),'checks':checks,'battery_messages':len(battery),'bridge_exit':proc.returncode}
(out/'bridge_result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
