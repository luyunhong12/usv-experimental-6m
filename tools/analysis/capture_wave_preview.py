#!/usr/bin/env python3
"""Record native Gazebo frames and contemporaneous wave telemetry."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import os,time,json,subprocess,argparse
from pathlib import Path
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('--partition',default='usv_refined_final');a=p.parse_args()
w=WORKSPACE;out=w/'../artifacts/validation/船体三项优化_2026-11-20/preview';out.mkdir(parents=True,exist_ok=True)
os.environ.update(GZ_PARTITION=a.partition,GZ_IP='127.0.0.1')
from gz.transport13 import Node
from gz.msgs10.double_v_pb2 import Double_V
node=Node();samples=[];node.subscribe(Double_V,'/usv/wave_response/state',lambda m:samples.append(list(m.data)))
start=time.monotonic()
while not samples and time.monotonic()-start<10:time.sleep(.1)
if not samples:raise RuntimeError('No live wave telemetry')
frames=[]
for i in range(24):
 f=out/f'frame_{i:02d}.png';before=samples[-1][0]
 subprocess.run(['python3',str(tool_path('capture_gazebo_view.py')),'--partition',a.partition,'--output',str(f)],check=True,stdout=subprocess.DEVNULL)
 frames.append({'file':f.name,'sim_before':before,'sim_after':samples[-1][0],'state_after':samples[-1]});time.sleep(.2)
(out/'frames.json').write_text(json.dumps(frames,indent=2));(out/'telemetry.json').write_text(json.dumps(samples))
images=[]
for f in frames:
 im=Image.open(out/f['file']).convert('RGB');im.thumbnail((760,760));images.append(im.copy())
durations=[max(100,min(3000,int(1000*(b['sim_after']-a['sim_after'])))) for a,b in zip(frames,frames[1:])];durations.append(durations[-1])
images[0].save(out/'waves.gif',save_all=True,append_images=images[1:],duration=durations,loop=0)
print(json.dumps({'frames':len(frames),'sim_seconds':frames[-1]['sim_after']-frames[0]['sim_after'],'output':str(out/'waves.gif')}))
