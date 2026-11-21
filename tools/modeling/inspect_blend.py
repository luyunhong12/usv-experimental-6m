
# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import bpy,json
from pathlib import Path
from mathutils import Vector
root=PROJECT_ROOT
bpy.ops.wm.open_mainfile(filepath=str(root/'design/azimuth_thruster/模型/第二船体_中央单机整体转向.blend'))
bpy.context.scene.frame_set(1)
rows=[]
for o in bpy.context.scene.objects:
 if o.type not in ('MESH','CURVE','EMPTY'):continue
 rows.append({'name':o.name,'type':o.type,'visible':o.visible_get(),'hide_render':o.hide_render,'collections':[c.name for c in o.users_collection],'parent':o.parent.name if o.parent else None,'location':list(o.matrix_world.translation),'dimensions':list(o.dimensions),'props':{k:str(o[k]) for k in o.keys()}})
(root/'ros_ws/../datasets/reference/blender_inventory.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
print('objects',len(rows),'visible',sum(r['visible'] and not r['hide_render'] for r in rows))

import os
os._exit(0)
