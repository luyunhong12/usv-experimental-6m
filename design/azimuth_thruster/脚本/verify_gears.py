import bpy,math,json,os
from pathlib import Path
from mathutils import Vector
from mathutils.bvhtree import BVHTree
P=Path(__file__).resolve().parents[1];bpy.ops.wm.open_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'))
s=bpy.context.scene;ctrl=bpy.data.objects['V2_CONTROL · 推力与转角'];ctrl.animation_data_clear();sun=bpy.data.objects['输入太阳轮'];ring=bpy.data.objects['固定内齿圈'];planets=[bpy.data.objects['自转行星轮_'+str(i+1)] for i in range(3)]
def tree(o,dg):
 e=o.evaluated_get(dg);m=e.to_mesh();t=BVHTree.FromPolygons([e.matrix_world@v.co for v in m.vertices],[tuple(p.vertices) for p in m.polygons]);e.to_mesh_clear();return t
rows=[]
limit=int(json.loads((P/'数据/可调参数.json').read_text())['steering_limit_deg'])
for a in range(-limit,limit+1):
 ctrl['steering_deg']=float(a);ctrl.update_tag();bpy.context.view_layer.update();dg=bpy.context.evaluated_depsgraph_get();st=tree(sun,dg);rt=tree(ring,dg)
 pairs=[]
 for p in planets:
  t=tree(p,dg)
  if st.overlap(t):pairs.append(['sun',p.name])
  if rt.overlap(t):pairs.append(['ring',p.name])
 rows.append({'angle':a,'intersections':pairs})
(P/'数据/齿轮啮合回读检查.json').write_text(json.dumps({'samples':rows,'scope':'sampled involute mesh surface intersections, 71 poses; no load/contact stress validation'},ensure_ascii=False,indent=2))
print('COLLISION_POSES',sum(bool(r['intersections']) for r in rows),flush=True)
print('PAIRS',sorted(set(tuple(p) for r in rows for p in r['intersections'])),flush=True)
assert not any(r['intersections'] for r in rows), 'Gear intersections found; review parameters'
os._exit(0)
