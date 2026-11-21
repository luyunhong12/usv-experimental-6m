"""Read saved assembly and independently inspect the displayed cable and kinematics."""
import bpy,math,json,os
from pathlib import Path
from mathutils import Vector
P=Path(__file__).resolve().parents[1];bpy.ops.wm.open_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'))
cfg=json.loads((P/'数据/可调参数.json').read_text())
s=bpy.context.scene;ctrl=bpy.data.objects['V2_CONTROL · 推力与转角'];pivot=bpy.data.objects['V2_AZIMUTH · 中央垂直转向轴'];wire=bpy.data.objects['中央推进恒长活动线束'];sun=bpy.data.objects['输入太阳轮'];rows=[]
for frame in [1,15,30,45,60,75,90,105,120]:
 s.frame_set(frame);bpy.context.view_layer.update();a=ctrl['steering_deg'];dg=bpy.context.evaluated_depsgraph_get();e=wire.evaluated_get(dg)
 mesh=e.to_mesh();count=len(wire.data.splines[0].points);assert len(mesh.vertices)%count==0
 ring_size=len(mesh.vertices)//count
 pts=[sum((mesh.vertices[j].co for j in range(i*ring_size,(i+1)*ring_size)),Vector())/ring_size for i in range(count)]
 e.to_mesh_clear()
 L=sum((v-u).length for u,v in zip(pts,pts[1:]));expected=Vector((-3.624-.14*math.sin(math.radians(a)),.14*math.cos(math.radians(a)),.70))
 assert (pts[-1]-expected).length<1e-4,('cable endpoint',frame,list(pts[-1]),list(expected))
 assert abs(L-cfg['cable_path_length_m'])<1e-4,('cable length',frame,L)
 assert abs(math.degrees(pivot.rotation_euler.z)-a)<1e-4
 assert abs(math.degrees(sun.rotation_euler.z)-(1+cfg['ring_teeth']/cfg['sun_teeth'])*a)<1e-4
 rows.append({'frame':frame,'azimuth_deg':a,'evaluated_cable_length_m':L,'endpoint_error_m':(pts[-1]-expected).length})
assert not bpy.data.collections['V2细化 · 可拆防护罩'].hide_render
(P/'数据/最终概念模型回读检查.json').write_text(json.dumps({'status':'pass','poses':rows,'cover_saved_closed':True},ensure_ascii=False,indent=2))
print('FINAL_READBACK_PASS',flush=True);os._exit(0)
