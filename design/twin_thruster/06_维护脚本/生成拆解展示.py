"""Read final assembly; create a clearly separated illustration only. Never modify assembly."""
import bpy,bmesh,os,json
from pathlib import Path
from mathutils import Vector as V,Matrix
p=Path(__file__).resolve().parents[1];s=bpy.context.scene;dg=bpy.context.evaluated_depsgraph_get()
# Bake visible evaluated geometry, so existing boolean cutters and parents cannot affect the display pose.
visible=[o for o in s.objects if o.type in ('MESH','CURVE','FONT') and not o.hide_render and not any(c.hide_render for c in o.users_collection)]
col=bpy.data.collections.new('拆解展示 · 非装配姿态');s.collection.children.link(col);made=[]
for o in visible:
 e=o.evaluated_get(dg);me=bpy.data.meshes.new_from_object(e,depsgraph=dg);n=bpy.data.objects.new(o.name+' · 展示',me);col.objects.link(n);n.matrix_world=o.matrix_world.copy();made.append((o.name,n))
for o in list(s.objects):
 if o not in [n for _,n in made] and o.type not in ('CAMERA','LIGHT'):o.hide_render=True;o.hide_set(True)
shift=V((0,1.7,1.25))
for name,o in made:
 if name=='C方案船体':
  upper=o.copy();upper.data=o.data.copy();col.objects.link(upper);upper.name='展示 · 船壳上部与甲板移开'
  for obj,up in [(o,False),(upper,True)]:
   bm=bmesh.new();bm.from_mesh(obj.data);bmesh.ops.transform(bm,matrix=obj.matrix_world,verts=bm.verts)
   bmesh.ops.bisect_plane(bm,geom=list(bm.verts)+list(bm.edges)+list(bm.faces),dist=.00001,plane_co=(0,0,.84),plane_no=(0,0,1),clear_inner=up,clear_outer=not up)
   # Near side opened for illustration, not a new hull design.
   if not up:
    faces=[f for f in bm.faces if f.calc_center_median().y<-.65 and f.calc_center_median().z>.15]
    bmesh.ops.delete(bm,geom=faces,context='FACES')
   bm.to_mesh(obj.data);bm.free();obj.matrix_world=Matrix.Identity(4)
  upper.location+=shift
 else:
  bb=[o.matrix_world@V(v) for v in o.bound_box];c=sum(bb,V())/8
  decknames=['检修盖','扣手','铰链','闭合压缩','过中心','拉紧钩','护舷','系缆','走带','甲板防滑','围缘']
  if min(v.z for v in bb)>.86 or (c.z>.80 and any(k in name for k in decknames)):
   o.location+=shift
s.render.engine='CYCLES';s.cycles.samples=32;s.cycles.use_denoising=True;s.render.resolution_x=1800;s.render.resolution_y=1300;s.render.resolution_percentage=100;s.render.use_border=False
s.camera.data.type='ORTHO';s.camera.data.ortho_scale=9.3;s.camera.location=(7,-10,8);aim=V((-.15,.5,1));s.camera.rotation_euler=(aim-s.camera.location).to_track_quat('-Z','Y').to_euler()
s['展示说明']='甲板上部整体移开、近侧舷板局部剖去，仅为看内部；不是可制造的整体拆装方案。所有尺寸/重量以最终总装为准。'
bpy.context.preferences.filepaths.save_version=0;bpy.ops.wm.save_as_mainfile(filepath=str(p/'01_模型/无人船_拆解展示.blend'),compress=True)
s.render.filepath=str(p/'02_图片/整船拆解展示.png');bpy.ops.render.render(write_still=True);print('EXPLODED_DONE',flush=True);os._exit(0)
