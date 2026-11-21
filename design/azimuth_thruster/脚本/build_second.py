"""Run with Blender background mode; derives a separate concept assembly from original."""
import bpy, math, json, hashlib
from pathlib import Path
from mathutils import Vector, Matrix
P=Path(__file__).resolve().parents[1]; ROOT=P.parent
source=ROOT/'twin_thruster/01_模型/无人船_最终总装.blend'
bpy.ops.wm.open_mainfile(filepath=str(source))
s=bpy.context.scene
# Preserve the source file; retired assemblies stay hidden in this derivative for traceability.
retire=['Navy 6.0 Evo 2024 标准桨参考位置','Navy 6.0 Evo 短轴双机外形试排','短轴双机安装与抬机检查','艉部双推进装配','自拟侧出与检修支撑','宽根固定叉与可视防脱','最近点测量标记 · 十字随动／圆圈为宽差艉板点','出水检视 · 厂图出线口','第一线卡样架测点 · 非安装件']
oldparts=list(bpy.data.collections['Navy 6.0 Evo 短轴双机外形试排'].objects)
for name in retire:
 c=bpy.data.collections.get(name)
 if c:
  c.hide_render=True;c.hide_viewport=True
for o in s.objects:
 if any(k in o.name for k in ['抬机销·','机A点后固定段']):o.hide_render=True;o.hide_set(True)
col=bpy.data.collections.new('第二船体 · 中央单机整体转向');s.collection.children.link(col)
def link(o,name):
 o.name=name
 for c in list(o.users_collection):c.objects.unlink(o)
 col.objects.link(o);return o
def mat(name,color):
 m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);return m
orange=mat('V2 转向机构 橙',(1,.31,.045));steel=mat('V2 安装结构',(0.24,.32,.39));dark=mat('V2 执行器',(0.045,.065,.09))
def box(name,loc,dim,material):
 bpy.ops.mesh.primitive_cube_add(size=1,location=loc);o=link(bpy.context.object,name);o.dimensions=dim;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);o.data.materials.append(material);return o
def cyl(name,loc,r,depth,material):
 bpy.ops.mesh.primitive_cylinder_add(vertices=48,radius=r,depth=depth,location=loc);o=link(bpy.context.object,name);o.data.materials.append(material);return o
def parent(o,p):
 m=o.matrix_world.copy();o.parent=p;o.matrix_world=m
ctrl=bpy.data.objects.new('V2_CONTROL · 推力与转角',None);col.objects.link(ctrl);ctrl['steering_deg']=0.;ctrl['thrust_N']=0.
ctrl.id_properties_ui('steering_deg').update(min=-35,max=35,description='Geometric azimuth, positive toward port; NOT ArduPilot steering sign')
ctrl['说明']='推力仅为接口占位，不驱动物理仿真；转角正值指向左舷，ArduPilot转向符号须按艉部力矩和实际执行器方向核验。'
pivot=bpy.data.objects.new('V2_AZIMUTH · 中央垂直转向轴',None);col.objects.link(pivot);pivot.location=(-3.624,0,.45)
bpy.context.view_layer.update()
newparts=[]
for o in oldparts:
 if o.name.startswith('左') and o.type=='MESH' and not o.hide_render:
  n=o.copy();n.data=o.data.copy();col.objects.link(n);n.parent=None;n.matrix_world=Matrix.Translation((-.25,-.6,0))@o.matrix_world;n.name='V2概念单机 · '+o.name[1:];n.hide_render=False;n.hide_set(False);parent(n,pivot);newparts.append(n)
# Structure is a concept envelope, not certified scantlings or a commercial conversion kit.
box('V2 中央艉板外背板',(-3.025,0,.48),(.05,.50,.66),steel)
box('V2 中央艉板内背板',(-2.97,0,.48),(.04,.50,.66),steel)
for y in [-.20,.20]:
 box('V2 艉伸安装梁',(-3.31,y,.42),(.57,.06,.12),steel)
 box('V2 舱内纵向补强概念',(-2.74,y,.36),(.44,.05,.30),steel)
box('V2 轴承横梁',(-3.624,0,.42),(.18,.46,.12),steel)
cyl('V2 固定轴承座',(-3.624,0,.48),.095,.18,steel)
shaft=cyl('V2 回转轴',(-3.624,0,.58),.043,.54,orange);parent(shaft,pivot)
box('V2 执行器上托梁',(-3.32,0,.83),(.64,.13,.065),steel)
cyl('V2 转向执行器 外形占位',(-3.624,0,.90),.105,.13,dark)
encoder=cyl('V2 角度反馈盖',(-3.624,0,.98),.055,.03,orange)
for y in [-.17,.17]:
 for z in [.25,.70]:
  bolt=cyl('V2 艉板紧固件占位',(-3.059,y,z),.014,.025,orange);bolt.rotation_euler[1]=math.pi/2
# Driver enables direct property editing and an inspectable steering animation.
d=pivot.driver_add('rotation_euler',2).driver;d.expression='angle * 0.017453292519943295';v=d.variables.new();v.name='angle';v.targets[0].id=ctrl;v.targets[0].data_path='["steering_deg"]'
s.frame_start=1;s.frame_end=120
for frame,angle in [(1,0),(30,-35),(60,0),(90,35),(120,0)]:
 ctrl['steering_deg']=angle;ctrl.keyframe_insert(data_path='["steering_deg"]',frame=frame)
# Geometric sweep sampled at every degree. A conservative box separates moving unit from transom.
checks=[]
for angle in range(-35,36):
 ctrl.animation_data_clear() if angle==-35 else None
 ctrl['steering_deg']=angle;ctrl.update_tag();bpy.context.view_layer.update()
 pts=[o.matrix_world@Vector(v) for o in newparts for v in o.bound_box]
 checks.append({'angle_deg':angle,'forwardmost_x_m':max(p.x for p in pts),'outer_half_width_m':max(abs(p.y) for p in pts)})
assert max(r['forwardmost_x_m'] for r in checks)<-3.05,'Moving unit reaches transom envelope'
for frame,angle in [(1,0),(30,-35),(60,0),(90,35),(120,0)]:
 ctrl['steering_deg']=angle;ctrl.keyframe_insert(data_path='["steering_deg"]',frame=frame)
s.frame_set(1)
s['第二船体说明']='中央单机整体转向概念装配；原6m船壳沿用，中央安装结构新增。非Navy原厂可转向改装认证，功率/重量/强度/线缆/水动力待核定。'
s['旧预算失效']='原双机1273.9kg预算不能用于第二船体；旧双机电气留作待改造参考，未实现单机接线。'
s.render.engine='BLENDER_WORKBENCH';s.render.resolution_x=1500;s.render.resolution_y=1000;s.render.resolution_percentage=100;s.render.use_border=False
sh=s.display.shading;sh.light='STUDIO';sh.color_type='MATERIAL';sh.show_shadows=True;sh.show_cavity=True;sh.background_type='WORLD';s.world.color=(.18,.18,.18)
cam=s.camera;cam.data.type='ORTHO'
def camera(loc,aim,scale):
 cam.location=loc;cam.rotation_euler=(Vector(aim)-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=scale
camera((-8,-9,6),(0,0,.3),8.4)
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'),compress=True)
print('SECOND_HULL_BASE_COMPLETE',flush=True)
import os
os._exit(0)
