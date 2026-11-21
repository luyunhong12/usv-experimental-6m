"""Finish configurable concept model, not a manufacturing drawing or calibrated simulator."""
import bpy, math, json, os
from pathlib import Path
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
P=Path(__file__).resolve().parents[1];cfg=json.loads((P/'数据/可调参数.json').read_text())
N=cfg['sun_teeth'];NP=cfg['planet_teeth'];NR=cfg['ring_teeth'];m=cfg['gear_module_m'];pressure=math.radians(cfg['pressure_angle_deg']);limit=cfg['steering_limit_deg'];width=cfg['gear_face_width_m'];clearance=cfg['tooth_clearance_m']
assert all(isinstance(n,int) and n>=18 for n in (N,NP,NR))
assert NR==N+2*NP and (N+NR)%3==0
assert 0<limit<=35 and 0<m and 0<width<=.032 and 0<clearance<m/4
assert m*(NR/2+1.25)<.105 and m*(N+NP)/2<.075
ratio=1+NR/N;radius=m*(N+NP)/2
bpy.ops.wm.open_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'))
s=bpy.context.scene;s.frame_set(1);ctrl=bpy.data.objects['V2_CONTROL · 推力与转角'];pivot=bpy.data.objects['V2_AZIMUTH · 中央垂直转向轴'];cx=pivot.location.x
ctrl.animation_data_clear()
del ctrl['steering_deg'];ctrl['steering_deg']=0.0
for cname in ['V2完成 · 联动齿轮','V2完成 · 恒长活动线束']:
 old=bpy.data.collections.get(cname)
 if old:
  for o in list(old.objects):bpy.data.objects.remove(o,do_unlink=True)
  bpy.data.collections.remove(old)
def col(name):c=bpy.data.collections.new(name);s.collection.children.link(c);return c
gears=col('V2完成 · 联动齿轮');cables=col('V2完成 · 恒长活动线束');gold=bpy.data.materials['V2 减速器示意'];silver=bpy.data.materials['V2 缎面铝'];navy=bpy.data.materials['V2 深海蓝']
for o in list(bpy.data.objects):
 if any(k in o.name for k in ['齿圈包络','行星轮包络','太阳轮包络','活动线缆 · 路由示意']):o.hide_render=True;o.hide_set(True)
# Tooth flanks sampled from involute circles, roots joined by circular arcs.
# Root fillets and manufacturing relief are intentionally omitted.
def inv(r,rb):
 a=math.acos(min(1,rb/r));return math.tan(a)-a

def profile(teeth,internal=False):
 rp=m*teeth/2;rb=rp*math.cos(pressure);root=rp-m*(1 if internal else 1.25);tip=rp+m*(1.25 if internal else 1)
 start=max(root,rb);phi=inv(rp,rb);half=math.pi/(2*teeth)+(clearance/(2*rp) if internal else -clearance/(2*rp));points=[]
 def hw(r):return half+phi-inv(max(r,rb),rb)
 for j in range(teeth):
  center=2*math.pi*j/teeth
  a0=center-math.pi/teeth
  points.append((root*math.cos(a0),root*math.sin(a0)))
  a=center-hw(start);points.append((root*math.cos(a),root*math.sin(a)))
  for k in range(9):
   r=start+(tip-start)*k/8;a=center-hw(r);points.append((r*math.cos(a),r*math.sin(a)))
  for k in range(1,5):
   a=center-hw(tip)+2*hw(tip)*k/4;points.append((tip*math.cos(a),tip*math.sin(a)))
  for k in range(7,-1,-1):
   r=start+(tip-start)*k/8;a=center+hw(r);points.append((r*math.cos(a),r*math.sin(a)))
  a=center+hw(start);points.append((root*math.cos(a),root*math.sin(a)))
 return points

def gear(name,teeth,loc,internal=False):
 path=profile(teeth,internal);count=len(path);verts=[];faces=[]
 # Ring gears have a circular outer edge; external gears have a shaft bore.
 for z in [-width/2,width/2]:
  verts.extend([(x,y,z) for x,y in path])
  for x,y in path:
   a=math.atan2(y,x);r=.108 if internal else .0065
   verts.append((r*math.cos(a),r*math.sin(a),z))
 for i in range(count):
  j=(i+1)%count
  faces.extend([(i,j,2*count+j,2*count+i),(count+j,count+i,3*count+i,3*count+j),(i,count+i,count+j,j),(2*count+j,3*count+j,3*count+i,2*count+i)])
 mesh=bpy.data.meshes.new(name);mesh.from_pydata(verts,[],faces);mesh.update();o=bpy.data.objects.new(name,mesh);gears.objects.link(o);o.location=loc;o.data.materials.append(gold if internal else silver)
 # Recalculate normals, including the inner surfaces of ring gears.
 import bmesh
 bm=bmesh.new();bm.from_mesh(mesh);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(mesh);bm.free()
 o['teeth']=teeth;o['module_m']=m;o['profile']='sampled involute; no root fillet or manufacturing validation'
 return o

def angle_driver(o,expr):
 d=o.driver_add('rotation_euler',2).driver;d.expression=expr;v=d.variables.new();v.name='a';v.targets[0].id=ctrl;v.targets[0].data_path='["steering_deg"]'

ring=gear('固定内齿圈',NR,(cx,0,.916),True);ring.rotation_euler.z=math.pi/NR
# Gap center on x axis; external sun tooth center on x axis.
sun=gear('输入太阳轮',N,(cx,0,.916));angle_driver(sun,f'{ratio}*a*pi/180')
planets=[]
for i in range(3):
 a=2*math.pi*i/3;phase=math.pi/NP
 o=gear('自转行星轮_'+str(i+1),NP,(0,0,0));o.parent=pivot;o.location=(radius*math.cos(a),radius*math.sin(a),.916-pivot.location.z)
 # Relative to carrier: -Nr/Np times the output angle. Phase seats tooth in sun gap.
 angle_driver(o,f'{phase}-{NR/NP}*a*pi/180');planets.append(o)
# Move original axle markers to new configurable planet centers.
for i,o in enumerate(sorted([o for o in bpy.data.objects if o.name.startswith('行星轮轴销')],key=lambda o:o.name)):
 a=2*math.pi*i/3;world=Vector((cx+radius*math.cos(a),radius*math.sin(a),.90));o.location=pivot.matrix_world.inverted()@world
# Add visible input shaft linking motor to sun, not just floating gear envelopes.
bpy.ops.mesh.primitive_cylinder_add(vertices=40,radius=.006,depth=.065,location=(cx,0,.953));shaft=bpy.context.object;shaft.name='太阳轮输入轴'
for c in list(shaft.users_collection):c.objects.unlink(shaft)
gears.objects.link(shaft);shaft.data.materials.append(silver);angle_driver(shaft,f'{ratio}*a*pi/180')
# Distinguish propeller spin from azimuth steering.
prop=bpy.data.objects.get('V2_PROP_SPIN · 桨轴自转')
if not prop:
 hub=bpy.data.objects['V2概念单机 · 桨毂.001'];bb=[hub.matrix_world@Vector(v) for v in hub.bound_box];center=sum(bb,Vector())/8
 prop=bpy.data.objects.new('V2_PROP_SPIN · 桨轴自转',None);bpy.data.collections['第二船体 · 中央单机整体转向'].objects.link(prop);prop.parent=pivot;prop.matrix_world=Matrix.Translation(center);bpy.context.view_layer.update()
 for o in list(bpy.data.objects):
  if o.name.startswith('V2概念单机') and ('桨毂' in o.name or '曲面桨叶' in o.name):
   w=o.matrix_world.copy();o.parent=prop;o.matrix_world=w
ctrl['propeller_demo_rpm']=cfg['propeller_demo_rpm'];ctrl.id_properties_ui('propeller_demo_rpm').update(min=-300,max=300,description='Visual only: not calibrated thrust')
d=prop.driver_add('rotation_euler',0).driver;d.expression=f'(frame-1)*rpm*2*pi/(60*{s.render.fps/s.render.fps_base})';v=d.variables.new();v.name='rpm';v.targets[0].id=ctrl;v.targets[0].data_path='["propeller_demo_rpm"]'
# 65-point cubic cable. Solve outer control-point offset for constant sampled arc length.
A=Vector((-3.065,.31,.78));target=cfg['cable_path_length_m'];npts=65

def path(angle,bow):
 a=math.radians(angle);D=Vector((cx-.14*math.sin(a),.14*math.cos(a),.70));C=Vector((cx-.32*math.sin(a),.32*math.cos(a)+bow,.86));B=Vector((-3.25,.48+bow,1.04))
 return [A*(1-t)**3+B*(3*(1-t)**2*t)+C*(3*(1-t)*t*t)+D*t**3 for t in [i/(npts-1) for i in range(npts)]]
def length(points):return sum((b-a).length for a,b in zip(points,points[1:]))
def solve(a):
 assert length(path(a,0))<=target,'Increase cable_path_length_m; cannot stretch a cable'
 lo,hi=0.,1.
 assert length(path(a,hi))>=target
 for _ in range(40):
  mid=(lo+hi)/2
  if length(path(a,mid))<target:lo=mid
  else:hi=mid
 return path(a,(lo+hi)/2)
curve=bpy.data.curves.new('恒长线束曲线','CURVE');curve.dimensions='3D';curve.bevel_depth=.009;curve.bevel_resolution=3;sp=curve.splines.new('POLY');sp.points.add(npts-1)
wire=bpy.data.objects.new('中央推进恒长活动线束',curve);cables.objects.link(wire);curve.materials.append(bpy.data.materials['V2 线缆黑'])
for p,v in zip(sp.points,solve(0)):p.co=(*v,1)
wire.shape_key_add(name='Basis')
for a in range(-35,36):
 key=wire.shape_key_add(name=f'angle_{a:+d}')
 for p,v in zip(key.data,solve(a)):p.co=v
 d=key.driver_add('value').driver;d.expression=f'max(0,1-abs(a-({a})))';v=d.variables.new();v.name='a';v.targets[0].id=ctrl;v.targets[0].data_path='["steering_deg"]'
wire['note']='Constant sampled arc length at integer poses; interpolated between them. Geometric routing only, no cable stiffness/contact simulation.'
# Current module parameters are now visible inside the file.
ctrl['max_steering_deg']=limit;ctrl['planetary_ratio']=ratio
ctrl.id_properties_ui('steering_deg').update(min=-limit,max=limit)
# Clamp driver so edits outside the range do not over-rotate the model.
pivot.animation_data.drivers[0].driver.expression=f'min({limit},max(-{limit},angle))*pi/180'
ctrl.animation_data_clear()
checks=[]
for a in [i*.5 for i in range(-int(limit*2),int(limit*2)+1)]:
 ctrl['steering_deg']=a;ctrl.update_tag();bpy.context.view_layer.update()
 assert abs(sun.rotation_euler.z-ratio*math.radians(a))<1e-5
 assert abs(planets[0].rotation_euler.z-(math.pi/NP-NR/NP*math.radians(a)))<1e-5
 # Evaluate cable centerline directly from the same blended basis positions.
 low=math.floor(a);high=math.ceil(a);t=a-low;p0=solve(low);p1=solve(high);pts=[u.lerp(v,t) for u,v in zip(p0,p1)]
 L=length(pts);assert abs(L-target)<.0001
 checks.append({'angle':a,'cable_length_m':L})
for f,a in [(1,0),(30,-limit),(60,0),(90,limit),(120,0)]:ctrl['steering_deg']=float(a);ctrl.keyframe_insert(data_path='["steering_deg"]',frame=f)
s.frame_set(1)
# Shorten the output hub below the sun gear and join it to the carrier plate.
hub=bpy.data.objects['减速器输出联轴套'];hub.dimensions.z=.063
w=hub.matrix_world.copy();w.translation.z=.8505;hub.matrix_world=w
# Connecting web, 12 mm thick, joins the existing carrier and output hub.
bpy.ops.mesh.primitive_cylinder_add(vertices=64,radius=.045,depth=.012,location=(cx,0,.883));web=bpy.context.object;web.name='输出联轴套至行星架连接盘'
for c in list(web.users_collection):c.objects.unlink(web)
gears.objects.link(web);web.data.materials.append(bpy.data.materials['V2 转向机构 橙']);bpy.context.view_layer.update();w=web.matrix_world.copy();web.parent=pivot;web.matrix_world=w

# Save concept state with covers closed and armature visible in the outliner.
s['转向细化边界']='可调整概念船模：有渐开线采样齿形、固定齿圈4:1联动、近似恒长线束、桨自转与整体转向。尚非加工图/实船选型/水动力验证。'
(P/'数据/概念完成检查.json').write_text(json.dumps({'parameters':cfg,'ratio':ratio,'gear_relation_ok':True,'cable_pose_checks':checks,'limits':'No load rating, tooth stress, root fillet, tolerance, cable mechanics or whole-hull collision validation.'},ensure_ascii=False,indent=2))
cam=s.camera
cam.location=(-8,-9,6);cam.rotation_euler=(Vector((0,0,.35))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=8.3
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'),compress=True)
# Isolated gearbox inspection gives a clear view of the actual tooth geometry.
for o in s.objects:
 if o.type in ('MESH','CURVE') and o not in list(gears.objects):o.hide_render=True
for o in gears.objects:o.hide_render=False
s.render.resolution_x=1200;s.render.resolution_y=900;cam.location=(cx-.10,-.14,1.19);cam.rotation_euler=(Vector((cx,0,.916))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=.29
# Hide input shaft only in inspection to expose sun teeth.
shaft.hide_render=True
s.render.filepath=str(P/'图片/转向减速齿轮_联动检查.png');bpy.ops.render.render(write_still=True)
print('CONCEPT_COMPLETE',flush=True);os._exit(0)
