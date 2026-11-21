"""Add inspectable concept bearings, coaxial reducer and a driven cable service loop.
No commercial component sizing or manufacturing approval is implied.
"""
import bpy, math, json
from pathlib import Path
from mathutils import Vector
from mathutils.bvhtree import BVHTree
P=Path(__file__).resolve().parents[1]
bpy.ops.wm.open_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'))
s=bpy.context.scene;s.frame_set(1)
base=bpy.data.collections['第二船体 · 中央单机整体转向'];pivot=bpy.data.objects['V2_AZIMUTH · 中央垂直转向轴'];ctrl=bpy.data.objects['V2_CONTROL · 推力与转角']
for name in ['V2细化 · 传动与支撑','V2细化 · 可拆防护罩','V2细化 · 活动线缆']:
 c=bpy.data.collections.get(name)
 if c:
  for o in list(c.objects):bpy.data.objects.remove(o,do_unlink=True)
  bpy.data.collections.remove(c)
def collection(name):
 c=bpy.data.collections.new(name);s.collection.children.link(c);return c
mech=collection('V2细化 · 传动与支撑');covers=collection('V2细化 · 可拆防护罩');wires=collection('V2细化 · 活动线缆')
def mat(name,color):
 m=bpy.data.materials.get(name) or bpy.data.materials.new(name);m.diffuse_color=(*color,1);return m
silver=mat('V2 缎面铝',(.4,.49,.57));navy=mat('V2 深海蓝',(.035,.14,.23));orange=mat('V2 转向机构 橙',(1,.32,.055));black=mat('V2 线缆黑',(.025,.035,.045));gold=mat('V2 减速器示意',(.7,.46,.15))
def link(o,name,c=mech,m=silver):
 o.name=name
 for old in list(o.users_collection):old.objects.unlink(o)
 c.objects.link(o);o.data.materials.clear();o.data.materials.append(m);return o
def parent(o):
 w=o.matrix_world.copy();o.parent=pivot;o.matrix_world=w
cx=-3.624
# Closed annular mesh with an actual shaft opening.
def ring(name,z,outer,inner,height,c=mech,m=silver):
 verts=[];faces=[];n=64
 for zz,r in [(z-height/2,outer),(z+height/2,outer),(z-height/2,inner),(z+height/2,inner)]:
  verts.extend([(cx+r*math.cos(i*2*math.pi/n),r*math.sin(i*2*math.pi/n),zz) for i in range(n)])
 for i in range(n):
  j=(i+1)%n
  faces.extend([(i,j,n+j,n+i),(2*n+j,2*n+i,3*n+i,3*n+j),(n+i,n+j,3*n+j,3*n+i),(j,i,2*n+i,2*n+j)])
 mesh=bpy.data.meshes.new(name);mesh.from_pydata(verts,[],faces);mesh.update();o=bpy.data.objects.new(name,mesh);c.objects.link(o);o.data.materials.append(m)
 bevel=o.modifiers.new('边缘圆角','BEVEL');bevel.width=.001;bevel.segments=2
 return o
def cylinder(name,loc,r,h,m=silver,c=mech):
 bpy.ops.mesh.primitive_cylinder_add(vertices=48,radius=r,depth=h,location=loc);return link(bpy.context.object,name,c,m)
for name in ['V2 固定轴承座','V2 转向执行器 外形占位','V2 角度反馈盖']:
 o=bpy.data.objects.get(name)
 if o:o.hide_render=True;o.hide_set(True);o['替代']='由细化集合替代'
ring('固定轴承筒 · 通孔',.48,.095,.076,.18)
for z in [.422,.538]:
 ring('轴承外圈 · 概念',z,.076,.065,.024)
 o=ring('轴承内圈 · 概念',z,.055,.044,.024,m=orange);parent(o)
 for i in range(12):
  a=2*math.pi*i/12
  bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=8,radius=.005,location=(cx+.060*math.cos(a),.060*math.sin(a),z));link(bpy.context.object,'轴承滚动体 · 位置示意')
for z in [.392,.568]:ring('轴端密封 · 概念',z,.075,.0435,.009,m=black)
o=ring('轴肩锁紧环',.582,.064,.043,.018,m=orange);parent(o)
# Shaft passes through upper support: explicit opening in existing beam.
beam=bpy.data.objects['V2 执行器上托梁']
cut=cylinder('轴孔刀具',(cx,0,.83),.052,.25)
cut.hide_render=True;cut.hide_set(True);cut.display_type='WIRE'
mod=beam.modifiers.get('回转轴通过孔') or beam.modifiers.new('回转轴通过孔','BOOLEAN');mod.operation='DIFFERENCE';mod.object=cut
# Replace the inherited upper streamlined strut with the circular steering shaft.
# Its lower body remains attached to the existing shaft at z=0.31..0.32.
strut=bpy.data.objects['V2概念单机 · 流线支腿']
bpy.ops.mesh.primitive_cube_add(size=1,location=(cx,0,.62));trim=link(bpy.context.object,'上部支腿修形刀具');trim.dimensions=(1,1,.60);bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
trim.hide_render=True;trim.hide_set(True);trim.display_type='WIRE'
mod=strut.modifiers.get('上部改圆轴') or strut.modifiers.new('上部改圆轴','BOOLEAN');mod.operation='DIFFERENCE';mod.object=trim
cross=bpy.data.objects['V2 轴承横梁'];hole=cylinder('横梁轴承孔刀具',(cx,0,.42),.096,.30);hole.hide_render=True;hole.hide_set(True);hole.display_type='WIRE'
mod=cross.modifiers.get('轴承安装通孔') or cross.modifiers.new('轴承安装通孔','BOOLEAN');mod.operation='DIFFERENCE';mod.object=hole
ring('轴承筒固定法兰',.491,.124,.076,.022)

# Reducer is a kinematic packaging sketch: coaxial output hub and one planetary stage.
ring('减速器固定底法兰',.853,.12,.045,.024)
ring('减速器齿圈包络 · 未定模数',.91,.108,.088,.078,m=gold)
out=cylinder('减速器输出联轴套',(cx,0,.862),.036,.085,orange);parent(out)
carrier=ring('减速器输出行星架',.883,.087,.037,.012,m=orange);parent(carrier)
for i in range(3):
 a=i*2*math.pi/3
 o=cylinder('行星轮包络 · 未生成齿形',(cx+.060*math.cos(a),.060*math.sin(a),.916),.026,.032,gold);parent(o)
 o=cylinder('行星轮轴销',(cx+.060*math.cos(a),.060*math.sin(a),.90),.006,.05,silver);parent(o)
cylinder('输入太阳轮包络 · 未生成齿形',(cx,0,.916),.034,.032,gold)
cylinder('转向电机包络',(cx,0,1.009),.071,.10,navy)
cylinder('角度编码器包络',(cx,0,1.074),.040,.025,orange)
ring('可拆减速器防护罩',.941,.124,.112,.17,c=covers,m=navy)
cylinder('可拆防护顶盖',(cx,0,1.099),.125,.018,navy,covers)
# Top cover includes the motor/encoder enclosure; extend shell to meet top cover.
ring('可拆电机防护罩',1.058,.124,.112,.064,c=covers,m=navy)
for i in range(4):
 a=math.pi/4+i*math.pi/2
 cylinder('防护罩紧固件',(cx+.102*math.cos(a),.102*math.sin(a),1.111),.006,.008,silver,covers)
fixed_gland=cylinder('中央线缆固定端密封接头',(-3.035,.31,.78),.022,.08,black);fixed_gland.rotation_euler[1]=math.pi/2
moving_gland=cylinder('机罩活动端线缆接头',(cx,.12,.70),.019,.065,black);moving_gland.rotation_euler[0]=math.pi/2;parent(moving_gland)

# Service-loop guide: cubic curve with control points tied to the steering angle.
curve=bpy.data.curves.new('活动线缆路径','CURVE');curve.dimensions='3D';curve.bevel_depth=.009;curve.bevel_resolution=3
sp=curve.splines.new('POLY');N=32;sp.points.add(N)
wire=bpy.data.objects.new('中央推进活动线缆 · 路由示意',curve);wires.objects.link(wire);curve.materials.append(black)
A=(-3.065,.31,.78);B=(-3.25,.48,1.04);C=(cx,.32,.86);D=(cx,.14,.70)
for i,p in enumerate(sp.points):
 t=i/N;weights=[(1-t)**3,3*(1-t)**2*t,3*(1-t)*t*t,t**3]
 p.co=(0,0,0,1)
 for axis in range(3):
  fixed=weights[0]*A[axis]+weights[1]*B[axis]
  if axis==0:expr=f'{fixed}+{weights[2]}*({cx}-{C[1]}*sin(a*pi/180))+{weights[3]}*({cx}-{D[1]}*sin(a*pi/180))'
  elif axis==1:expr=f'{fixed}+{weights[2]*C[1]+weights[3]*D[1]}*cos(a*pi/180)'
  else:expr=str(fixed+weights[2]*C[2]+weights[3]*D[2])
  driver=p.driver_add('co',axis).driver;driver.expression=expr;v=driver.variables.new();v.name='a';v.targets[0].id=ctrl;v.targets[0].data_path='["steering_deg"]'
wire['说明']='活动线束路由示意；两端随固定座/转动机罩关联。不是恒长电缆力学模型；长度及最小弯曲半径待实际线缆选型。'
# Save a geometric interference audit: moving propulsion envelope vs fixed mounting frame.
moving=[o for o in base.objects if o.name.startswith('V2概念单机')]
fixed=[o for o in base.objects if o.type=='MESH' and not o.hide_render and not o.parent and any(k in o.name for k in ['梁','外背板'])]
def tree(o,dg):
 e=o.evaluated_get(dg);mesh=e.to_mesh();verts=[e.matrix_world@v.co for v in mesh.vertices];faces=[tuple(p.vertices) for p in mesh.polygons];b=BVHTree.FromPolygons(verts,faces);e.to_mesh_clear();return b
# Sample by frames spanning the animation as well as exact requested angle values.
ctrl.animation_data_clear();report=[]
for angle in range(-35,36):
 ctrl['steering_deg']=angle;ctrl.update_tag();bpy.context.view_layer.update();dg=bpy.context.evaluated_depsgraph_get()
 overlaps=[]
 ft=[(o.name,tree(o,dg)) for o in fixed]
 for o in moving:
  mt=tree(o,dg)
  for name,bt in ft:
   if mt.overlap(bt):overlaps.append([o.name,name])
 pts=[Vector(p.co[:3]) for p in sp.points]
 report.append({'angle_deg':angle,'surface_intersections':overlaps})
for f,a in [(1,0),(30,-35),(60,0),(90,35),(120,0)]:ctrl['steering_deg']=a;ctrl.keyframe_insert(data_path='["steering_deg"]',frame=f)
s.frame_set(1)
(P/'数据/转向细化检查.json').write_text(json.dumps({'scope':'71 poses; moving propulsion mesh surfaces against fixed external mount beams and outer backplate. Does not detect complete containment; excludes intentional shaft/bearing/gear contact, cables, hull internals. Not full assembly validation.','samples':report},ensure_ascii=False,indent=2))
s['转向细化边界']='已加空心轴承座、双轴承示意、轴封、联轴套、行星减速器包络、可拆防护罩、活动线束路由；未定模数/减速比/力矩/材料/标准件，不能用于加工。'
bpy.context.preferences.filepaths.save_version=0
cam=s.camera;cam.data.type='ORTHO'
def camera(loc,aim,scale):cam.location=loc;cam.rotation_euler=(Vector(aim)-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=scale
camera((-8,-9,6),(0,0,.35),8.3)
bpy.ops.wm.save_as_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'),compress=True)
print('DETAIL_COMPLETE',flush=True)
import os
os._exit(0)
