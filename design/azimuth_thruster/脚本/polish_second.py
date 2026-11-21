"""Refine model appearance and render geometry-based steering comparison."""
import bpy,math,json
from pathlib import Path
from mathutils import Vector
P=Path(__file__).resolve().parents[1]
bpy.ops.wm.open_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'))
s=bpy.context.scene;s.frame_set(1)
col=bpy.data.collections['第二船体 · 中央单机整体转向']
def material(name,color):
 m=bpy.data.materials.get(name) or bpy.data.materials.new(name);m.diffuse_color=(*color,1);return m
navy=material('V2 深海蓝',(0.035,.14,.23));white=material('V2 珍珠白',(.78,.84,.88));silver=material('V2 缎面铝',(.40,.49,.57));orange=material('V2 转向机构 橙',(1,.32,.055));black=material('V2 石墨黑',(.055,.075,.09))
def assign(o,m):
 o.data.materials.clear();o.data.materials.append(m)
def bevel(o,width):
 mod=o.modifiers.get('V2 精修圆角') or o.modifiers.new('V2 精修圆角','BEVEL');mod.width=width;mod.segments=4
 mod=o.modifiers.get('V2 加权法线') or o.modifiers.new('V2 加权法线','WEIGHTED_NORMAL')
for o in col.objects:
 if o.type!='MESH':continue
 if o.name.startswith('V2概念单机'):
  if '机罩顶壳' in o.name:assign(o,white)
  elif '分色带' in o.name:assign(o,orange)
  elif '机罩下壳' in o.name:assign(o,navy)
  elif '桨' in o.name:assign(o,silver)
  else:assign(o,navy)
  for poly in o.data.polygons:poly.use_smooth=True
 else:
  bevel(o,.008 if any(k in o.name for k in ['梁','背板']) else .003)
  if any(k in o.name for k in ['梁','背板','座']):assign(o,silver)
  if '执行器' in o.name:assign(o,navy)
# Upper beam and fixed motor cap become a coherent rounded housing; dimensions stay unchanged.
# Hull materials are local to this derivative and do not alter the baseline.
for o in s.objects:
 if o.type!='MESH' or o.hide_render:continue
 if o.name=='C方案船体':
  # Keep material assignments separating deck and shell; brighten inherited gray colors.
  for i,slot in enumerate(o.material_slots):
   if slot.material:
    m=slot.material.copy();m.name='V2船壳 · '+m.name
    rgb=m.diffuse_color[:3]
    m.diffuse_color=(*( (.10,.23,.31) if sum(rgb)<1 else (.72,.79,.82)),1)
    slot.material=m
# Rendering settings only: clean light background and brighter studio illumination.
s.render.engine='BLENDER_WORKBENCH';s.render.resolution_x=1600;s.render.resolution_y=1000;s.render.resolution_percentage=100
s.view_settings.view_transform='Standard';s.view_settings.exposure=.6;s.view_settings.gamma=1
sh=s.display.shading;sh.light='STUDIO';sh.studiolight_rotate_z=.35;sh.color_type='MATERIAL';sh.show_shadows=True;sh.show_cavity=True;sh.cavity_type='BOTH';sh.curvature_ridge_factor=1.2;sh.curvature_valley_factor=.8;sh.background_type='WORLD';s.world.color=(.82,.86,.90)
cam=s.camera;cam.data.type='ORTHO'
def camera(loc,aim,scale):
 cam.location=loc;cam.rotation_euler=(Vector(aim)-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=scale
camera((-8,-9,6),(0,0,.35),8.3)
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'),compress=True)
print('POLISH_COMPLETE',flush=True)
import os
os._exit(0)
