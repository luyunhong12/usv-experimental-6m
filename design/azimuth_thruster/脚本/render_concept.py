import bpy,os,json,math
from pathlib import Path
from mathutils import Vector
P=Path(__file__).resolve().parents[1];bpy.ops.wm.open_mainfile(filepath=str(P/'模型/第二船体_中央单机整体转向.blend'))
s=bpy.context.scene;s.frame_set(1);cam=s.camera
# Final full-assembly previews reflect the new cable routing and propeller parent.
s.render.resolution_x=1400;s.render.resolution_y=900
cam.location=(-8,-9,6);cam.rotation_euler=(Vector((0,0,.35))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=8.3
s.render.filepath=str(P/'图片/第二船体_总览.png');bpy.ops.render.render(write_still=True)
cam.location=(-6,-4,2.4);cam.rotation_euler=(Vector((-3.30,0,.43))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=2.65;s.frame_set(90)
s.render.filepath=str(P/'图片/中央推进器_转向35度.png');bpy.ops.render.render(write_still=True)
# Actual saved animation, isolated to its mechanical assembly for clear viewing.
allowed=['第二船体 · 中央单机整体转向','V2细化 · 传动与支撑','V2细化 · 可拆防护罩','V2完成 · 联动齿轮','V2完成 · 恒长活动线束']
for o in s.objects:
 if o.type in ('MESH','CURVE') and not any(c.name in allowed for c in o.users_collection):o.hide_render=True
s.render.resolution_x=640;s.render.resolution_y=640
cam.location=(-6,-4,2.4);cam.rotation_euler=(Vector((-3.34,0,.44))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=1.9
frames=Path('/tmp/usv_concept_animation');frames.mkdir(exist_ok=True)
for i,f in enumerate([1,9,17,25,30,38,46,54,60,68,76,84,90,98,106,114,120]):
 s.frame_set(f);s.render.filepath=str(frames/f'{i:03d}.png');bpy.ops.render.render(write_still=True)
 print('ANIMATION_FRAME',i,flush=True)
import subprocess
subprocess.run(['ffmpeg','-v','error','-y','-framerate','6','-i',str(frames/'%03d.png'),'-filter_complex','[0:v]split[a][b];[a]palettegen[p];[b][p]paletteuse',str(P/'图片/整体转向动作演示.gif')],check=True)
subprocess.run(['ffmpeg','-v','error','-y','-framerate','6','-i',str(frames/'%03d.png'),'-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(P/'图片/整体转向动作演示.mp4')],check=True)
print('PREVIEWS_COMPLETE',flush=True);os._exit(0)
