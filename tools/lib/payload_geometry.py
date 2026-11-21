"""Engineering device envelopes and brackets attached to the existing hull."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import xml.etree.ElementTree as E
import math

def add_mounts(hull,cfg):
 def part(name,pose,size,color):
  for kind in ['visual','collision']:
   e=E.SubElement(hull,kind,name=name+'_'+kind);E.SubElement(e,'pose').text=' '.join(map(str,pose));b=E.SubElement(E.SubElement(e,'geometry'),'box');E.SubElement(b,'size').text=' '.join(map(str,size))
   if kind=='visual':
    m=E.SubElement(e,'material');E.SubElement(m,'diffuse').text=color;p=E.SubElement(E.SubElement(m,'pbr'),'metal');E.SubElement(p,'metalness').text='.2';E.SubElement(p,'roughness').text='.55'
 cameras=['camera','camera_right','camera_side'] if cfg['advanced_sensors'] else ['camera']
 for name in cameras:
  x,y,z,r,p,yaw=cfg['poses'][name];part(name+'_housing',[x-.065*math.cos(yaw),y-.065*math.sin(yaw),z,r,p,yaw],[.10,.075,.065],'.12 .14 .16 1')
  part(name+'_bracket',[x-.065*math.cos(yaw),y-.065*math.sin(yaw),(z+.99)/2,0,0,yaw],[.035,.035,z-.99],'.5 .52 .55 1')
 x,y,z,*_=cfg['poses']['lidar'];part('lidar_mast',[x,y,(z-.08+.99)/2,0,0,0],[.045,.045,z-.08-.99],'.5 .52 .55 1');part('lidar_housing',[x,y,z,0,0,0],[.10,.10,.09],'.1 .12 .14 1')
