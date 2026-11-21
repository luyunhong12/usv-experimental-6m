"""Export existing appendage silhouettes without saving the source Blender file.
Run: blender -b --python-exit-code 1 --python tools/modeling/export_appendages.py
"""
from pathlib import Path
import hashlib,json,os


def convex_outline(points):
    points = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def main():
    import bpy
    from mathutils import Vector
    root=next(p for p in Path(__file__).resolve().parents if (p/'project.toml').is_file())
    source=root/'design/azimuth_thruster/模型/第二船体_中央单机整体转向.blend'
    target=root/'ros_ws/src/usv_description/config/appendage_hydrodynamics.json'
    cfg=json.loads(target.read_text());export=json.loads((root/'datasets/reference/boat_export.json').read_text())
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    if digest!=export['source_sha256']:raise RuntimeError('Re-export boat and collision geometry first; source revision differs')
    bpy.ops.wm.open_mainfile(filepath=str(source));bpy.context.scene.frame_set(1)
    dg=bpy.context.evaluated_depsgraph_get();origin=Vector(export['origins']['azimuth'])
    for s in cfg['surfaces']:
        obj=bpy.data.objects[s['source_object']].evaluated_get(dg);mesh=obj.to_mesh()
        vertices=[obj.matrix_world@v.co-origin for v in mesh.vertices]
        outline=convex_outline([(float(v.x),float(v.z)) for v in vertices]);obj.to_mesh_clear()
        s['outline_xz_m']=outline
        s['area_m2']=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(outline,outline[1:]+outline[:1])))/2
        s['span_m']=max(p[1] for p in outline)-min(p[1] for p in outline)
    cfg['source_sha256']=digest;target.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n')
    print('Exported appendage silhouettes; original Blender file unchanged',flush=True)
    # Some headless Blender audio backends hang during interpreter destruction.
    os._exit(0)


if __name__=='__main__':main()
