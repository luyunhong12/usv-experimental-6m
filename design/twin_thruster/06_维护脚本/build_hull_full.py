#!/usr/bin/env python3
"""
C 方案船体完整建模脚本 v2
严格按照 34 篇原始站位数据建模，坐标单位统一为米
龙骨基线 z=0，向上为正；x 从船尾向船首为正
"""
import bpy
import bmesh

print("="*60)
print("C 方案船体建模 v2（严格按 34 篇数据）")
print("="*60)

# 清理场景
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()
for mesh in bpy.data.meshes:
    bpy.data.meshes.remove(mesh)
for curve in bpy.data.curves:
    bpy.data.curves.remove(curve)

# ===== 严格按照 34 篇原始站位数据 =====
# 单位 mm，坐标系：x 从船中向首，y 向左舷，z 从龙骨基线向上
# 每行：(x, 龙骨K, 折角半宽B, 折角高C, 甲板半宽W, 舷侧高H)
stations_mm = [
    (-3000,  40,  740, 130,  900, 880),   # 0 船尾
    (-2400,  15,  820, 110,  990, 880),   # 1
    (-1200,   0,  900, 115, 1090, 880),   # 2
    (    0,   0,  925, 135, 1125, 880),   # 3 船中
    ( 1000,  10,  870, 165, 1100, 890),   # 4
    ( 1500,  30,  770, 195, 1020, 910),   # 5
    ( 2100,  80,  560, 250,  790, 940),   # 6
    ( 2600, 170,  290, 340,  470, 980),   # 7
    ( 2850, 275,  125, 400,  245,1000),   # 8
    ( 3000,1020,    0,1020,    0,1020),   # 9 船首（收尖成一点）
]

print(f"\n载入 {len(stations_mm)} 个站位数据（来自 34 篇）")

mesh = bpy.data.meshes.new("船体")
hull_obj = bpy.data.objects.new("C方案船体", mesh)
bpy.context.collection.objects.link(hull_obj)

bm = bmesh.new()

all_verts = []  # 每个元素是该站位的 [左舷, 左折角, 龙骨, 右折角, 右舷]

for i, (x, K, B, C, W, H) in enumerate(stations_mm):
    s = 1/1000  # mm → m

    # 船首 (站位9) 收尖成一点
    if B == 0:
        p = (x*s, 0, K*s)
        v = bm.verts.new(p)
        all_verts.append([v, v, v, v, v])
        print(f"  站{i:2d} x={x/1000:+.2f}m  船首尖点 z={K*s:.3f}m")
    else:
        # 5个点：左舷甲板 → 左折角 → 龙骨 → 右折角 → 右舷甲板
        v0 = bm.verts.new((x*s,  W*s, H*s))   # 左舷甲板
        v1 = bm.verts.new((x*s,  B*s, C*s))   # 左折角
        v2 = bm.verts.new((x*s,    0, K*s))   # 龙骨
        v3 = bm.verts.new((x*s, -B*s, C*s))   # 右折角
        v4 = bm.verts.new((x*s, -W*s, H*s))   # 右舷甲板
        all_verts.append([v0, v1, v2, v3, v4])
        print(f"  站{i:2d} x={x/1000:+.2f}m  宽={2*W/1000:.2f}m  高={H/1000:.3f}m  龙骨z={K/1000:.3f}m")

print(f"\n连接站位，创建面...")

# 在相邻站位之间创建面
for i in range(len(all_verts) - 1):
    curr = all_verts[i]
    nxt  = all_verts[i + 1]

    # 判断下一站是否是船首尖点
    if nxt[0] is nxt[1]:  # 船首收尖
        # 创建三角形（收尖到一点）
        for j in range(4):
            if curr[j] is not curr[j+1]:
                try:
                    bm.faces.new([curr[j], curr[j+1], nxt[0]])
                except ValueError:
                    pass
    else:
        # 普通四边形面
        for j in range(4):
            try:
                bm.faces.new([curr[j], curr[j+1], nxt[j+1], nxt[j]])
            except ValueError:
                pass

# 首版双斜面甲板：横坡暂取 2%，船中拱高 22.5 mm。
deck_slope = 0.02
ridge = []
for section in all_verts:
    port, starboard = section[0], section[4]
    if port is starboard:
        ridge.append(port)
    else:
        x, half_width, edge_z = port.co
        ridge.append(bm.verts.new((x, 0, edge_z + half_width * deck_slope)))

stern = all_verts[0]
bm.faces.new([*reversed(stern), ridge[0]])
for i, (aft, fore) in enumerate(zip(all_verts, all_verts[1:])):
    for edge in (0, 4):
        corners = [aft[edge], fore[edge], ridge[i + 1], ridge[i]]
        bm.faces.new(list(dict.fromkeys(corners)))

# 更新法线（确保朝外）
bm.normal_update()
bm.to_mesh(mesh)
bm.free()

# 重新计算法线
bpy.context.view_layer.objects.active = hull_obj
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.normals_make_consistent(inside=False)
bpy.ops.object.mode_set(mode='OBJECT')

# 平滑着色（但不加细分曲面，保持折角清晰）
mesh.shade_smooth()
# 加 Edge Split 修改器，让折角保持尖锐
edge_split = hull_obj.modifiers.new(name="EdgeSplit", type='EDGE_SPLIT')
edge_split.split_angle = 0.5236  # 30度，折角处不平滑

print(f"\n{'='*60}")
print("✓ 船体建模完成（v2，严格按 34 篇数据）")
print(f"{'='*60}")
print(f"  顶点: {len(mesh.vertices)}")
print(f"  面:   {len(mesh.polygons)}")
print(f"\n船体规格（C 方案）：")
print(f"  船长: 6.0 m（−3.0 ~ +3.0）")
print(f"  最大型宽: 2.25 m（2 × 1125 mm，船中站）")
print(f"  型深: 0.88 m（船中舷侧高 880 mm）")
print(f"  静水面: z ≈ 0.190 m（满油工况）")
print(f"\n提示：")
print(f"  小键盘 1 → 前视图（看 V 形船底）")
print(f"  小键盘 3 → 侧视图（看龙骨和船侧轮廓）")
print(f"  小键盘 7 → 俯视图（看甲板轮廓）")
print(f"  小键盘 . → 居中显示")
