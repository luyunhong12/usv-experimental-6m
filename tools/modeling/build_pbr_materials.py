#!/usr/bin/env python3
"""生成或复用模型渲染材质与纹理资源。

Derive PBR render assets from existing CAD palette; physical meshes untouched.
"""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys

# 项目脚本可从任意工作目录启动；建模工具保持在共享库之前。
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/lib"
    ),
)
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/modeling"
    ),
)
from project_paths import WORKSPACE
from pathlib import Path
import json
import hashlib
import xml.etree.ElementTree as E
import numpy as np
from PIL import Image

w = WORKSPACE
root = w / "src/usv_description"
out = root / "meshes/pbr"
out.mkdir(parents=True, exist_ok=True)
ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
paths = [root / "meshes" / f"{g}.dae" for g in ["hull", "azimuth", "propeller"]]
hashes = {p.stem: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
manifest = root / "config/pbr_visuals.json"
if (
    manifest.exists()
    and json.loads(manifest.read_text()).get("schema") == 2
    and json.loads(manifest.read_text()).get("source_hashes") == hashes
    and all(
        Path(v["mesh"]).exists()
        for group in json.loads(manifest.read_text())["groups"].values()
        for v in group
    )
):
    print("PBR assets current")
    raise SystemExit
(out / "base.mtl").write_text("newmtl surface\nKd 1 1 1\n")
report = {
    "schema": 2,
    "source_hashes": hashes,
    "basis": "CAD palette and named finish approximations, procedural microtexture; no measured BRDF; geometry unchanged",
    "groups": {},
}
rng = np.random.default_rng(20261120)
grain = rng.normal(0, 1, (128, 128))
yy, xx = np.mgrid[:128, :128]
grain = 0.65 * grain + 0.35 * np.sin(2 * np.pi * yy / 8)
normal = np.zeros((128, 128, 3), dtype=np.uint8)
normal[:, :, 0] = np.clip(128 + 2 * np.gradient(grain, axis=1), 0, 255)
normal[:, :, 1] = np.clip(128 + 2 * np.gradient(grain, axis=0), 0, 255)
normal[:, :, 2] = 255
Image.fromarray(normal).save(out / "micro_normal.png")
for path in paths:
    t = E.parse(path)
    mesh = t.find(".//c:geometry/c:mesh", ns)
    src = {
        s.get("id"): np.fromstring(
            s.findtext("c:float_array", namespaces=ns), sep=" "
        ).reshape(-1, 3)
        for s in mesh.findall("c:source", ns)
    }
    pos = src["pos"]
    norm = src["norm"]
    records = []
    for i, tr in enumerate(mesh.findall("c:triangles", ns)):
        ef = t.find(f".//c:effect[@id='e{i}']/c:profile_COMMON/c:technique/c:phong", ns)
        color = np.fromstring(ef.findtext("c:diffuse/c:color", namespaces=ns), sep=" ")
        spec = np.fromstring(ef.findtext("c:specular/c:color", namespaces=ns), sep=" ")[
            :3
        ]
        shin = float(ef.findtext("c:shininess/c:float", namespaces=ns))
        delta = color[:3] - 0.04
        metal = float(
            np.clip(np.dot(spec - 0.04, delta) / max(np.dot(delta, delta), 1e-12), 0, 1)
        )
        rough = float(np.clip(np.sqrt(2 / (shin + 2)), 0.05, 1))
        stem = f"{path.stem}_{i:02d}"
        pairs = np.fromstring(
            tr.findtext("c:p", namespaces=ns), sep=" ", dtype=int
        ).reshape(-1, 3, 2)
        axis = np.abs(norm[pairs[:, :, 1]].mean(1)).argmax(1)
        keys = np.column_stack((pairs.reshape(-1, 2), np.repeat(axis, 3)))
        unique, inv = np.unique(keys, axis=0, return_inverse=True)
        v = pos[unique[:, 0]]
        n = norm[unique[:, 1]]
        uv = np.empty((len(v), 2))
        for ax, dims in [(0, [1, 2]), (1, [0, 2]), (2, [0, 1])]:
            mask = unique[:, 2] == ax
            uv[mask] = v[mask][:, dims]
        with (out / (stem + ".obj")).open("w") as f:
            f.write(
                "# PBR-only mesh; source positions and split normals retained\nmtllib base.mtl\nusemtl surface\n"
            )
            np.savetxt(f, v, fmt="v %.8g %.8g %.8g")
            np.savetxt(f, uv, fmt="vt %.8g %.8g")
            np.savetxt(f, n, fmt="vn %.8g %.8g %.8g")
            faces = inv.reshape(-1, 3) + 1
            for a, b, c in faces:
                f.write(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n")
        linear = np.clip(
            color[:3][None, None, :] * (1 + 0.006 * grain[:, :, None]), 0, 1
        )
        rgb = (
            255
            * np.where(
                linear <= 0.0031308, 12.92 * linear, 1.055 * linear ** (1 / 2.4) - 0.055
            )
        ).astype(np.uint8)
        Image.fromarray(rgb).save(out / (stem + "_albedo.png"))
        Image.fromarray(
            np.clip((rough + 0.015 * grain) * 255, 0, 255).astype(np.uint8)
        ).save(out / (stem + "_roughness.png"))
        records.append(
            {
                "name": stem,
                "mesh": str((out / (stem + ".obj")).resolve()),
                "albedo": str((out / (stem + "_albedo.png")).resolve()),
                "roughness_map": str((out / (stem + "_roughness.png")).resolve()),
                "normal_map": str((out / "micro_normal.png").resolve()),
                "metalness": metal,
                "roughness": rough,
                "triangles": len(pairs),
            }
        )
    report["groups"][path.stem] = records
    print(path.stem, sum(r["triangles"] for r in records), flush=True)
manifest.write_text(json.dumps(report, indent=2))
