#!/usr/bin/env python3
"""验证逐件设备台账的质量、油量和功率记账。

Accounting invariants independent of the generated SDF layout.
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
import unittest
import copy
from unittest.mock import patch
import numpy as np
from equipment_ledger import load, rows, dry_links, auxiliary_w
from mass_properties import aggregate
from fuel_mass import combine, configuration, tank_properties


class ContactPrecisionTest(unittest.TestCase):
    """用解析立方体和人为失败轨迹验证接触误差判定。"""

    def test_signed_gap_includes_inside_sphere(self):
        """球心进入实体时不可把到表面的正距离误认为没有穿透。"""
        import trimesh
        from contact_checks import sphere_mesh_gaps

        mesh = trimesh.creation.box(extents=[2, 2, 2])
        gaps = sphere_mesh_gaps(
            [[0, 0, 1.1], [0, 0, 0.9]], mesh.vertices, mesh.faces, 0.1
        )
        np.testing.assert_allclose(gaps, [0, -0.2], atol=1e-12)

    def test_precision_rejects_penetration_and_sampling_gaps(self):
        """稳态 6 mm 穿透、整球穿过及漏采样必须失败。"""
        import trimesh
        from contact_checks import evaluate_fin_precision

        mesh = trimesh.creation.box(extents=[2, 2, 2])
        times = np.arange(0, 8.002, 0.001)
        rows = np.tile([0, 0, 1.025], (len(times), 1))
        args = (0, [0, 0, 0], mesh.vertices, mesh.faces, 0.001, 0.6)
        self.assertTrue(evaluate_fin_precision(rows, times, *args)["passed"])
        rows[:, 2] -= 0.006
        self.assertFalse(evaluate_fin_precision(rows, times, *args)["passed"])
        rows[:, 2] = 0.9
        self.assertFalse(evaluate_fin_precision(rows, times, *args)["passed"])
        rows[:, 2] = 1.025
        self.assertFalse(evaluate_fin_precision(rows[::3], times[::3], *args)["passed"])


class LedgerTests(unittest.TestCase):
    """验证设备台账质量、功耗和燃油的独立记账。"""

    def test_profile_increment(self):
        """验证方案升级增加的质量与台账一致。"""
        self.assertAlmostEqual(
            sum(r["mass_kg"] for r in rows("speed15_concept"))
            - sum(r["mass_kg"] for r in rows()),
            4 * 76 + 4 * 2 + 30 + 1.5 * (1.5**3 - 1),
        )

    def test_unique_and_physical(self):
        """验证标识唯一且质量惯性满足基本物理约束。"""
        for profile in ["baseline", "speed15_concept"]:
            for case in ["light", "nominal", "heavy"]:
                p = aggregate(rows(profile, case))
                self.assertTrue(np.linalg.eigvalsh(p["inertia_at_cg_kg_m2"]).min() > 0)

    def test_reject_duplicate(self):
        """验证重复台账标识被拒绝。"""
        data = load()
        data["parts"].append(copy.deepcopy(data["parts"][0]))
        with patch("equipment_ledger.load", return_value=data):
            with self.assertRaises(ValueError):
                rows()

    def test_fuel_exact_once(self):
        """验证燃油质量只计入一次。"""
        cfg = configuration()
        dry = dry_links()["hull"]
        self.assertAlmostEqual(cfg["dry_hull"]["mass_kg"], dry["mass_kg"])
        for fuel_litres in [0, 6, 30, 60]:
            self.assertAlmostEqual(
                combine(dry, tank_properties(cfg, fuel_litres))["mass_kg"]
                - dry["mass_kg"],
                0.84 * fuel_litres,
            )

    def test_power_modes(self):
        """验证不同模式的辅助功率预算。"""
        self.assertLess(auxiliary_w("normal"), auxiliary_w("survey"))
        self.assertLess(auxiliary_w("survey"), auxiliary_w("peak"))
        self.assertAlmostEqual(
            auxiliary_w("peak"),
            sum(
                r["power"]["rated_w"]
                for r in rows()
                if r["power"]["class"] == "auxiliary"
            ),
        )

    def test_battery_capacity_matches_count(self):
        """验证电池容量与所选电池数量一致。"""
        self.assertEqual(sum("E163" in r["name"] for r in rows()), 2)
        self.assertEqual(sum("E163" in r["name"] for r in rows("speed15_concept")), 6)

    def test_nominal_fore_ballast_equilibrium(self):
        """新增鳍后检验真实浮力平衡，两方案均不生成纵向配重。"""
        from pathlib import Path
        from scenario_profiles import apply_speed15
        import xml.etree.ElementTree as E

        path = (
            Path(__file__).resolve().parents[2]
            / "ros_ws/src/usv_description/models/usv_azimuth_trial.sdf"
        )
        for profile in ["baseline", "speed15_concept"]:
            model = E.parse(path).find("model")
            result = apply_speed15(model, profile=profile)
            solid = model.find(
                "link[@name='hull']/collision[@name='fixed_fore_trim_ballast_collision']"
            )
            self.assertIsNone(solid)
            if profile == "speed15_concept":
                from fuel_equilibrium import displaced

                pose = result["pose_z_roll_pitch"]
                volume, cb, rotation = displaced(pose)
                cg = rotation @ np.array(result["ship_cg_m"])
                np.testing.assert_allclose(cb[:2], cg[:2], atol=1e-8)
                self.assertAlmostEqual(volume * 1000, result["ship_mass_kg"], places=5)
                self.assertLess(max(abs(np.array(pose[1:]))), np.radians(0.05))
                self.assertAlmostEqual(
                    result["ship_mass_kg"],
                    sum(p["mass_kg"] for p in rows(profile)) + 60 * 0.84,
                )

    def test_relocated_batteries_clear_equipment(self):
        """电池与托架须避开集中设备包络，并保留至少 30 mm 船壳余量。"""
        parts = rows("speed15_concept")
        distributed = {"wiring", "fenders", "mooring", "installation_fasteners"}
        obstacles = [p for p in parts if "size_m" in p and p["id"] not in distributed]
        for part in parts:
            if part["id"] in {"retained_2", "retained_3"}:
                obstacles.append(dict(part, size_m=[0.47, 0.295, 0.495]))
            if part["id"] == "fixed_trim_ballast":
                obstacles.append(dict(part, size_m=part["geometry"]["size_m"]))
        moved = [
            p for p in parts if p["id"].startswith(("extra_battery_", "extra_tray_"))
        ]
        self.assertEqual(sum(p["mass_kg"] for p in moved), 312)
        for part in moved:
            for obstacle in obstacles:
                if part["id"] == obstacle["id"]:
                    continue
                overlap = (np.array(part["size_m"]) + obstacle["size_m"]) / 2 - abs(
                    np.array(part["cg_m"]) - obstacle["cg_m"]
                )
                self.assertLessEqual(
                    float(min(overlap)),
                    0,
                    msg=part["id"] + " overlaps " + obstacle["id"],
                )

    def test_fixed_ballast_mass_geometry_and_balance(self):
        """配重来自横向质量矩，实体体积和惯性必须与台账一致。"""
        import itertools
        from pathlib import Path
        from scipy.spatial import ConvexHull
        from payload_geometry import add_ledger_solids
        import xml.etree.ElementTree as E

        parts = rows("speed15_concept")
        self.assertAlmostEqual(sum(p["mass_kg"] * p["cg_m"][1] for p in parts), 0)
        root = Path(__file__).resolve().parents[2]
        triangles = np.loadtxt(
            root / "ros_ws/src/usv_description/config/hull_triangles.txt"
        ).reshape(-1, 3)
        envelope = ConvexHull(triangles).equations
        for ballast in (p for p in parts if p.get("geometry", {}).get("type") == "box"):
            size = np.array(ballast["geometry"]["size_m"])
            mass = ballast["mass_kg"]
            if "material_density_kg_m3" in ballast["geometry"]:
                self.assertAlmostEqual(
                    np.prod(size) * ballast["geometry"]["material_density_kg_m3"], mass
                )
            else:
                np.testing.assert_allclose(size, ballast["size_m"])
            expected = mass / 12 * (np.sum(size**2) - size**2)
            np.testing.assert_allclose(
                np.diag(ballast["inertia_at_cg_kg_m2"]), expected
            )
            for signs in itertools.product([-1, 1], repeat=3):
                corner = np.array(ballast["cg_m"]) + np.array(signs) * size / 2
                self.assertLessEqual(
                    np.max(envelope[:, :3] @ corner + envelope[:, 3]), -0.03
                )
        hull = E.Element("link")
        add_ledger_solids(hull, parts)
        add_ledger_solids(hull, parts)  # repeated profile application is idempotent
        self.assertEqual(len(hull.findall("visual")), 9)
        self.assertEqual(len(hull.findall("collision")), 9)
        self.assertIsNone(hull.find("inertial"))  # no duplicate mass from rendering


class StabilizerTests(unittest.TestCase):
    """以独立解析体积及几何记账检查新增小型被动鳍。"""

    def test_solid_accounting(self):
        """解析椭圆柱加相似截面棱台校验体积和台账质量惯性。"""
        from passive_stabilizer import configuration, geometry, mass_properties

        part = configuration()
        g = part["geometry"]
        chord, thickness, span = g["size_m"]
        taper, scale = g["tip_taper_m"], g["tip_scale"]
        exact = (
            np.pi
            * chord
            * thickness
            / 4
            * (span - taper + taper * (1 + scale + scale**2) / 3)
        )
        volume, mass, cg, inertia = mass_properties(part)
        self.assertLess(abs(volume / exact - 1), 0.002)
        self.assertAlmostEqual(mass, part["mass_kg"], places=9)
        np.testing.assert_allclose(cg, part["cg_m"], atol=1e-10)
        np.testing.assert_allclose(inertia, part["inertia_at_cg_kg_m2"], atol=1e-10)
        vertices, faces, _ = geometry(part)
        self.assertGreater(
            vertices[:, 2].min(), -0.5
        )  # 高于原推进器，不增加整艇最深点。
        edges = np.sort(
            np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]),
            axis=1,
        )
        _, counts = np.unique(edges, axis=0, return_counts=True)
        self.assertTrue(np.all(counts == 2))
        self.assertEqual(part["power"]["rated_w"], 0)

    def test_generation_idempotent(self):
        """两次生成仍各有一个显示、碰撞及独立受力实例。"""
        import tempfile
        import xml.etree.ElementTree as E
        from passive_stabilizer import add_stabilizer, configuration

        model = E.fromstring('<model><link name="hull"/></model>')
        with tempfile.TemporaryDirectory() as folder:
            for _ in range(2):
                add_stabilizer(model, configuration(), folder)
            self.assertEqual(len(model.findall("link/visual")), 1)
            self.assertEqual(len(model.findall("link/collision")), 1)
            self.assertEqual(len(model.findall("plugin")), 1)
            self.assertEqual(
                model.findtext("link/visual/geometry/mesh/uri"),
                model.findtext("link/collision/geometry/mesh/uri"),
            )
            self.assertEqual(model.findtext("plugin/topic_namespace"), "usv")
            self.assertEqual(model.findtext("plugin/telemetry_group"), "stabilizer")
            from pathlib import Path

            lines = (
                Path(model.findtext("link/collision/geometry/mesh/uri"))
                .read_text()
                .splitlines()
            )
            normals = [
                list(map(float, line.split()[1:]))
                for line in lines
                if line.startswith("vn ")
            ]
            np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-10)
            for line in (line for line in lines if line.startswith("f ")):
                for index in line.split()[1:]:
                    self.assertTrue(1 <= int(index.split("//")[1]) <= len(normals))

    def test_invalid_geometry(self):
        """非法尺寸和截面数量在生成之前拒绝。"""
        from passive_stabilizer import configuration, geometry

        for key, value in [
            ("tip_scale", 0),
            ("tip_taper_m", 1),
            ("sections", 17),
            ("size_m", [1, float("nan"), 1]),
        ]:
            part = configuration()
            part["geometry"][key] = value
            with self.assertRaises(ValueError):
                geometry(part)


if __name__ == "__main__":
    unittest.main()
