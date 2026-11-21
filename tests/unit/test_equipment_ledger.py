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
        """重布置后名义满油调平，两方案均不生成纵向配重。"""
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
                np.testing.assert_allclose(
                    result["pose_z_roll_pitch"][1:], [0, 0], atol=1e-8
                )
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
        for ballast in (p for p in parts if "geometry" in p):
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


if __name__ == "__main__":
    unittest.main()
