"""验证去部件惯性计算及候选船的隔离生成行为。"""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/modeling"))
from prepare_simple_vessel import prepare, subtract_part  # noqa: E402


class SimpleVesselTest(unittest.TestCase):
    """用可手算的对称刚体和实际候选配置检查基础契约。"""

    def test_remove_offset_body(self):
        """两个相距2米的同质量刚体移除其一后恢复剩余自身惯性。"""
        part = {
            "mass_kg": 1.0,
            "cg_m": [1.0, 0, 0],
            "inertia_at_cg_kg_m2": np.eye(3).tolist(),
        }
        result = subtract_part(2.0, [0, 0, 0], np.diag([2, 4, 4]), part)
        self.assertEqual(result["mass_kg"], 1.0)
        np.testing.assert_allclose(result["cg_m"], [-1, 0, 0])
        np.testing.assert_allclose(result["inertia_at_cg_kg_m2"], np.eye(3))

    def test_reject_nonpositive_remaining_mass(self):
        """拒绝删除整船或比整船更重的部件。"""
        with self.assertRaises(ValueError):
            subtract_part(1.0, [0, 0, 0], np.eye(3), {"mass_kg": 1.0})

    def test_candidate_contract(self):
        """生成不含鳍和能源配置的独立候选且拒绝覆盖试验输出。"""
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "candidate"
            prepare(output)
            model = ET.parse(output / "model.sdf").getroot().find("model")
            self.assertIsNone(model.find("plugin[@name='usv::PassiveStabilizer']"))
            propulsion = model.find("plugin[@name='usv::AzimuthPropulsion']")
            self.assertIsNone(propulsion.find("energy"))
            self.assertIsNone(propulsion.find("fuel_mass"))
            self.assertEqual(propulsion.findtext("energy_enabled"), "false")
            self.assertEqual(len(model.findall("joint")), 2)
            with self.assertRaises(FileExistsError):
                prepare(output)

    def test_explicit_source_is_used(self):
        """显式来源必须生效，不能静默退回默认6 kW模型。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(__file__).resolve().parents[2]
            tree = ET.parse(
                root / "ros_ws/src/usv_description/models/usv_azimuth_trial.sdf"
            )
            tree.getroot().find("model").set("name", "selected_source")
            source = Path(directory) / "input.sdf"
            tree.write(source)
            output = Path(directory) / "candidate"
            prepare(output, source)
            result = ET.parse(output / "model.sdf").getroot().find("model")
            self.assertEqual(result.get("name"), "selected_source")


if __name__ == "__main__":
    unittest.main()
