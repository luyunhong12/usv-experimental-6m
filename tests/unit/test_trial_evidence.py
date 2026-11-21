"""验证旧结果、变更工况和篡改遥测不能通过证据绑定。"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/lib"))
from trial_evidence import initialize_suite, seal_case, verify_case


class EvidenceTests(unittest.TestCase):
    """在临时目录中构造独立证据，不启动 Gazebo。"""

    def setUp(self):
        """建立最小完整工况及其不可混用的套件记录。"""
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "run"
        self.spec = {"stages": [{"name": "neutral", "duration": 10}]}
        self.snapshot = {"source": "version-a", "binary": "binary-a"}
        self.binding = initialize_suite(self.root, {"case": self.spec}, self.snapshot)
        self.case = self.root / "case"
        self.case.mkdir()
        for filename, value in {
            "result.json": {"spec": self.spec, "passed": True},
            "run_status.json": {"spec": self.spec},
            "stages.json": self.spec["stages"],
        }.items():
            (self.case / filename).write_text(json.dumps(value))
        (self.case / "propulsion_state.csv").write_text("0,1\n1,2\n")
        seal_case(self.case, "case", self.spec, self.binding["fingerprint"])

    def test_explicit_identical_resume(self):
        """只有显式恢复且源码、二进制、工况一致时才允许复用。"""
        same = initialize_suite(self.root, {"case": self.spec}, self.snapshot, True)
        self.assertEqual(same, self.binding)
        self.assertTrue(
            verify_case(self.case, "case", self.spec, same["fingerprint"])["passed"]
        )
        with self.assertRaises(ValueError):
            initialize_suite(self.root, {"case": self.spec}, self.snapshot)

    def test_source_and_binary_change_rejected(self):
        """源码或构建库内容变化均使旧缓存失效。"""
        for key in self.snapshot:
            changed = dict(self.snapshot, **{key: "different"})
            with self.assertRaises(ValueError):
                initialize_suite(self.root, {"case": self.spec}, changed, True)

    def test_requested_spec_change_rejected(self):
        """同名工况不能用旧遥测证明新参数。"""
        other = {"stages": [{"name": "neutral", "duration": 20}]}
        with self.assertRaises(ValueError):
            verify_case(self.case, "case", other)

    def test_trace_modification_rejected(self):
        """结果仍写 passed 时，修改过的遥测也必须被拒绝。"""
        (self.case / "propulsion_state.csv").write_text("0,999\n")
        with self.assertRaises(ValueError):
            verify_case(self.case, "case", self.spec)

    def test_legacy_and_incomplete_outputs_rejected(self):
        """无版本清单的历史输出不能直接标记为新版本验收。"""
        (self.root / "suite_binding.json").unlink()
        with self.assertRaises(ValueError):
            initialize_suite(self.root, {"case": self.spec}, self.snapshot, True)
        (self.case / "case_binding.json").unlink()
        with self.assertRaises(FileNotFoundError):
            verify_case(self.case, "case", self.spec)

    def test_missing_hash_rejected(self):
        """删去必需遥测摘要不能使不完整证据继续通过。"""
        path = self.case / "case_binding.json"
        value = json.loads(path.read_text())
        value["files"].pop("propulsion_state.csv")
        path.write_text(json.dumps(value))
        with self.assertRaises(ValueError):
            verify_case(self.case, "case", self.spec, self.binding["fingerprint"])

    def test_failed_process_cannot_be_masked_by_result_json(self):
        """子进程失败时，即使遗留结果写通过也必须使套件失败。"""
        import runpy
        from unittest.mock import patch

        output = Path(self.temp.name) / "failed_process"
        scenario = Path(self.temp.name) / "spec.json"
        scenario.write_text(json.dumps({"case": self.spec}))
        harness = Path(self.temp.name) / "failed_harness.py"
        harness.write_text(
            "import sys,json\nfrom pathlib import Path\n"
            "out=Path(sys.argv[sys.argv.index('--output')+1])\n"
            "out.mkdir(parents=True)\n"
            "(out/'result.json').write_text(json.dumps({'passed':True}))\n"
            "raise SystemExit(1)\n"
        )
        runner = (
            Path(__file__).resolve().parents[2]
            / "tests/regression/run_performance_suite.py"
        )
        argv = [
            str(runner),
            "--spec",
            str(scenario),
            "--output",
            str(output),
            "--workers",
            "1",
        ]
        with patch.object(sys, "argv", argv), patch(
            "trial_evidence.execution_snapshot", return_value=self.snapshot
        ), patch("project_paths.tool_path", return_value=harness):
            with self.assertRaises(SystemExit) as caught:
                runpy.run_path(str(runner), run_name="__main__")
        self.assertEqual(caught.exception.code, 1)
        summary = json.loads((output / "summary.json").read_text())
        self.assertFalse(summary["passed"])
        self.assertEqual(summary["exit_codes"]["case"], 1)
