"""确保不完整导航运行、缺失检查和错误结果类型不能伪装通过。"""

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/lib"))
from navigation_acceptance import passed_checks, required_checks


class NavigationAcceptanceTests(unittest.TestCase):
    """验证提前退出和分阶段验收的失败语义。"""

    def test_empty_and_partial_runs_fail(self):
        """空结果、仅连接成功或未完成任务不得通过。"""
        self.assertFalse(passed_checks({}))
        self.assertFalse(passed_checks({"normal_arming": True, "finite": True}))
        self.assertFalse(passed_checks(required_checks()))

    def test_every_required_stage_is_enforced(self):
        """逐项丢失或失败都应使完整运行失败。"""
        checks = dict.fromkeys(required_checks(), True)
        self.assertTrue(passed_checks(checks))
        for name in checks:
            with self.subTest(name=name):
                self.assertFalse(
                    passed_checks({k: v for k, v in checks.items() if k != name})
                )
                self.assertFalse(passed_checks({**checks, name: False}))
                self.assertFalse(passed_checks({**checks, name: "False"}))

    def test_failsafe_cannot_certify_navigation(self):
        """仅失联试验不能充当 AUTO、返航与接管验收。"""
        checks = dict.fromkeys(required_checks(True), True)
        self.assertTrue(passed_checks(checks, True))
        self.assertFalse(passed_checks(checks))


class SustainedStopTests(unittest.TestCase):
    """拒绝单点停推、遥测缺口和中途重新推进。"""

    def test_stop_requires_continuous_window(self):
        """窗口内每个样本均须处于 HOLD 且停推。"""
        from navigation_acceptance import sustained_stop

        samples = [(i / 10, 0.0, 4) for i in range(61)]
        self.assertTrue(sustained_stop(samples))
        self.assertFalse(sustained_stop(samples[-1:]))
        self.assertFalse(sustained_stop(samples[:10] + samples[25:]))
        self.assertFalse(sustained_stop(samples[:30] + [(3.0, 0.2, 4)] + samples[31:]))
        self.assertFalse(sustained_stop([(t, c, 0) for t, c, _ in samples]))


class LoiterTargetTests(unittest.TestCase):
    """防止缺失目标或移动目标伪装保持成功。"""

    def test_target_coverage_and_stationarity(self):
        """不足时长、缺测、漂移目标和非有限数值必须拒绝。"""
        from navigation_acceptance import observed_loiter_target

        samples = [[16 + i / 5, 2.0, -33.7, 150.6, -33.7, 150.6] for i in range(521)]
        self.assertTrue(observed_loiter_target(samples))
        self.assertFalse(observed_loiter_target([]))
        self.assertFalse(observed_loiter_target(samples[:100]))
        self.assertFalse(observed_loiter_target(samples[:100] + samples[120:]))
        drift = [row.copy() for row in samples]
        drift[-1][4] += 0.001
        self.assertFalse(observed_loiter_target(drift))
        drift[-1][4] = float("nan")
        self.assertFalse(observed_loiter_target(drift))
        invalid_time = [row.copy() for row in samples]
        invalid_time[100][0] = float("nan")
        self.assertFalse(observed_loiter_target(invalid_time))
        self.assertFalse(observed_loiter_target(samples + [[121, 0]]))


class SustainedMotionTests(unittest.TestCase):
    """确保失联故障仅在真实持续运动时注入。"""

    def test_motion_requires_speed_thrust_and_coverage(self):
        """拒绝只有指令、短时移动、模式错误和中途缺测。"""
        from navigation_acceptance import sustained_motion

        samples = [[i / 10, 0.6, 0.2, 15] for i in range(31)]
        self.assertTrue(sustained_motion(samples))
        self.assertFalse(sustained_motion(samples[:10]))
        self.assertFalse(sustained_motion(samples[:10] + samples[20:]))
        for column, value in [(1, 0.0), (2, 0.0), (3, 4), (1, float("nan"))]:
            invalid = [row.copy() for row in samples]
            invalid[15][column] = value
            self.assertFalse(sustained_motion(invalid))


class ResponseIdentificationTests(unittest.TestCase):
    """区分开环辨识与完整导航验收，拒绝不完整响应数据。"""

    def test_response_schedule_and_scope(self):
        """确认反推边界、结束回中及辨识不能冒充导航通过。"""
        from navigation_acceptance import response_command

        self.assertEqual(response_command(149.9)[1][2], 1563)
        self.assertEqual(response_command(150)[1][2], 1437)
        self.assertEqual(response_command(290)[1], (1500, 1500, 1500))
        for value in [-1, float("nan"), float("inf")]:
            with self.assertRaises(ValueError):
                response_command(value)
        checks = dict.fromkeys(required_checks(response_only=True), True)
        self.assertTrue(passed_checks(checks, response_only=True))
        self.assertFalse(passed_checks(checks))
        checks["response_coverage"] = False
        self.assertFalse(passed_checks(checks, response_only=True))

    def test_response_requires_complete_continuous_observations(self):
        """拒绝截短、缺段、长缺测及非有限响应记录。"""
        from navigation_acceptance import response_command, response_coverage

        samples = [[i / 10, i / 10, response_command(i / 10)[0]] for i in range(2900)]
        self.assertTrue(response_coverage(samples))
        self.assertFalse(response_coverage(samples[:-100]))
        self.assertFalse(response_coverage(samples[:500] + samples[520:]))
        samples[100][0] = float("nan")
        self.assertFalse(response_coverage(samples))


class WeatherAcceptanceTests(unittest.TestCase):
    """确保风浪输入缺失、物理遥测缺测和姿态超界不能伪装通过。"""

    def test_weather_requires_additional_gates(self):
        """原25项全绿仍不能代替包含风浪的30项验收。"""
        checks = dict.fromkeys(required_checks(), True)
        self.assertFalse(passed_checks(checks, weather=True))
        checks.update(dict.fromkeys(required_checks(weather=True), True))
        self.assertTrue(passed_checks(checks, weather=True))
        checks["wind_applied"] = False
        self.assertFalse(passed_checks(checks, weather=True))

    def test_weather_rejects_absent_wrong_or_incomplete_inputs(self):
        """只有完整时间窗、正确实测风速及非平静波面才可通过。"""
        from navigation_acceptance import weather_checks

        import json

        limits = json.loads(
            (
                Path(__file__).resolve().parents[1]
                / "scenarios/environment_acceptance.json"
            ).read_text()
        )
        propulsion = [[i / 10] + [0.0] * 16 for i in range(301)]
        for row in propulsion:
            row[8] = 1.0
        environment = [
            [i / 10, (i % 20) / 100] + [0.0] * 5 + [0.0, 5.0, 0.0] for i in range(301)
        ]
        self.assertTrue(
            all(weather_checks(environment, propulsion, (0, 5, 0), 1, limits).values())
        )
        self.assertFalse(
            weather_checks([], propulsion, (0, 5, 0), 1, limits)["environment_coverage"]
        )
        self.assertFalse(
            weather_checks(environment[:100], propulsion, (0, 5, 0), 1, limits)[
                "environment_coverage"
            ]
        )
        self.assertFalse(
            weather_checks(environment, propulsion, (5, 0, 0), 1, limits)[
                "wind_applied"
            ]
        )
        flat = [r.copy() for r in environment]
        for row in flat:
            row[1] = 0.0
        self.assertFalse(
            weather_checks(flat, propulsion, (0, 5, 0), 1, limits)["wave_observed"]
        )
        propulsion[-1][15] = 0.6
        self.assertFalse(
            weather_checks(environment, propulsion, (0, 5, 0), 1, limits)[
                "attitude_envelope"
            ]
        )
        environment[90][8] = float("nan")
        self.assertFalse(
            any(weather_checks(environment, propulsion, (0, 5, 0), 1, limits).values())
        )


class NavigationLauncherTests(unittest.TestCase):
    """验证长测启动器在磁盘文件变化后仍执行已冻结版本。"""

    def test_running_launcher_survives_source_rewrite(self):
        """替身Docker等待期间覆盖原脚本，不应改变续执行和输入快照。"""
        import os
        import shutil
        import subprocess
        import tempfile
        import time

        project = Path(__file__).resolve().parents[2]
        relative = Path("tools/setup/validate_navigation_containers.sh")
        original = (project / relative).read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in [
                relative,
                Path("tools/lib/navigation_acceptance.py"),
                Path("tests/integration/validate_navigation.py"),
            ]:
                (root / name).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(project / name, root / name)
            config = root / "ros_ws/src/usv_ardupilot/config"
            config.mkdir(parents=True)
            (config / "boat.parm").write_text("FRAME_CLASS 2\n")
            binary = root / "bin"
            binary.mkdir()
            stub = binary / "docker"
            stub.write_text(
                """#!/bin/bash
case "$1 $2" in
  'container inspect') exit 1 ;;
  'image inspect') echo '[]' ;;
  'create --name') echo stub-id ;;
  'start -a') touch "$USV_STUB_READY"; while [[ ! -e "$USV_STUB_RELEASE" ]]; do sleep 0.05; done ;;
  'inspect -f') echo 0 ;;
esac
exit 0
"""
            )
            stub.chmod(0o755)
            ready, release = root / "ready", root / "release"
            environment = {
                **os.environ,
                "PATH": str(binary) + os.pathsep + os.environ["PATH"],
                "USV_STUB_READY": str(ready),
                "USV_STUB_RELEASE": str(release),
            }
            environment.pop("USV_NAV_RUNNER_FROZEN", None)
            environment.pop("USV_NAV_CONFIG_DIR", None)
            process = subprocess.Popen(
                ["bash", str(root / relative), "frozen_test"],
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            try:
                deadline = time.monotonic() + 10
                while (
                    not ready.exists()
                    and time.monotonic() < deadline
                    and process.poll() is None
                ):
                    time.sleep(0.02)
                self.assertTrue(
                    ready.exists(), "Launcher did not reach the blocked Docker stub"
                )
                (root / relative).write_text("#!/bin/bash\nexit 42\n")
                release.touch()
                output, _ = process.communicate(timeout=10)
                self.assertEqual(process.returncode, 0, output)
                self.assertIn("Navigation evidence:", output)
                self.assertEqual(
                    (
                        root / "artifacts/navigation/frozen_test/inputs/runner.sh"
                    ).read_bytes(),
                    original,
                )
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate(timeout=5)
