"""验证单机方位推进力接口的方向、力矩和限幅。"""

import math
import unittest
from vector_thrust import wrench, steering_target, slew_angle


class InterfaceTests(unittest.TestCase):
    """验证方位推进接口的符号、力矩和约束。"""

    def test_straight(self):
        """验证零转角直推的力与力矩。"""
        self.assertEqual(wrench(200, 0, (0, 0)).yaw_nm, 0)

    def test_right_turn_and_reverse(self):
        """验证右转与反推的方向约定。"""
        f = wrench(200, steering_target(1), (0, 0))
        r = wrench(-200, 35, (0, 0))
        self.assertGreater(f.as_frd().yaw_nm, 0)
        self.assertAlmostEqual(r.yaw_nm, -f.yaw_nm)
        self.assertAlmostEqual(math.hypot(f.fx_n, f.fy_n), 200)

    def test_cg_offset_moment(self):
        """验证重心偏置产生的力矩。"""
        self.assertAlmostEqual(wrench(100, 0, (0, 0.2)).yaw_nm, 20)

    def test_actuator_rate_and_limits(self):
        """验证执行器速率和角度限制。"""
        self.assertEqual(steering_target(2), 35)
        self.assertEqual(slew_angle(0, 35, 0.2, 10), 2)
        self.assertEqual(slew_angle(34, 90, 2, 10), 35)
        self.assertEqual(slew_angle(5, -5, 0, 10), 5)

    def test_invalid(self):
        """验证非法输入被拒绝。"""
        for thrust, angle in [(math.nan, 0), (0, 36), (math.inf, 0)]:
            with self.assertRaises(ValueError):
                wrench(thrust, angle, (0, 0))
        with self.assertRaises(ValueError):
            slew_angle(0, 0, -1, 10)


if __name__ == "__main__":
    unittest.main()
