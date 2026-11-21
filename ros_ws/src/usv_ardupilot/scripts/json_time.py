"""在相邻物理观测间插值并限制飞控推进步长。

Bound AP's simulated time advance, interpolating between observed physics states.

The transport must acknowledge every output before sending the next step.
"""

import numpy as np
from scipy.spatial.transform import Rotation


class StepLimiter:
    """在观测物理状态之间插值，限制飞控单次仿真时间推进。"""

    def __init__(self, max_step=0.02):
        """初始化时间步长限制器。

        Args:
            max_step: 最大推进步长，单位 s，范围大于零且不超过 0.02。
        """
        if not 0 < max_step <= 0.02:
            raise ValueError("Maximum step must be 0..20 ms")
        self.max_step = max_step
        self.last = None
        self.interpolated = 0
        self.max_delta = 0.0

    def next(self, target):
        """返回下一段有界物理状态；重复时刻返回空值，回退时刻报错。

        Args:
            target: 目标观测状态，包含时间戳、位置、速度、四元数、IMU 与电池信息。
        """
        if self.last is None:
            self.last = target
            return target
        delta = target["timestamp"] - self.last["timestamp"]
        if delta < -1e-9:
            raise RuntimeError(
                "Gazebo reset detected: restart bridge and SITL together"
            )
        if delta <= 1e-9:
            return None
        if delta <= self.max_step + 1e-9:
            result = target
        else:
            a = self.max_step / delta
            old = self.last
            result = dict(target)
            result["timestamp"] = old["timestamp"] + self.max_step

            def mix(x, y):
                """按当前时间比例对向量进行线性插值。

                Args:
                    x: 上一观测的向量分量。
                    y: 目标观测的向量分量。
                """
                return ((1 - a) * np.array(x) + a * np.array(y)).tolist()

            for key in ["position", "velocity"]:
                result[key] = mix(old[key], target[key])
            result["imu"] = {
                key: mix(old["imu"][key], target["imu"][key])
                for key in ["gyro", "accel_body"]
            }
            result["battery"] = {
                key: (1 - a) * old["battery"][key] + a * target["battery"][key]
                for key in ["voltage", "current"]
            }

            def rot(q):
                """将标量在前的四元数转换为 SciPy 旋转对象。

                Args:
                    q: 按 w、x、y、z 排列的四元数。
                """
                return Rotation.from_quat([q[1], q[2], q[3], q[0]])

            r0, r1 = rot(old["quaternion"]), rot(target["quaternion"])
            q = (r0 * Rotation.from_rotvec((r0.inv() * r1).as_rotvec() * a)).as_quat()
            result["quaternion"] = [q[3], *q[:3]]
            self.interpolated += 1
        self.max_delta = max(
            self.max_delta, result["timestamp"] - self.last["timestamp"]
        )
        self.last = result
        return result
