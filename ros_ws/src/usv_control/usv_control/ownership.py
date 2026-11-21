"""通过独占租约限制同船同时存在多个控制端。

One provided actuator-command source per boat and Gazebo partition.
"""

import fcntl
import hashlib
import os
import tempfile
from pathlib import Path


class ControlLease:
    """通过独占锁约束同一艘船的控制权。"""

    def __init__(self, boat, owner):
        """为控制端申请当前船舶的独占使用权。

        Args:
            boat: 船舶标识，用于区分控制锁。
            owner: 控制端标识，用于记录持有者。
        """
        partition = os.environ.get("GZ_PARTITION", "default")
        token = hashlib.sha256(
            (str(os.getuid()) + "|" + partition + "|" + boat).encode()
        ).hexdigest()[:24]
        self.path = Path(tempfile.gettempdir()) / ("usv-control-" + token + ".lock")
        self.file = self.path.open("a+")
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.seek(0)
            other = self.file.read().strip()
            self.file.close()
            raise RuntimeError(
                "Actuator controller already active: "
                + other
                + ". Stop it or launch interfaces with control:=False."
            )
        self.file.seek(0)
        self.file.truncate()
        self.file.write(f"{owner} pid={os.getpid()} boat={boat}")
        self.file.flush()

    def close(self):
        """释放当前实例持有的控制权。"""
        if not self.file.closed:
            fcntl.flock(self.file, fcntl.LOCK_UN)
            self.file.close()
