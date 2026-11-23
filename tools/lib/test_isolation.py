"""为同一主机用户的原生测试租用互斥 ROS 域，防止不同套件命令串扰。"""

import fcntl
import os
from pathlib import Path
import tempfile


def reserve_ros_domain(directory=None):
    """申请测试专用域并返回必须保持打开的文件锁。

    使用 120–159 域，避开本项目其他固定入口；调用方必须在所有子进程
    清理后才关闭租约。锁随进程退出自动释放，但不能管理外部非协作节点。

    Args:
        directory: 可选锁目录；默认同用户共享的临时目录，测试可用独立目录。

    Returns:
        ROS 域号与持有排他锁的文件对象；域号必须写入运行版本记录。

    Raises:
        RuntimeError: 全部测试域已被其他协作测试占用。
    """
    root = (
        Path(directory)
        if directory is not None
        else Path(tempfile.gettempdir()) / f"usv-ros-domains-{os.getuid()}"
    )
    root.mkdir(parents=True, exist_ok=True)
    for domain in range(120, 160):
        lease = (root / f"{domain}.lock").open("a+")
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lease.close()
            continue
        lease.seek(0)
        lease.truncate()
        lease.write(str(os.getpid()) + "\n")
        lease.flush()
        return domain, lease
    raise RuntimeError("All native-test ROS domains are occupied")
