#!/usr/bin/env python3
"""下载并组织悉尼场景所需的外部资源。

Fetch pinned official Sydney ZIP; ranges, length and all ZIP CRCs are checked.
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
from concurrent.futures import ThreadPoolExecutor, as_completed
import requests
import time
import zipfile
import hashlib
import json

root = WORKSPACE
url = "https://fuel.gazebosim.org/1.0/openrobotics/models/sydney_regatta/3/sydney_regatta.zip"
old = (
    Path.home()
    / ".gz/fuel/fuel.gazebosim.org/openrobotics/models/sydney_regatta/3/sydney_regatta.zip"
)
size = 128980600
parts = root / "../dependencies/vendor/fuel/download_parts"
parts.mkdir(parents=True, exist_ok=True)
start = old.stat().st_size if old.exists() else 0


# The official cached ZIP is an interrupted prefix; full ZIP CRC verifies reuse.
def fetch(pair):
    """下载指定范围的资产数据并写入分块文件。

    Args:
        pair: 下载范围的起止边界
    """
    a, b = pair
    p = parts / str(a)
    if p.exists() and p.stat().st_size == b - a + 1:
        return p
    for attempt in range(4):
        try:
            r = requests.get(url, headers={"Range": f"bytes={a}-{b}"}, timeout=(20, 90))
            r.raise_for_status()
            if r.status_code != 206 or len(r.content) != b - a + 1:
                raise ValueError("invalid range response")
            p.write_bytes(r.content)
            return p
        except Exception:
            if attempt == 3:
                raise
            time.sleep(1 + attempt)


ranges = [(a, min(a + 524287, size - 1)) for a in range(start, size, 524288)]
with ThreadPoolExecutor(max_workers=32) as pool:
    for i, f in enumerate(
        as_completed([pool.submit(fetch, pair) for pair in ranges]), 1
    ):
        f.result()
        if i % 10 == 0:
            print(f"{i}/{len(ranges)} chunks", flush=True)
archive = root / "../dependencies/vendor/fuel/sydney_regatta_complete.zip"
with archive.open("wb") as f:
    if start:
        f.write(old.read_bytes())
    for a, b in ranges:
        f.write((parts / str(a)).read_bytes())
expected = "ccf768134ff7e7929f67970fd059c0286569bc22ead9d959d45061a5c1ee967a"
if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
    raise ValueError("Official v3 ZIP differs from recorded source SHA-256")
with zipfile.ZipFile(archive) as z:
    bad = z.testzip()
    if bad:
        raise ValueError("CRC failed: " + bad)
    target = root / "../dependencies/vendor/fuel/sydney_regatta"
    for info in z.infolist():
        if not (target / info.filename).resolve().is_relative_to(target.resolve()):
            raise ValueError("unsafe archive path")
    z.extractall(target)
(root / "../dependencies/vendor/fuel/SOURCE.json").write_text(
    json.dumps(
        {
            "url": url,
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            "bytes": size,
            "zip_crc_passed": True,
        },
        indent=2,
    )
)
print("EXTRACTED VERIFIED SYDNEY", flush=True)
