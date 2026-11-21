#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
blender_bin="${BLENDER_BIN:-blender}"
for script in build_second.py polish_second.py detail_steering.py complete_concept.py verify_gears.py verify_concept.py render_concept.py; do
  "$blender_bin" -b --python-exit-code 1 --python "$script_dir/$script"
done
python -m unittest discover -s "$script_dir/../仿真接口" -v
