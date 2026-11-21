#!/usr/bin/env python3
"""为悉尼水域准备指定方案、载荷与环境场景。

Compose the upstream Sydney venue with our boat, without modifying VRX.
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
import xml.etree.ElementTree as E
import json
import hashlib
import os
import argparse
import re
import shutil

parser = argparse.ArgumentParser()
parser.add_argument("--payload-kg", type=float, default=0.0)
parser.add_argument("--payload-center", type=float, nargs=3, default=[0.0, 0.0, 0.6])
parser.add_argument("--fuel-l", type=float, default=60.0)
parser.add_argument(
    "--mass-case", choices=["nominal", "light", "heavy"], default="nominal"
)
parser.add_argument(
    "--power-mode", choices=["normal", "survey", "peak"], default="normal"
)
parser.add_argument(
    "--task",
    choices=["none", "stationkeeping", "wayfinding", "navigation"],
    default="none",
)
parser.add_argument("--output", type=Path)
parser.add_argument(
    "--profile", choices=["baseline", "speed15_concept"], default="baseline"
)
parser.add_argument("--name", default="usv")
parser.add_argument("--second-name")
parser.add_argument("--wave-gain", type=float, default=0.0)
a = parser.parse_args()
if not 0 <= a.fuel_l <= 60:
    parser.error("Fuel must be 0..60 litres")
if not all(abs(v) <= b for v, b in zip(a.payload_center, [2.0, 0.8, 1.2])):
    parser.error("Payload CG outside tested installation envelope")
if not 0 <= a.payload_kg <= 300:
    parser.error("Payload must be 0..300 kg")
names = [a.name] + ([a.second_name] if a.second_name else [])
if len(set(names)) != len(names) or any(
    not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", x) for x in names
):
    parser.error("Boat names must be unique identifiers")
if not 0 <= a.wave_gain <= 1:
    parser.error("Wave gain must be in [0,1] for this approximation")

ROOT = WORKSPACE
source = ROOT / "../dependencies/src/vrx/vrx_gz/worlds/sydney_regatta.sdf"
tree = E.parse(source)
world = tree.getroot().find("world")
world.set("name", "usv_sydney")
world.find("plugin[@name='gz::sim::systems::Physics']").set(
    "filename", "libUsvPhysics.so"
)
# Keep the initial visual wave spectrum and physical wave forcing consistent.
publisher = world.find("plugin[@name='vrx::PublisherPlugin']/message")
publisher.text = publisher.text.replace(
    "double_value: 0.3", f"double_value: {a.wave_gain}"
)
wind = world.find("plugin[@name='vrx::USVWind']")
world.remove(wind)  # upstream targets wamv; this validation is zero wind
assets = {}
for uri in world.findall("include/uri"):
    if uri.text.startswith("https://fuel.gazebosim.org/"):
        name = uri.text.rsplit("/", 1)[-1]
        cache = Path.home() / ".gz/fuel/fuel.gazebosim.org/openrobotics/models" / name
        local = ROOT / "../dependencies/vendor/fuel" / name

        def numeric_directory_name_1(p):
            """把目录名称转换为整数，用于按编号排序。

            Args:
                p: 待排序的 pathlib 路径对象。
            """
            return int(p.name)

        versions = sorted(
            (p for p in cache.glob("*") if (p / "model.sdf").is_file()),
            key=numeric_directory_name_1,
        )
        if (local / "model.sdf").is_file():
            versions.append(local)
        if not versions:
            raise FileNotFoundError(f"Missing cached VRX asset: {name}")
        if not local.exists():
            shutil.copytree(versions[-1], local)
        uri.text = str(local)
        assets[name] = str(local)
    elif uri.text == "coast_waves":
        wave_tree = E.parse(
            ROOT / "../dependencies/src/vrx/vrx_gz/models/coast_waves/model.sdf"
        )
        waves = wave_tree.getroot().find("model")
        waves.set("name", "Coast Waves")
        base = ROOT / "../dependencies/src/vrx/vrx_gz/models/coast_waves"
        for el in (
            list(waves.iter("uri"))
            + list(waves.iter("vertex"))
            + list(waves.iter("fragment"))
        ):
            el.text = str(base / el.text)
        wave = E.SubElement(waves.find(".//wavefield"), "wave")
        E.SubElement(wave, "gain").text = str(a.wave_gain)
        # WaveVisual resolves hard-coded texture names against SourceFilePath.
        # Keep a separate model file (inlining into the world breaks textures).
        calm = ROOT / (
            "src/usv_description/models/coast_waves_gain_"
            + str(a.wave_gain).replace(".", "_")
        )
        calm.mkdir(parents=True, exist_ok=True)
        for folder in ["materials", "meshes"]:
            link = calm / folder
            if not link.exists():
                link.symlink_to(
                    os.path.relpath(base / folder, calm), target_is_directory=True
                )
        E.indent(wave_tree)
        wave_tree.write(calm / "model.sdf", encoding="utf-8", xml_declaration=True)
        uri.text = str(calm / "model.sdf")
for field in world.findall(".//wavefield"):
    wave = field.find("wave")
    if wave is None:
        wave = E.SubElement(field, "wave")
    gain = wave.find("gain")
    if gain is None:
        gain = E.SubElement(wave, "gain")
    gain.text = str(a.wave_gain)
base_trial = (
    E.parse(ROOT / "src/usv_simulation/worlds/usv_hull_trial.sdf")
    .getroot()
    .find("world/include")
)
spawn_poses = {}
profile_reports = {}
for index, boat_name in enumerate(names):
    boat_tree = E.parse(ROOT / "src/usv_description/models/usv_azimuth_trial.sdf")
    boat = boat_tree.getroot().find("model")
    boat.set("name", boat_name)
    boat.find("plugin[@name='usv::AzimuthPropulsion']/energy/initial_fuel_l").text = (
        str(a.fuel_l)
    )
    from scenario_profiles import apply_speed15

    profile_reports[boat_name] = apply_speed15(
        boat, a.payload_kg, a.payload_center, a.mass_case, a.power_mode, a.profile
    )
    for element in boat.iter():
        if element.text:
            element.text = element.text.replace("/usv/", f"/{boat_name}/")
            if element.text.startswith("usv/"):
                element.text = boat_name + element.text[3:]
        if element.tag == "topic_namespace":
            element.text = boat_name
        if element.tag == "gain" and element in boat.findall(".//wavefield/wave/gain"):
            element.text = str(a.wave_gain)
    instance = (
        ROOT
        / "src/usv_description/models/instances"
        / (
            boat_name
            + "_"
            + a.profile
            + "_payload"
            + str(a.payload_kg)
            + "_wave"
            + str(a.wave_gain).replace(".", "_")
        )
    )
    if a.output:
        instance = a.output.resolve().parent / "models" / boat_name
    instance.mkdir(parents=True, exist_ok=True)
    model_path = instance / "usv_azimuth_trial.sdf"
    E.indent(boat_tree)
    boat_tree.write(model_path, encoding="utf-8", xml_declaration=True)
    trial = E.SubElement(world, "include")
    E.SubElement(trial, "uri").text = str(model_path)
    E.SubElement(trial, "name").text = boat_name
    pose = base_trial.find("pose").text.split()
    pose[:2] = ["-480", str(180 + index * 20)]
    if boat_name in profile_reports:
        pose[2:5] = list(map(str, profile_reports[boat_name]["pose_z_roll_pitch"]))
    E.SubElement(trial, "pose").text = " ".join(pose)
    spawn_poses[boat_name] = list(map(float, pose))
if a.task != "none":
    task_world = (
        E.parse(ROOT / "../dependencies/src/vrx/vrx_gz/worlds" / (a.task + "_task.sdf"))
        .getroot()
        .find("world")
    )
    scoring = {
        "stationkeeping": "Stationkeeping",
        "wayfinding": "Wayfinding",
        "navigation": "Navigation",
    }[a.task]
    task = task_world.find("plugin[@name='vrx::" + scoring + "ScoringPlugin']")
    task.find("vehicle").text = names[0]
    world.append(task)
    if a.task == "navigation":
        if task.find("bonus") is None:
            E.SubElement(task, "bonus").text = "1.0"
        course = task_world.find("include[uri='model://short_navigation_course_0']")
        course_source = E.parse(
            ROOT
            / "../dependencies/src/vrx/vrx_gz/models/short_navigation_course_0/model.sdf"
        )
        for uri in course_source.findall(".//include/uri"):
            asset = ROOT / "../dependencies/vendor/fuel" / uri.text.rsplit("/", 1)[-1]
            if not (asset / "model.sdf").exists():
                raise FileNotFoundError(asset)
            uri.text = str(asset / "model.sdf")
        course_file = ROOT / "src/usv_description/models/navigation_course_local.sdf"
        course_source.write(course_file, encoding="utf-8", xml_declaration=True)
        course.find("uri").text = str(course_file)
        world.append(course)
world.find("gui/plugin[@filename='MinimalScene']/camera_pose").text = (
    "-468 168 8 0 0.40 2.35619449"
)
E.SubElement(world, "gravity").text = "0 0 -9.81"
target = a.output if a.output else ROOT / "src/usv_simulation/worlds/usv_sydney.sdf"
target.parent.mkdir(parents=True, exist_ok=True)
E.indent(tree)
tree.write(target, encoding="utf-8", xml_declaration=True)
report = {
    "source": str(source),
    "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "world": str(target),
    "profile": a.profile,
    "task": a.task,
    "profile_details": profile_reports,
    "spawn_pose": spawn_poses[names[0]],
    "boats": spawn_poses,
    "wind_m_s": 0,
    "initial_fuel_l": a.fuel_l,
    "payload_center_m": a.payload_center,
    "mass_case": a.mass_case,
    "power_mode": a.power_mode,
    "wave_gain": a.wave_gain,
    "assets": assets,
    "limitation": "VRX wave-coupled distributed hull pressure, zero steepness/phase; deep-water orbital flow; constant diagonal added mass; no radiation/slamming, damping uncalibrated.",
}
(
    target.parent / "scene.json"
    if a.output
    else ROOT / "../datasets/reference/sydney_scene.json"
).write_text(json.dumps(report, indent=2) + "\n")
print(target)
