#!/usr/bin/env python3
"""导出已完成受力与操纵试验的证据和图表。

Export compact, unmodified evidence and plots from completed local trials.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analyze_force_audit import analyze


def main():
    """从完成的本地试验导出证据与图表。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    manifest = []

    def copy(src, name):
        """复制证据文件并登记原文件哈希。

        Args:
            src: 来源文件路径
            name: 发布目录内的相对目标路径
        """
        target = a.output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, target)
        manifest.append(
            {
                "source": str(src),
                "published": name,
                "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            }
        )

    for name in [
        "resistance_budget.json",
        "geometry_symmetry.json",
        "maneuver_preregistration.json",
        "maneuver_criteria_correction.json",
        "final_source_manifest.json",
        "postprocessing_note.json",
    ]:
        copy(a.input / name, name)
    results = {}
    for label in ["diagnostics", "corrected_diagnostics"]:
        results[label] = {}
        for p in sorted((a.input / label).iterdir()):
            if p.is_dir() and (p / "result.json").exists():
                results[label][p.name] = analyze(p)
                for file in [
                    "result.json",
                    "source_versions.json",
                    "run_status.json",
                    "stages.json",
                    "audit_hull.csv",
                    "audit_propulsion.csv",
                    "audit_appendages.csv",
                    "propulsion_state.csv",
                    "fuel_mass_state.csv",
                ]:
                    copy(p / file, f"{label}/{p.name}/{file}")
    (a.output / "force_audit.json").write_text(json.dumps(results, indent=2) + "\n")
    copy(a.input / "maneuvers/maneuver_assessment.json", "maneuver_assessment.json")
    copy(a.input / "public_cli/result.json", "public_cli.json")
    for file in [
        "maneuver_acceptance.json",
        "assess_maneuvers.py",
        "maneuver_assessment.json",
    ]:
        copy(a.input / "acceptance_v1" / file, "acceptance_v1/" + file)
    for p in sorted((a.input / "maneuvers").iterdir()):
        if p.is_dir() and (p / "result.json").exists():
            for file in [
                "result.json",
                "stages.json",
                "source_versions.json",
                "propulsion_state.csv",
                "actuators_state.csv",
            ]:
                copy(p / file, f"maneuvers/{p.name}/{file}")
    (a.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    names = [
        "nominal",
        "cancel_water_couple",
        "reverse_water_couple",
        "symmetric_mass",
        "symmetric_mass_no_couple",
    ]
    fig, ax = plt.subplots(figsize=(10, 4))
    xx = np.arange(len(names))
    for j, (label, title) in enumerate(
        [
            ("diagnostics", "Before geometry fix"),
            ("corrected_diagnostics", "Corrected geometry + budget"),
        ]
    ):
        ax.bar(
            xx + (j - 0.5) * 0.36,
            [results[label][n]["yaw_rate_deg_s"] for n in names],
            width=0.36,
            label=title,
        )
    ax.set_xticks(
        xx,
        [
            "Normal",
            "No water couple",
            "Reversed couple",
            "Symmetric mass",
            "Symmetric + no couple",
        ],
    )
    ax.set_ylabel("Tail yaw rate [deg/s]")
    ax.axhline(0, color="black", lw=0.6)
    ax.legend()
    fig.tight_layout()
    fig.savefig(a.output / "diagnostic_yaw.png", dpi=140)
    plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for label in ["035", "065", "100"]:
        p = a.input / "maneuvers" / ("neutral_" + label)
        d = np.loadtxt(p / "propulsion_state.csv", delimiter=",")
        s = json.loads((p / "stages.json").read_text())[0]
        d = d[d[:, 0] >= s["start_sim"]]
        axes[0, 0].plot(
            d[:, 0] - d[0, 0],
            np.degrees(np.unwrap(d[:, 14]) - d[0, 14]),
            label=label + " throttle",
        )
    axes[0, 0].set(
        xlabel="Time [s]",
        ylabel="Heading change [deg]",
        title="Direct zero-rudder runs",
    )
    axes[0, 0].legend()
    for name in ["turn_055_left", "turn_055_right", "turn_100_left", "turn_100_right"]:
        p = a.input / "maneuvers" / name
        d = np.loadtxt(p / "propulsion_state.csv", delimiter=",")
        s = json.loads((p / "stages.json").read_text())[1]
        d = d[(d[:, 0] >= s["start_sim"]) & (d[:, 0] <= s["end_sim"])]
        q = d[:, 12:14] - d[0, 12:14]
        h = d[0, 14]
        axes[0, 1].plot(
            q[:, 0] * np.cos(h) + q[:, 1] * np.sin(h),
            -q[:, 0] * np.sin(h) + q[:, 1] * np.cos(h),
            label=name,
        )
    axes[0, 1].set(
        xlabel="Initial forward [m]",
        ylabel="Initial port [m]",
        title="Turning trajectories",
    )
    axes[0, 1].axis("equal")
    axes[0, 1].legend(fontsize=7)
    for name in ["coast_100", "brake_100"]:
        p = a.input / "maneuvers" / name
        d = np.loadtxt(p / "propulsion_state.csv", delimiter=",")
        s = json.loads((p / "stages.json").read_text())[1]
        d = d[(d[:, 0] >= s["start_sim"]) & (d[:, 0] <= s["end_sim"])]
        axes[1, 0].plot(d[:, 0] - d[0, 0], d[:, 10], label=name)
    axes[1, 0].set(
        xlabel="Time after command [s]",
        ylabel="Surge speed [m/s]",
        title="Coast versus reverse braking",
    )
    axes[1, 0].axhline(0, color="black", lw=0.6)
    axes[1, 0].legend()
    p = a.input / "maneuvers/zigzag_10_10"
    d = np.loadtxt(p / "propulsion_state.csv", delimiter=",")
    s = json.loads((p / "stages.json").read_text())[1]
    d = d[d[:, 0] >= s["start_sim"]]
    axes[1, 1].plot(
        d[:, 0] - d[0, 0], np.degrees(np.unwrap(d[:, 14]) - d[0, 14]), label="Heading"
    )
    act = np.loadtxt(p / "actuators_state.csv", delimiter=",")
    act = act[act[:, 0] >= s["start_sim"]]
    axes[1, 1].plot(act[:, 0] - d[0, 0], np.degrees(act[:, 1]), label="Actual rudder")
    axes[1, 1].set(
        xlabel="Time [s]", ylabel="Angle [deg]", title="10/10 heading-triggered zigzag"
    )
    axes[1, 1].legend()
    for ax in axes.flat:
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(a.output / "maneuvers.png", dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    main()
