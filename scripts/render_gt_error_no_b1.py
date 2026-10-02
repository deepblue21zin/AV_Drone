#!/usr/bin/env python3
"""Replot recorded GT errors, omit B1, and highlight N-double without changing data."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUN = "20260911T161139Z_9abd773a46f1"
SOURCE = ROOT / "artifacts" / RUN / "lidar_registration"
SHOWN = ("B0", "N-prior", "N-single-A", "N-single-B", "N-double")
COLORS = {"B0": "#dc2626", "B1": "#d97706", "N-prior": "#64748b",
          "N-single-A": "#9333ea", "N-single-B": "#0891b2", "N-double": "#2563eb"}


def read_data():
    metrics = json.loads((SOURCE / "metrics.json").read_text())["conditions"]
    data, hashes = {}, {}
    for condition in ("B0", "B1", *SHOWN[1:]):
        path = SOURCE / "trajectories" / (condition + ".csv")
        hashes[condition] = hashlib.sha256(path.read_bytes()).hexdigest()
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        data[condition] = {}
        for vehicle in ("drone1", "drone2"):
            samples = [row for row in rows if row["vehicle"] == vehicle]
            values = {"progress": [], "position": [], "yaw": []}
            for row in samples:
                x, y, yaw, gx, gy, gyaw = (float(row[key]) for key in
                    ("x", "y", "yaw", "gt_x", "gt_y", "gt_yaw"))
                values["progress"].append(float(row["progress_m"]))
                values["position"].append(math.hypot(x-gx, y-gy))
                values["yaw"].append(abs(math.degrees((yaw-gyaw+math.pi) % (2*math.pi)-math.pi)))
            for key, metric in (("position", "ate_translation_m"), ("yaw", "ate_yaw_deg")):
                rmse = math.sqrt(sum(value*value for value in values[key])/len(samples))
                assert math.isclose(rmse, metrics[condition]["vehicles"][vehicle][metric]["rmse"], abs_tol=1e-9)
                values[key+"_rmse"] = rmse
            data[condition][vehicle] = values
    # All conditions use the same original keyframes and raw-SLAM progress axis.
    for vehicle in ("drone1", "drone2"):
        for condition in SHOWN:
            assert data[condition][vehicle]["progress"] == data["B0"][vehicle]["progress"]
    return data, hashes


def render(output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if output.exists():
        raise FileExistsError("Choose a new output folder to preserve previous images")
    output.mkdir(parents=True)
    data, hashes = read_data()
    original_hash = hashlib.sha256((SOURCE / "overview_errors.png").read_bytes()).hexdigest()
    # Recover Matplotlib's original per-panel limits, including B1. Removing a
    # curve must not silently rescale the original drone1 yaw comparison.
    ref, ref_axes = plt.subplots(2, 2, figsize=(14, 7))
    for col, vehicle in enumerate(("drone1", "drone2")):
        for row, key in enumerate(("position", "yaw")):
            for condition in data:
                values = data[condition][vehicle]
                ref_axes[row, col].plot(values["progress"], values[key])
    limits = {(row, col): (axis.get_xlim(), axis.get_ylim())
              for row, axes_row in enumerate(ref_axes) for col, axis in enumerate(axes_row)}
    plt.close(ref)
    rankings = {}

    def draw(axis, row, col, annotate):
        vehicle = ("drone1", "drone2")[col]
        key = ("position", "yaw")[row]
        for condition in SHOWN:
            values = data[condition][vehicle]
            line, = axis.plot(values["progress"], values[key], color=COLORS[condition],
                              label=condition, linewidth=2.25 if condition == "N-double" else 1.5,
                              zorder=4 if condition == "N-double" else 2)
            assert list(line.get_ydata()) == values[key]
            assert list(line.get_xdata()) == values["progress"]
        axis.set(xlim=limits[row, col][0], ylim=limits[row, col][1],
                 title=vehicle if row == 0 else "",
                 ylabel="absolute position error [m]" if row == 0 else "absolute wrapped yaw error [deg]")
        if row == 1:
            axis.set_xlabel("raw-SLAM accumulated distance [m]")
        axis.grid(alpha=.2)
        legend = axis.legend(loc="upper left", fontsize=9)
        for label in legend.get_texts():
            if label.get_text() == "N-double":
                label.set_weight("bold")
                label.set_color(COLORS["N-double"])
        ranking = sorted((data[c][vehicle][key+"_rmse"], c) for c in SHOWN)
        rankings[vehicle+"_"+key] = [{"condition": c, "rmse": value} for value, c in ranking]
        if annotate:
            value = data["N-double"][vehicle][key+"_rmse"]
            unit = "m" if key == "position" else "deg"
            if ranking[0][1] == "N-double":
                label = "N-double: lowest RMSE\n%.3f %s (shown conditions)" % (value, unit)
                edge, face = "#2563eb", "#eff6ff"
            else:
                label = "N-double RMSE: %.3f %s\nN-single-B: %.3f %s (lower)" % (value, unit, ranking[0][0], unit)
                edge, face = "#a16207", "#fffbeb"
            axis.text(.53, .955, label, transform=axis.transAxes, va="top", ha="left",
                      fontsize=9.1, color=edge, fontweight="bold",
                      bbox={"boxstyle": "round,pad=0.4", "facecolor": face, "edgecolor": edge, "alpha": .96},
                      zorder=6)

    for annotate, filename in ((False, "overview_errors_no_b1.png"), (True, "overview_errors_no_b1_highlight.png")):
        figure, axes = plt.subplots(2, 2, figsize=(14, 7))
        for row in (0, 1):
            for col in (0, 1):
                draw(axes[row, col], row, col, annotate)
        figure.tight_layout()
        figure.savefig(output / filename, dpi=200)
        plt.close(figure)
    assert sum(r[0]["condition"] == "N-double" for r in rankings.values()) == 3
    assert hashlib.sha256((SOURCE / "overview_errors.png").read_bytes()).hexdigest() == original_hash
    audit = {"run": RUN, "omitted_condition": "B1", "shown_conditions": SHOWN,
             "ranking_basis": "whole-keyframe GT absolute error RMSE, shown conditions only",
             "rankings": rankings, "minimum_rmse_panels_for_n_double": 3,
             "curve_values_unchanged": True, "original_axis_limits_preserved": True,
             "source_csv_sha256": hashes, "original_graph_sha256": original_hash,
             "streamlit_and_original_files_modified": False}
    (output / "data_audit.json").write_text(json.dumps(audit, indent=2)+"\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    render(parser.parse_args().output)
