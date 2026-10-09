"""Relative position error along the flight, before and after the correction, for all 16 approach flights.

  (ROS container) python3 error_curves.py     # writes curves.json

Same pose graph as ../2026-10-07_approach_main/analyze_approach.py (accepted factor only). For every drone2
keyframe the error is |estimated (drone2 - drone1) - true (drone2 - drone1)| against the drone1 keyframe
closest in time. Used to see why the meeting position matters for the terminal error.
"""
import json
import math
import os

import numpy as np
import yaml

from drone_cslam.pose_graph import PoseFactor, PosePrior, optimize_pose_graph
from drone_cslam.se2 import between

import select_zone as sz

aa = sz.aa
HERE = sz.HERE
CONFIG = yaml.safe_load(open(os.path.join(aa.PROJECT, "src/drone_cslam/config/lidar_no_gt_two_regions_converged.yaml")))


def curve(item):
    g = CONFIG["pose_graph"]
    base, rows, regs = aa.load(item["run"])
    tracks = {"drone1": [], "drone2": []}
    for r in rows:
        tracks[r["vehicle"]].append({"node": int(r["node"]), "t": float(r["timestamp"]),
                                     "raw": np.array([float(r["x"]), float(r["y"]), float(r["yaw"])]),
                                     "gt": np.array([float(r["gt_x"]), float(r["gt_y"])])})
    t1, t2 = tracks["drone1"], tracks["drone2"]
    nodes = sorted(t1 + t2, key=lambda k: k["node"])
    poses = np.array([k["raw"] for k in nodes])
    selected = [r for r in regs.values() if r["accepted"] == "True"]
    corrected = poses
    if selected:
        intra = np.array([g["intra_translation_sigma_m"]] * 2 + [math.radians(g["intra_yaw_sigma_deg"])])
        prior = np.array([g["prior_translation_sigma_m"]] * 2 + [math.radians(g["prior_yaw_sigma_deg"])])
        inter = np.array([g["registration_translation_sigma_m"]] * 2 + [math.radians(g["registration_yaw_sigma_deg"])])
        factors = [PoseFactor(a["node"], b["node"], between(a["raw"], b["raw"]), intra.copy(), "intra")
                   for tr in (t1, t2) for a, b in zip(tr, tr[1:])]
        for r in selected:
            factors.append(PoseFactor(t1[int(r["source_keyframe"])]["node"], t2[int(r["target_keyframe"])]["node"],
                                      np.array(json.loads(r["estimated_pose"])), inter.copy(), "meeting"))
        spawn = {v["name"]: np.array(v["spawn"], float) for v in CONFIG["vehicles"]}
        priors = [PosePrior(t1[0]["node"], spawn["drone1"], prior.copy()), PosePrior(t2[0]["node"], spawn["drone2"], prior.copy())]
        opt = optimize_pose_graph(poses, factors, priors, loss=g["robust_loss"], robust_scale=g["robust_scale"],
                                  max_function_evaluations=g["max_function_evaluations"])
        if opt.success:
            corrected = opt.poses
    index = {k["node"]: n for n, k in enumerate(nodes)}
    time1 = np.array([k["t"] for k in t1])
    points = []
    for b in t2:
        a = t1[int(np.argmin(np.abs(time1 - b["t"])))]
        true = b["gt"] - a["gt"]
        err = [float(np.linalg.norm((p[index[b["node"]], :2] - p[index[a["node"]], :2]) - true)) for p in (poses, corrected)]
        points.append({"x": round(float(b["gt"][0]), 2), "y": round(float(b["gt"][1]), 2), "b0": round(err[0], 3), "corrected": round(err[1], 3)})
    meeting = [{"drone1": [round(float(v), 2) for v in t1[int(r["source_keyframe"])]["gt"]],
                "drone2": [round(float(v), 2) for v in t2[int(r["target_keyframe"])]["gt"]],
                "correct": r["evaluation_pose_match_correct"] == "True",
                "err_m": round(float(r["estimated_translation_error_m"]), 2)} for r in selected]
    return {"world": item["world"], "approach_zone_index": item["approach_zone_index"], "zone_type": item["zone_type"],
            "drone1_path": [[round(float(k["gt"][0]), 2), round(float(k["gt"][1]), 2)] for k in t1],
            "points": points, "meeting": meeting}


if __name__ == "__main__":
    out = {r["run"]: curve(r) for r in sz.load_runs()}
    json.dump(out, open(os.path.join(HERE, "curves.json"), "w"))
    for run, c in out.items():
        print(c["world"], "z%d" % (c["approach_zone_index"] + 1), c["zone_type"], "terminal b0 %.1f corrected %.1f" % (c["points"][-1]["b0"], c["points"][-1]["corrected"]),
              "meeting", [(m["drone2"][0], m["correct"], m["err_m"]) for m in c["meeting"]])
