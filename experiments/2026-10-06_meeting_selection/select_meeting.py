"""A2: does choosing the meeting place with the alias score improve the correction?

Run inside the ROS container (scipy, drone_cslam):
  PYTHONPATH=/workspace/AV_Drone/src/drone_cslam python3 select_meeting.py

Each valid trailing flight offers its same-place pairs inside the zones as candidate
meeting places. A selection rule picks ONE candidate using only GT-free quantities;
the registration at that place (if it passes the gate) is the single inter-drone factor
of an SE(2) pose graph built like drone_cslam.place_recognition. GT is used to fit the
sigma model and thresholds on the design worlds (w1, w2) and to score every outcome.
"""
import ast
import csv
import glob
import json
import math
import os
from multiprocessing import Pool

import numpy as np
import yaml

from drone_cslam.pose_graph import PoseFactor, PosePrior, optimize_pose_graph
from drone_cslam.registration import register_submaps
from drone_cslam.se2 import between

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "../.."))
FLIGHTS = os.path.join(HERE, "../2026-10-06_zoned_trailing_flights")
CONFIG = yaml.safe_load(open(os.path.join(PROJECT, "src/drone_cslam/config/lidar_no_gt_two_regions_converged.yaml")))
DESIGN = ("zoned_corridor_w1", "zoned_corridor_w2")
ALIAS_WINDOWS = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 15.0, 30.0]
K_SIGMA, MIN_WINDOW, MAX_WINDOW = 3.0, 1.5, 6.0
CORRECT_M, CORRECT_DEG = 1.0, 5.0


def load_run(item):
    run, world = item["run"], item["world"]
    base = os.path.join(PROJECT, "artifacts", run, "lidar_registration")
    tracks = {"drone1": [], "drone2": []}
    for r in csv.DictReader(open(os.path.join(base, "trajectories/B0.csv"))):
        tracks[r["vehicle"]].append({"node": int(r["node"]), "t": float(r["timestamp"]),
                                     "raw": [float(r["x"]), float(r["y"]), float(r["yaw"])],
                                     "gt": [float(r["gt_x"]), float(r["gt_y"]), float(r["gt_yaw"])]})
    regs = {}
    for r in csv.DictReader(open(os.path.join(base, "registrations.csv"))):
        regs.setdefault(int(r["pair"]), r)
    return {"run": run, "world": world, "base": base, "tracks": tracks, "regs": regs}


def candidates(data, pairs, alias):
    out = []
    for p in pairs:
        if p["run"] != data["run"] or not p["same_place"] or p["zone"] not in ("RU", "RR", "SU", "SR"):
            continue
        r = data["regs"][p["pair"]]
        out.append({"run": data["run"], "world": data["world"], "pair": p["pair"], "zone": p["zone"],
                    "i": int(r["source_keyframe"]), "j": int(r["target_keyframe"]),
                    "gt_x": p["gt_x"], "progress_m": float(r["target_vehicle_progress_m"]),
                    "initial_err_m": float(r["initial_translation_error_m"]),
                    "alpha": p["alpha"], "alias": alias[f"{data['run']}|{p['pair']}"],
                    "fixed6": {"pose": ast.literal_eval(r["estimated_pose"]), "gate": r["geometry_pass"] == "True",
                               "correct": r["evaluation_pose_match_correct"] == "True",
                               "err_m": float(r["estimated_translation_error_m"]), "window": 6.0}})
    return sorted(out, key=lambda c: c["gt_x"])


def register_adaptive(job):
    base, pair, window = job
    d = np.load(glob.glob(os.path.join(base, "submaps", f"pair_{pair:03d}_*.npz"))[0])
    config = {**CONFIG, "registration": {**CONFIG["registration"], "search_translation_m": window}}
    result = register_submaps(d["source_points"], d["target_points"], d["initial_pose"], config)
    err = between(np.asarray(d["ground_truth_pose"], float), result.pose)
    et, ey = float(np.hypot(err[0], err[1])), abs(math.degrees(err[2]))
    return {"pose": [float(v) for v in result.pose], "gate": bool(result.accepted),
            "correct": et <= CORRECT_M and ey <= CORRECT_DEG, "err_m": et, "window": window}


def solve(data, cand, result):
    """Terminal / RMS relative position error with this single inter-drone factor (or none)."""
    t1, t2 = data["tracks"]["drone1"], data["tracks"]["drone2"]
    nodes = sorted(t1 + t2, key=lambda k: k["node"])
    poses = np.array([k["raw"] for k in nodes])
    if result is not None and result["gate"]:
        g = CONFIG["pose_graph"]
        intra = np.array([g["intra_translation_sigma_m"]] * 2 + [math.radians(g["intra_yaw_sigma_deg"])])
        prior = np.array([g["prior_translation_sigma_m"]] * 2 + [math.radians(g["prior_yaw_sigma_deg"])])
        inter = np.array([g["registration_translation_sigma_m"]] * 2 + [math.radians(g["registration_yaw_sigma_deg"])])
        factors = [PoseFactor(a["node"], b["node"], between(np.array(a["raw"]), np.array(b["raw"])), intra.copy(), "intra_raw_slam")
                   for track in (t1, t2) for a, b in zip(track, track[1:])]
        factors.append(PoseFactor(t1[cand["i"]]["node"], t2[cand["j"]]["node"], np.array(result["pose"]), inter, "meeting"))
        spawn = {v["name"]: np.array(v["spawn"], float) for v in CONFIG["vehicles"]}
        priors = [PosePrior(t1[0]["node"], spawn["drone1"], prior.copy()), PosePrior(t2[0]["node"], spawn["drone2"], prior.copy())]
        opt = optimize_pose_graph(poses, factors, priors, loss=g["robust_loss"], robust_scale=g["robust_scale"],
                                  max_function_evaluations=g["max_function_evaluations"])
        if opt.success:
            poses = opt.poses
    est = {k["node"]: poses[n, :2] for n, k in enumerate(nodes)}
    time1 = np.array([k["t"] for k in t1])
    errors = []
    for b in t2:
        a = t1[int(np.argmin(np.abs(time1 - b["t"])))]
        errors.append(np.linalg.norm((est[b["node"]] - est[a["node"]]) - (np.array(b["gt"][:2]) - np.array(a["gt"][:2]))))
    terminal = np.linalg.norm((est[t2[-1]["node"]] - est[t1[-1]["node"]]) - (np.array(t2[-1]["gt"][:2]) - np.array(t1[-1]["gt"][:2])))
    return {"terminal_m": float(terminal), "rmse_m": float(np.sqrt(np.mean(np.square(errors))))}


def near_window(w):
    return str(min(ALIAS_WINDOWS, key=lambda x: abs(x - w)))


def best_threshold(values, bad):
    """Threshold on a risk score maximising balanced accuracy for flagging accepted-wrong."""
    values, bad = np.asarray(values, float), np.asarray(bad, bool)
    if bad.all() or (~bad).all():
        return float("inf")
    best, best_t = -1.0, float("inf")
    for t in np.unique(values):
        flagged = values >= t
        score = 0.5 * (flagged[bad].mean() + (~flagged)[~bad].mean())
        if score > best:
            best, best_t = score, float(t)
    return best_t


def main():
    runs = json.load(open(os.path.join(FLIGHTS, "runs.json")))["valid"]
    pairs = json.load(open(os.path.join(FLIGHTS, "pairs.json")))
    alias = json.load(open(os.path.join(HERE, "../2026-10-06_sigma_sweep/alias_windows.json")))
    data = [load_run(item) for item in runs]
    cands = {d["run"]: candidates(d, pairs, alias) for d in data}

    # sigma model: relative error grows with drone2 progress; RMS slope fitted on the design worlds only
    design = [c for d in data if d["world"] in DESIGN for c in cands[d["run"]]]
    slope = float(np.sqrt(np.mean([(c["initial_err_m"] / c["progress_m"]) ** 2 for c in design])))
    for cs in cands.values():
        for c in cs:
            c["sigma_hat"] = slope * c["progress_m"]
            c["window"] = float(min(MAX_WINDOW, max(K_SIGMA * c["sigma_hat"], MIN_WINDOW)))
            c["alias_fixed"] = c["alias"]["6.0"]
            c["alias_adaptive"] = c["alias"][near_window(c["window"])]
    jobs = [(d["base"], c["pair"], c["window"]) for d in data for c in cands[d["run"]]]
    with Pool(int(os.environ.get("WORKERS", "24"))) as pool:
        adaptive = pool.map(register_adaptive, jobs, chunksize=2)
    flat = [c for d in data for c in cands[d["run"]]]
    for c, a in zip(flat, adaptive):
        c["adaptive"] = a

    # outcome of meeting at every candidate with either matcher
    by_run = {d["run"]: d for d in data}
    baseline = {d["run"]: solve(d, None, None) for d in data}
    for c in flat:
        for matcher in ("fixed6", "adaptive"):
            c[matcher]["outcome"] = solve(by_run[c["run"]], c, c[matcher])
        print(c["run"][:16], c["pair"], c["zone"], round(c["gt_x"], 1), "w=%.1f" % c["window"],
              "fixed %s/%s %.2f" % (c["fixed6"]["gate"], c["fixed6"]["correct"], c["fixed6"]["outcome"]["terminal_m"]),
              "adapt %s/%s %.2f" % (c["adaptive"]["gate"], c["adaptive"]["correct"], c["adaptive"]["outcome"]["terminal_m"]), flush=True)

    # thresholds from the design worlds, per matcher (risk score high = avoid)
    thresholds = {}
    for matcher, alias_key in (("fixed6", "alias_fixed"), ("adaptive", "alias_adaptive")):
        bad = [c[matcher]["gate"] and not c[matcher]["correct"] for c in design]
        thresholds[matcher] = {"alpha": best_threshold([-c["alpha"] for c in design], bad),
                               "alias": best_threshold([c[alias_key] for c in design], bad)}

    def pick(cs, rule, matcher):
        if rule == "middle":
            return min(cs, key=lambda c: abs(c["gt_x"] - 75.0))
        if rule == "latest":
            return cs[-1]
        if rule == "alpha_safe":
            safe = [c for c in cs if -c["alpha"] < thresholds[matcher]["alpha"]]
        else:
            key = "alias_fixed" if matcher == "fixed6" else "alias_adaptive"
            safe = [c for c in cs if c[key] < thresholds[matcher]["alias"]]
        return safe[-1] if safe else None

    results = []
    for d in data:
        cs = cands[d["run"]]
        for matcher in ("fixed6", "adaptive"):
            for rule in ("middle", "latest", "alpha_safe", "alias_safe"):
                c = pick(cs, rule, matcher)
                if c is None:
                    out, used = baseline[d["run"]], None
                else:
                    out, used = c[matcher]["outcome"], c
                results.append({"run": d["run"], "world": d["world"], "matcher": matcher, "rule": rule,
                                "pair": None if used is None else used["pair"],
                                "gt_x": None if used is None else used["gt_x"],
                                "zone": None if used is None else used["zone"],
                                "gate": None if used is None else used[matcher]["gate"],
                                "correct": None if used is None else used[matcher]["correct"],
                                "terminal_m": out["terminal_m"], "rmse_m": out["rmse_m"]})
            best = min(cs, key=lambda c: c[matcher]["outcome"]["terminal_m"])
            results.append({"run": d["run"], "world": d["world"], "matcher": matcher, "rule": "oracle_best",
                            "pair": best["pair"], "gt_x": best["gt_x"], "zone": best["zone"],
                            "gate": best[matcher]["gate"], "correct": best[matcher]["correct"],
                            "terminal_m": best[matcher]["outcome"]["terminal_m"], "rmse_m": best[matcher]["outcome"]["rmse_m"]})
    json.dump({"sigma_slope_m_per_m": slope, "thresholds": thresholds, "baseline_b0": baseline,
               "candidates": flat, "selections": results},
              open(os.path.join(HERE, "selection.json"), "w"), indent=1)
    print("sigma slope", slope, "thresholds", thresholds, flush=True)


if __name__ == "__main__":
    main()
