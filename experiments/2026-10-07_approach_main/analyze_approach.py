"""Approach main experiment: score the four pre-registered criteria (see PREREGISTRATION.md).

  python3 analyze_approach.py            # numpy; candidates, prediction from drone1 data only
  (ROS container) python3 analyze_approach.py --pose-graph   # adds terminal error with the accepted factor(s)

Inputs: runs.json (this folder), sim_assets/worlds/zoned_corridor_a*_layout.json,
artifacts/<run>/lidar_registration/{registrations.csv, submaps/*.npz, trajectories/B0.csv}.
GT is used only for the same-place test and for scoring; the prediction uses drone1's raw poses and submaps.
"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "../.."))
sys.path.insert(0, os.path.join(HERE, "../2026-10-06_zoned_trailing_flights"))
import analyze_zones as az  # noqa: E402  (alias/alpha definitions of the earlier studies)

ALIAS_THRESHOLD = 0.79      # fixed on w1/w2, 2026-10-06
DIP_Y_MAX = 3.5             # drone2 keyframes below this y belong to the approach
SAME_PLACE = 0.5


def overlap(a, b, r=0.6):
    if len(a) == 0 or len(b) == 0:
        return 0.0
    return float((np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)).min(1) < r).mean())


def load(run):
    base = os.path.join(PROJECT, "artifacts", run, "lidar_registration")
    rows = list(csv.DictReader(open(os.path.join(base, "trajectories/B0.csv"))))
    regs = {}
    for r in csv.DictReader(open(os.path.join(base, "registrations.csv"))):
        regs.setdefault(int(r["pair"]), r)
    return base, rows, regs


def analyse(item):
    run, x0, x1 = item["run"], *item["approach_world_x"]
    base, rows, regs = load(run)
    gt = {(r["vehicle"], int(r["keyframe"])): np.array([float(r["gt_x"]), float(r["gt_y"]), float(r["gt_yaw"])]) for r in rows}
    raw = {(r["vehicle"], int(r["keyframe"])): np.array([float(r["x"]), float(r["y"]), float(r["yaw"])]) for r in rows}
    subs = {int(os.path.basename(f)[5:8]): np.load(f) for f in glob.glob(os.path.join(base, "submaps/*.npz"))}

    # --- prediction from drone1 only: its submaps placed at its own (raw) poses
    d1 = {}
    for pid, d in subs.items():
        d1[int(regs[pid]["source_keyframe"])] = d["source_points"]
    grid = az.Grid(np.vstack([az.T(raw[("drone1", k)], pts) for k, pts in d1.items()]))
    scores = []
    for k, pts in sorted(d1.items()):
        pose = raw[("drone1", k)]
        if x0 - 3.0 <= pose[0] <= x1 + 3.0:
            scores.append({"keyframe": k, "raw_x": float(pose[0]), "alias": az.alias_score(grid, pts, pose, 6.0, 6.0, exclude=1.5),
                           "alpha": az.alpha(pts)})
    alias_med = float(np.median([s["alias"] for s in scores])) if scores else float("nan")
    alpha_med = float(np.median([s["alpha"] for s in scores])) if scores else float("nan")

    # --- candidates made while drone2 was approaching
    cands = []
    for pid, r in sorted(regs.items()):
        if pid not in subs:
            continue
        t = gt[("drone2", int(r["target_keyframe"]))]
        if not (t[1] < DIP_Y_MAX and t[0] > 15.0):
            continue
        d = subs[pid]
        g = overlap(az.T(d["ground_truth_pose"], d["target_points"]), d["source_points"])
        err = float(r["estimated_translation_error_m"])
        ok, gate = r["evaluation_pose_match_correct"] == "True", r["geometry_pass"] == "True"
        cands.append({"pair": pid, "same": g >= SAME_PLACE, "gt_overlap": g, "correct": ok, "gate": gate,
                      "err_m": err, "initial_err_m": float(r["initial_translation_error_m"]),
                      "alias_error": (not ok) and 3.0 <= err <= 5.5, "selected": r["accepted"] == "True",
                      "reasons": r["rejection_reasons"]})
    same = [c for c in cands if c["same"]]
    drift = {}
    for v in ("drone1", "drone2"):
        keys = [k for k in gt if k[0] == v]
        k = min(keys, key=lambda q: abs(gt[q][0] - 0.5 * (x0 + x1)))
        drift[v] = float(np.linalg.norm(raw[k][:2] - gt[k][:2]))
    return {**item, "candidates": len(cands), "same_place": len(same),
            "usable": sum(c["correct"] and c["gate"] for c in same),
            "alias_errors": sum(c["alias_error"] for c in same),
            "alias_errors_passed_gate": sum(c["alias_error"] and c["gate"] for c in same),
            "wrong_passed_gate_all": sum((not c["correct"]) and c["gate"] for c in cands),
            "initial_err_median_m": float(np.median([c["initial_err_m"] for c in same])) if same else float("nan"),
            "correct_err_median_m": float(np.median([c["err_m"] for c in same if c["correct"]])) if any(c["correct"] for c in same) else float("nan"),
            "drift_at_approach_m": drift,
            "pred_alias_median": alias_med, "pred_alpha_median": alpha_med, "pred_submaps": len(scores),
            "predicted": "risky" if alias_med >= ALIAS_THRESHOLD else "safe",
            "prediction_correct": (alias_med >= ALIAS_THRESHOLD) == (item["zone_type"] == "RR"),
            "cands": cands}


def pose_graph(item):
    import yaml
    from drone_cslam.pose_graph import PoseFactor, PosePrior, optimize_pose_graph
    from drone_cslam.se2 import between
    config = yaml.safe_load(open(os.path.join(PROJECT, "src/drone_cslam/config/lidar_no_gt_two_regions_converged.yaml")))
    g = config["pose_graph"]
    base, rows, regs = load(item["run"])
    tracks = {"drone1": [], "drone2": []}
    for r in rows:
        tracks[r["vehicle"]].append({"node": int(r["node"]), "t": float(r["timestamp"]),
                                     "raw": np.array([float(r["x"]), float(r["y"]), float(r["yaw"])]),
                                     "gt": np.array([float(r["gt_x"]), float(r["gt_y"])])})
    t1, t2 = tracks["drone1"], tracks["drone2"]
    nodes = sorted(t1 + t2, key=lambda k: k["node"])
    poses = np.array([k["raw"] for k in nodes])

    def terminal(p):
        est = {k["node"]: p[n, :2] for n, k in enumerate(nodes)}
        return float(np.linalg.norm((est[t2[-1]["node"]] - est[t1[-1]["node"]]) - (t2[-1]["gt"] - t1[-1]["gt"])))

    out = {"b0_terminal_m": terminal(poses)}
    selected = [r for r in regs.values() if r["accepted"] == "True"]
    out["selected_pairs"] = [int(r["pair"]) for r in selected]
    out["selected_correct"] = [r["evaluation_pose_match_correct"] == "True" for r in selected]
    if selected:
        intra = np.array([g["intra_translation_sigma_m"]] * 2 + [math.radians(g["intra_yaw_sigma_deg"])])
        prior = np.array([g["prior_translation_sigma_m"]] * 2 + [math.radians(g["prior_yaw_sigma_deg"])])
        inter = np.array([g["registration_translation_sigma_m"]] * 2 + [math.radians(g["registration_yaw_sigma_deg"])])
        factors = [PoseFactor(a["node"], b["node"], between(a["raw"], b["raw"]), intra.copy(), "intra")
                   for tr in (t1, t2) for a, b in zip(tr, tr[1:])]
        for r in selected:
            factors.append(PoseFactor(t1[int(r["source_keyframe"])]["node"], t2[int(r["target_keyframe"])]["node"],
                                      np.array(json.loads(r["estimated_pose"])), inter.copy(), "meeting"))
        spawn = {v["name"]: np.array(v["spawn"], float) for v in config["vehicles"]}
        priors = [PosePrior(t1[0]["node"], spawn["drone1"], prior.copy()), PosePrior(t2[0]["node"], spawn["drone2"], prior.copy())]
        opt = optimize_pose_graph(poses, factors, priors, loss=g["robust_loss"], robust_scale=g["robust_scale"],
                                  max_function_evaluations=g["max_function_evaluations"])
        out["corrected_terminal_m"] = terminal(opt.poses if opt.success else poses)
    else:
        out["corrected_terminal_m"] = out["b0_terminal_m"]
    return out


def main():
    runs = json.load(open(os.path.join(HERE, "runs.json")))["valid"]
    if "--pose-graph" in sys.argv:
        res = {item["run"]: pose_graph(item) for item in runs}
        json.dump(res, open(os.path.join(HERE, "pose_graph.json"), "w"), indent=1)
        print(json.dumps(res, indent=1))
        return
    flights = [analyse(item) for item in runs]
    pg = json.load(open(os.path.join(HERE, "pose_graph.json"))) if os.path.exists(os.path.join(HERE, "pose_graph.json")) else {}
    for f in flights:
        f.update(pg.get(f["run"], {}))
    pooled = {}
    for z in ("RU", "RR"):
        s = [f for f in flights if f["zone_type"] == z]
        n = sum(f["same_place"] for f in s)
        pooled[z] = {"flights": len(s), "same_place": n, "usable": sum(f["usable"] for f in s),
                     "usable_rate": sum(f["usable"] for f in s) / n if n else float("nan"),
                     "alias_errors": sum(f["alias_errors"] for f in s),
                     "alias_errors_passed_gate": sum(f["alias_errors_passed_gate"] for f in s),
                     "corrected_terminal_median_m": float(np.median([f["corrected_terminal_m"] for f in s])) if pg else None,
                     "b0_terminal_median_m": float(np.median([f["b0_terminal_m"] for f in s])) if pg else None}
    verdict = {"1_yield_rr_lower": pooled["RR"]["usable_rate"] < pooled["RU"]["usable_rate"],
               "2_prediction_hits": sum(f["prediction_correct"] for f in flights),
               "2_prediction_ok": sum(f["prediction_correct"] for f in flights) >= 7,
               "3_alias_only_in_rr": pooled["RU"]["alias_errors"] == 0 and pooled["RR"]["alias_errors"] > 0,
               "4_terminal_ru_lower": (pooled["RU"]["corrected_terminal_median_m"] < pooled["RR"]["corrected_terminal_median_m"]) if pg else None}
    json.dump({"flights": flights, "pooled": pooled, "verdict": verdict, "alias_threshold": ALIAS_THRESHOLD},
              open(os.path.join(HERE, "summary.json"), "w"), indent=1)
    for f in flights:
        print(f["world"], "z%d" % (f["approach_zone_index"] + 1), f["zone_type"], "| same", f["same_place"], "usable", f["usable"],
              "alias", f["alias_errors"], "(passed", f["alias_errors_passed_gate"], ") | init err %.1f" % f["initial_err_median_m"],
              "| pred alias %.2f alpha %.2f ->" % (f["pred_alias_median"], f["pred_alpha_median"]), f["predicted"], f["prediction_correct"],
              "| terminal B0 %s -> %s" % (f.get("b0_terminal_m"), f.get("corrected_terminal_m")))
    print(json.dumps({"pooled": pooled, "verdict": verdict}, indent=1))


if __name__ == "__main__":
    main()
