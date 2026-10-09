"""Three-candidate zone selection: latest safe zone and a two-term cost, against simple baselines.

  python3 select_latest_safe.py                                  # numpy; decisions from drone1 data, outcomes
  (ROS container) python3 select_latest_safe.py --pose-graph     # terminal errors and error curves

Worlds b1-b6 have three candidate zones (x = 24-36, 54-66, 84-96 m) and one flight per zone.
A rule sees only drone1's submaps and raw poses, picks one zone, and is scored by the flight that approached it.
Rule parameters were fixed before the flights (PREREGISTRATION.md):
  ALIAS_THRESHOLD 0.79 from w1/w2; cost parameters from the 16 flights of a1-a8.
"""
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "../2026-10-07_approach_main")
sys.path.insert(0, MAIN)
import analyze_approach as aa  # noqa: E402
az = aa.az

WINDOWS = [(24.0, 36.0), (54.0, 66.0), (84.0, 96.0)]
THRESHOLD = aa.ALIAS_THRESHOLD          # 0.79
COST = {"k_m_per_m": 0.05, "e_safe_m": 0.3, "e_risky_m": 4.0, "x_end_m": 142.0}
RULES = ("latest_safe", "cost", "nearest_safe", "farthest", "nearest", "lowest_alias", "alpha_best", "oracle")


def zone_scores(run):
    base, rows, regs = aa.load(run)
    raw = {(r["vehicle"], int(r["keyframe"])): np.array([float(r["x"]), float(r["y"]), float(r["yaw"])]) for r in rows}
    d1 = {}
    for f in glob.glob(os.path.join(base, "submaps/*.npz")):
        pid = int(os.path.basename(f)[5:8])
        d1[int(regs[pid]["source_keyframe"])] = np.load(f)["source_points"]
    grid = az.Grid(np.vstack([az.T(raw[("drone1", k)], pts) for k, pts in d1.items()]))
    out = []
    for x0, x1 in WINDOWS:
        inside = [(raw[("drone1", k)], pts) for k, pts in sorted(d1.items()) if x0 - 3.0 <= raw[("drone1", k)][0] <= x1 + 3.0]
        out.append({"submaps": len(inside),
                    "alias": float(np.median([az.alias_score(grid, pts, pose, 6.0, 6.0, exclude=1.5) for pose, pts in inside])) if inside else float("nan"),
                    "alpha": float(np.median([az.alpha(pts) for pose, pts in inside])) if inside else float("nan")})
    return out


def predicted_cost(alias, k):
    e = COST["e_safe_m"] if alias < THRESHOLD else COST["e_risky_m"]
    return math.hypot(e, COST["k_m_per_m"] * (COST["x_end_m"] - 0.5 * sum(WINDOWS[k])))


def choose(alias, alpha, term):
    safe = [k for k in range(3) if alias[k] < THRESHOLD]
    return {"latest_safe": max(safe) if safe else None,
            "cost": int(np.argmin([predicted_cost(alias[k], k) for k in range(3)])),
            "nearest_safe": min(safe) if safe else None,
            "farthest": 2, "nearest": 0,
            "lowest_alias": int(np.argmin(alias)), "alpha_best": int(np.argmax(alpha)),
            "oracle": int(np.argmin(term))}


def main():
    runs = json.load(open(os.path.join(HERE, "runs.json")))["valid"]
    if "--pose-graph" in sys.argv:
        sys.path.insert(0, os.path.join(HERE, "../2026-10-09_zone_selection"))
        res = {r["run"]: aa.pose_graph(r) for r in runs}
        json.dump(res, open(os.path.join(HERE, "pose_graph.json"), "w"), indent=1)
        import error_curves
        json.dump({r["run"]: error_curves.curve(r) for r in runs}, open(os.path.join(HERE, "curves.json"), "w"))
        print(json.dumps(res, indent=1))
        return
    pg = json.load(open(os.path.join(HERE, "pose_graph.json")))
    worlds = {}
    for r in runs:
        f = aa.analyse(r)
        f.pop("cands")
        f.update(pg[r["run"]])
        f["zone_scores"] = zone_scores(r["run"])
        worlds.setdefault(r["world"], [None] * 3)[r["approach_zone_index"]] = f
    table = []
    for w, fl in sorted(worlds.items()):
        # nanmean: the evaluator stores drone1 submaps only for candidate pairs, so a flight can have none in a
        # window (b4 z1 and b6 z1 flights had no drone1 submap saved beyond x = 79 m); the other flights cover it
        alias = [float(np.nanmean([f["zone_scores"][k]["alias"] for f in fl])) for k in range(3)]
        alpha = [float(np.nanmean([f["zone_scores"][k]["alpha"] for f in fl])) for k in range(3)]
        term = [f["corrected_terminal_m"] for f in fl]
        types = [f["zone_type"] for f in fl]
        table.append({"world": w, "zone_types": types, "alias": alias, "alpha": alpha,
                      "alias_per_flight": [[f["zone_scores"][k]["alias"] for k in range(3)] for f in fl],
                      "classified_safe": [a < THRESHOLD for a in alias],
                      "classification_correct": [(a < THRESHOLD) == (t == "RU") for a, t in zip(alias, types)],
                      "predicted_cost_m": [predicted_cost(alias[k], k) for k in range(3)],
                      "choice": choose(alias, alpha, term), "terminal_m": term,
                      "b0_terminal_m": [f["b0_terminal_m"] for f in fl],
                      "selected_correct": [bool(f["selected_correct"]) and all(f["selected_correct"]) for f in fl],
                      "has_constraint": [bool(f["selected_pairs"]) for f in fl],
                      "usable": [f["usable"] for f in fl], "same_place": [f["same_place"] for f in fl]})
    summary = {"worlds": len(table), "zones_classified_correctly": int(sum(sum(t["classification_correct"]) for t in table)),
               "no_correction_median_m": float(np.median([np.mean(t["b0_terminal_m"]) for t in table]))}
    for rule in RULES:
        val = [t["terminal_m"][t["choice"][rule]] if t["choice"][rule] is not None else float(np.mean(t["b0_terminal_m"])) for t in table]
        ok = [t["selected_correct"][t["choice"][rule]] if t["choice"][rule] is not None else False for t in table]
        summary[rule] = {"terminal_median_m": float(np.median(val)), "terminal_mean_m": float(np.mean(val)),
                         "constraint_correct": int(sum(ok)), "picks": [None if t["choice"][rule] is None else t["choice"][rule] + 1 for t in table]}
    json.dump({"threshold": THRESHOLD, "cost": COST, "worlds": table, "summary": summary}, open(os.path.join(HERE, "selection.json"), "w"), indent=1)
    for t in table:
        print(t["world"], t["zone_types"], "alias", [round(a, 2) for a in t["alias"]], "cost", [round(c, 1) for c in t["predicted_cost_m"]],
              "| picks", {r: (None if v is None else v + 1) for r, v in t["choice"].items()},
              "| terminal", [round(v, 1) for v in t["terminal_m"]], "| ok", t["selected_correct"])
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
