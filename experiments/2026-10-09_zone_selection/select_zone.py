"""Zone selection: does approaching the zone with the lower alias score give a better correction?

  python3 select_zone.py                                   # numpy; decisions from drone1 data, paired outcomes
  (ROS container) python3 select_zone.py --pose-graph      # terminal error of the new flights (pose_graph.json)

Every approach world was flown twice, once approaching candidate zone 1 (x = 24-36 m) and once zone 2
(x = 54-66 m). A selection rule looks only at drone1's submaps and raw poses (no GT, no drone2 data),
picks one of the two zones, and is scored by the flight that actually approached that zone.
Worlds a1-a4 come from ../2026-10-07_approach_main (already analysed, exploratory here);
worlds a5-a8 are listed in runs.json of this folder (flown after PREREGISTRATION.md).
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "../2026-10-07_approach_main")
sys.path.insert(0, MAIN)
import analyze_approach as aa  # noqa: E402
az = aa.az

WINDOWS = [(24.0, 36.0), (54.0, 66.0)]
RULES = ("alias", "alpha", "near", "far", "oracle", "worst")


def zone_scores(run):
    """Median alias score and alpha of drone1's submaps in each candidate window, from drone1 data only."""
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
                    "alias": float(np.median([az.alias_score(grid, pts, pose, 6.0, 6.0, exclude=1.5) for pose, pts in inside])),
                    "alpha": float(np.median([az.alpha(pts) for pose, pts in inside]))})
    return out


def load_runs():
    runs = [dict(r, batch="a1-a4 (exploratory)") for r in json.load(open(os.path.join(MAIN, "runs.json")))["valid"]]
    own = os.path.join(HERE, "runs.json")
    if os.path.exists(own):
        runs += [dict(r, batch="a5-a8 (pre-registered)") for r in json.load(open(own))["valid"]]
    return runs


def main():
    runs = load_runs()
    if "--pose-graph" in sys.argv:
        new = [r for r in runs if r["batch"].startswith("a5")]
        res = {r["run"]: aa.pose_graph(r) for r in new}
        json.dump(res, open(os.path.join(HERE, "pose_graph.json"), "w"), indent=1)
        print(json.dumps(res, indent=1))
        return
    pg = json.load(open(os.path.join(MAIN, "pose_graph.json")))
    if os.path.exists(os.path.join(HERE, "pose_graph.json")):
        pg.update(json.load(open(os.path.join(HERE, "pose_graph.json"))))
    worlds = {}
    for r in runs:
        f = aa.analyse(r)
        f.pop("cands")
        f.update(pg.get(r["run"], {}))
        f["zone_scores"] = zone_scores(r["run"])
        worlds.setdefault(r["world"], {"batch": r["batch"], "flights": [None, None]})["flights"][r["approach_zone_index"]] = f
    table = []
    for w, entry in sorted(worlds.items()):
        fl = entry["flights"]
        if None in fl or any("corrected_terminal_m" not in f for f in fl):
            continue
        # each flight's drone1 map covers both windows; the decision uses the mean of the two flights
        alias = [float(np.mean([f["zone_scores"][k]["alias"] for f in fl])) for k in (0, 1)]
        alpha = [float(np.mean([f["zone_scores"][k]["alpha"] for f in fl])) for k in (0, 1)]
        per_flight_alias_choice = [int(np.argmin([f["zone_scores"][k]["alias"] for k in (0, 1)])) for f in fl]
        term = [f["corrected_terminal_m"] for f in fl]
        choice = {"alias": int(np.argmin(alias)), "alpha": int(np.argmax(alpha)), "near": 0, "far": 1,
                  "oracle": int(np.argmin(term)), "worst": int(np.argmax(term))}
        table.append({"world": w, "batch": entry["batch"], "zone_types": [f["zone_type"] for f in fl],
                      "alias": alias, "alpha": alpha, "alias_choice_agrees_between_flights": len(set(per_flight_alias_choice)) == 1,
                      "choice": choice, "terminal_m": term, "b0_terminal_m": [f["b0_terminal_m"] for f in fl],
                      "selected_correct": [bool(f["selected_correct"]) and all(f["selected_correct"]) for f in fl],
                      "usable": [f["usable"] for f in fl], "same_place": [f["same_place"] for f in fl]})
    summary = {}
    for batch in sorted({t["batch"] for t in table}) + ["all"]:
        rows = [t for t in table if batch == "all" or t["batch"] == batch]
        s = {"worlds": len(rows)}
        for rule in RULES:
            pick = [t["choice"][rule] for t in rows]
            s[rule] = {"terminal_median_m": float(np.median([t["terminal_m"][k] for t, k in zip(rows, pick)])),
                       "terminal_mean_m": float(np.mean([t["terminal_m"][k] for t, k in zip(rows, pick)])),
                       "constraint_correct": int(sum(t["selected_correct"][k] for t, k in zip(rows, pick))),
                       "picked_unique_zone": int(sum(t["zone_types"][k] == "RU" for t, k in zip(rows, pick)))}
        s["no_correction_median_m"] = float(np.median([np.mean(t["b0_terminal_m"]) for t in rows]))
        s["alias_better_than_other_zone"] = int(sum(t["terminal_m"][t["choice"]["alias"]] < t["terminal_m"][1 - t["choice"]["alias"]] for t in rows))
        s["paired_gain_median_m"] = float(np.median([t["terminal_m"][1 - t["choice"]["alias"]] - t["terminal_m"][t["choice"]["alias"]] for t in rows]))
        summary[batch] = s
    json.dump({"worlds": table, "summary": summary}, open(os.path.join(HERE, "selection.json"), "w"), indent=1)
    for t in table:
        a = t["choice"]["alias"]
        print(t["world"], t["zone_types"], "alias %.2f/%.2f" % tuple(t["alias"]), "alpha %.2f/%.2f" % tuple(t["alpha"]),
              "| alias->z%d alpha->z%d" % (a + 1, t["choice"]["alpha"] + 1), "| terminal %.1f/%.1f" % tuple(t["terminal_m"]),
              "| constraint ok", t["selected_correct"], "| agree", t["alias_choice_agrees_between_flights"])
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
