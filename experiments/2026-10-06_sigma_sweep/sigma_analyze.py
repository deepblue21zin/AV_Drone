"""Summarise the sigma sweep and compare fixed-window and sigma-window alias scores.

  python3 sigma_analyze.py          # numpy only; reads registrations_sigma.csv

The alias score of a pair is computed on drone1's own map (no GT) for several search
windows; the sigma-window score uses the window max(k*sigma, 2 m). k is chosen on the
design worlds (w1, w2) and reported on the evaluation worlds (w3, w4).
"""
import collections
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FLIGHTS = os.path.join(HERE, "../2026-10-06_zoned_trailing_flights")
sys.path.insert(0, FLIGHTS)
import analyze_zones as az  # noqa: E402

WINDOWS = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 15.0, 30.0]
K_CHOICES = [1.0, 1.5, 2.0, 3.0]
DESIGN = ("zoned_corridor_w1", "zoned_corridor_w2")
ZONES = ("RU", "RR", "SU", "SR")


def window_scores():
    """alias score per (run, pair, window), cached in alias_windows.json."""
    cache = os.path.join(HERE, "alias_windows.json")
    if os.path.exists(cache):
        return {tuple(k.split("|")[:2]): v for k, v in json.load(open(cache)).items()}
    import csv as _csv
    import glob
    pairs = [p for p in json.load(open(os.path.join(FLIGHTS, "pairs.json")))
             if p["same_place"] and p["zone"] in ZONES]
    out = {}
    for run in sorted({p["run"] for p in pairs}):
        base = os.path.join(az.PROJECT, "artifacts", run, "lidar_registration")
        regs = {}
        for r in _csv.DictReader(open(os.path.join(base, "registrations.csv"))):
            regs.setdefault(r["pair"], r)
        kf = {}
        for r in _csv.DictReader(open(os.path.join(base, "trajectories/B0.csv"))):
            if r["vehicle"] == "drone1":
                kf[int(r["keyframe"])] = np.array([float(r["x"]), float(r["y"]), float(r["yaw"])])
        subs = {str(int(os.path.basename(f)[5:8])): np.load(f) for f in glob.glob(os.path.join(base, "submaps/*.npz"))}
        placed = {int(regs[pid]["source_keyframe"]): az.T(kf[int(regs[pid]["source_keyframe"])], d["source_points"])
                  for pid, d in subs.items()}
        grid = az.Grid(np.vstack(list(placed.values())))
        for p in [p for p in pairs if p["run"] == run]:
            d = subs[str(p["pair"])]
            pose = kf[int(regs[str(p["pair"])]["source_keyframe"])]
            out[(run, str(p["pair"]))] = {str(w): az.alias_score(grid, d["source_points"], pose, w, min(w, 6.0), exclude=1.5)
                                          for w in WINDOWS}
        print(run, "alias windows done", file=sys.stderr)
    json.dump({"|".join(k): v for k, v in out.items()}, open(cache, "w"), indent=1)
    return out


def nearest_window(w):
    return str(min(WINDOWS, key=lambda x: abs(x - w)))


def main():
    rows = list(csv.DictReader(open(os.path.join(HERE, "registrations_sigma.csv"))))
    pairs = {(p["run"], str(p["pair"])): p for p in json.load(open(os.path.join(FLIGHTS, "pairs.json")))}
    alias = window_scores()
    for r in rows:
        r["sigma"] = float(r["sigma"])
        r["wrong"] = r["correct"] != "True"
        r["gate"] = r["gate"] == "True"
        r["accepted_wrong"] = r["wrong"] and r["gate"]
    sigmas = sorted({r["sigma"] for r in rows})

    # 1. outcome by zone and sigma
    rates = {}
    for z in ZONES:
        rates[z] = {}
        for s in sigmas:
            sel = [r for r in rows if r["zone"] == z and r["sigma"] == s]
            rates[z][str(s)] = {"n": len(sel), "wrong": float(np.mean([r["wrong"] for r in sel])),
                                "accepted_wrong": float(np.mean([r["accepted_wrong"] for r in sel])),
                                "rejected": float(np.mean([not r["gate"] for r in sel]))}

    # 2. predicting accepted-wrong at each sigma, per registration attempt
    def score(r, kind, k=None):
        key = (r["run"], r["pair"])
        if kind == "alpha":
            return -pairs[key]["alpha"]
        if kind == "fixed30":
            return alias[key]["30.0"]
        if kind == "fixed6":
            return alias[key]["6.0"]
        return alias[key][nearest_window(max(k * r["sigma"], 2.0))]

    def au(sel, kind, k=None):
        return az.auroc([score(r, kind, k) for r in sel], [r["accepted_wrong"] for r in sel])

    design = [r for r in rows if r["world"] in DESIGN and r["sigma"] > 0]
    k_fit = {str(k): au(design, "sigma", k) for k in K_CHOICES}
    k_best = max(K_CHOICES, key=lambda k: k_fit[str(k)])
    table = {}
    for name, worlds in (("design", DESIGN), ("evaluation", ("zoned_corridor_w3", "zoned_corridor_w4"))):
        table[name] = {}
        for s in sigmas[1:] + ["all"]:
            sel = [r for r in rows if r["world"] in worlds and r["sigma"] > 0 and (s == "all" or r["sigma"] == s)]
            table[name][str(s)] = {"n": len(sel), "accepted_wrong": int(sum(r["accepted_wrong"] for r in sel)),
                                   "alpha": au(sel, "alpha"), "alias_fixed_6m": au(sel, "fixed6"),
                                   "alias_fixed_30m": au(sel, "fixed30"), "alias_k_sigma": au(sel, "sigma", k_best)}
    summary = {"registrations": len(rows), "outcome_by_zone_sigma": rates,
               "k_fit_on_design": k_fit, "k_chosen": k_best, "auroc_accepted_wrong": table,
               "note": "Gate = overlap/RMSE/score-margin/correction checks; neighbour consistency is not applied here."}
    json.dump(summary, open(os.path.join(HERE, "summary.json"), "w"), indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
