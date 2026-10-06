"""If the inter-drone relative position were known to ~sigma, would the approach registrations succeed?

Run inside the ROS container:
  PYTHONPATH=/workspace/AV_Drone/src/drone_cslam python3 known_relative_position.py
For every approach-zone candidate of the pilot flights: (a) the flight's own result (B0 prior, +-6 m),
(b)/(c) initial guess = truth + Gaussian error of RMS sigma (0.5 m, 1.0 m; yaw N(0, 2 deg)), search +-3 sigma.
GT is used to emulate the external relative-position measurement and to score.
"""
import csv, glob, json, math, os, zlib
from multiprocessing import Pool
import numpy as np, yaml
from drone_cslam.registration import register_submaps
from drone_cslam.se2 import between, compose

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "../.."))
CONFIG = yaml.safe_load(open(os.path.join(PROJECT, "src/drone_cslam/config/lidar_no_gt_two_regions_converged.yaml")))
CASES = {"known_0.5m": 0.5, "known_1.0m": 1.0}
REPEATS = 5


def run(job):
    item, case, rep = job
    base = os.path.join(PROJECT, "artifacts", item["run"], "lidar_registration")
    d = np.load(glob.glob(os.path.join(base, "submaps", f"pair_{item['pair']:03d}_*.npz"))[0])
    truth = np.asarray(d["ground_truth_pose"], float)
    sigma = CASES[case]
    rng = np.random.default_rng(zlib.crc32(f"{item['run']}:{item['pair']}:{case}:{rep}".encode()))
    dx, dy = rng.normal(0, sigma / math.sqrt(2), 2)
    initial = compose(truth, np.array([dx, dy, math.radians(rng.normal(0, 2.0))]))
    window = max(3.0 * sigma, 1.5)
    config = {**CONFIG, "registration": {**CONFIG["registration"], "search_translation_m": window}}
    res = register_submaps(d["source_points"], d["target_points"], initial, config)
    err = between(truth, res.pose)
    et, ey = float(np.hypot(err[0], err[1])), abs(math.degrees(err[2]))
    return {**item, "case": case, "rep": rep, "window_m": window, "err_m": round(et, 3),
            "correct": et <= 1.0 and ey <= 5.0, "gate": bool(res.accepted), "overlap": round(res.overlap_ratio, 3),
            "margin": round(res.score_margin_ratio, 4), "reasons": ";".join(res.rejection_reasons)}


def main():
    items = json.load(open(os.path.join(HERE, "near_pairs.json")))
    jobs = [(it, c, r) for it in items for c in CASES for r in range(REPEATS)]
    with Pool(24) as pool:
        rows = pool.map(run, jobs, chunksize=2)
    # the flights' own results
    for it in items:
        regs = {}
        for r in csv.DictReader(open(os.path.join(PROJECT, "artifacts", it["run"], "lidar_registration/registrations.csv"))):
            regs.setdefault(int(r["pair"]), r)
        r = regs[it["pair"]]
        rows.append({**it, "case": "flight_b0_6m", "rep": 0, "window_m": 6.0, "err_m": round(float(r["estimated_translation_error_m"]), 3),
                     "correct": r["evaluation_pose_match_correct"] == "True", "gate": r["geometry_pass"] == "True",
                     "overlap": round(float(r["overlap_ratio"]), 3), "margin": round(float(r["score_margin_ratio"]), 4),
                     "reasons": r["rejection_reasons"]})
    json.dump(rows, open(os.path.join(HERE, "known_relative_position.json"), "w"), indent=1)
    print("written", len(rows))


if __name__ == "__main__":
    main()
