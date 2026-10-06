"""A1b: same pairs, Gaussian initial error of RMS sigma, three search-window policies.

Run inside the ROS container (needs scipy and drone_cslam):
  PYTHONPATH=/workspace/AV_Drone/src/drone_cslam python3 sigma_window_register.py

Inputs (read-only): ../2026-10-06_zoned_trailing_flights/pairs.json and
artifacts/<run>/lidar_registration/submaps/*.npz. Registration and gate use the same
config as the flights; only the initial guess changes. GT is used to build the perturbed
initial guess and to score, as in a controlled sensitivity study.
"""
import csv
import glob
import json
import math
import os
import zlib
from multiprocessing import Pool

import numpy as np
import yaml

from drone_cslam.registration import register_submaps
from drone_cslam.se2 import between, compose

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "../.."))
CONFIG = yaml.safe_load(open(os.path.join(PROJECT, "src/drone_cslam/config/lidar_no_gt_two_regions_converged.yaml")))
SIGMAS = [0.5, 1.0, 2.0, 4.0]
# search half-width in metres as a function of sigma; the flights used the fixed 6 m window
POLICIES = {"fixed6": lambda s: 6.0, "k3": lambda s: min(6.0, max(3.0 * s, 1.5)), "k2": lambda s: min(6.0, max(2.0 * s, 1.5))}
REPEATS = 5
YAW_NOISE_DEG = 2.0       # typical B0 relative yaw error; held fixed so only translation varies
CORRECT_M, CORRECT_DEG = 1.0, 5.0


def jobs():
    pairs = json.load(open(os.path.join(HERE, "../2026-10-06_zoned_trailing_flights/pairs.json")))
    out = []
    for p in pairs:
        if not p["same_place"] or p["zone"] not in ("RU", "RR", "SU", "SR"):
            continue
        for sigma in SIGMAS:
            for rep in range(REPEATS):
                for policy in POLICIES:
                    out.append((p["run"], p["world"], p["zone"], p["pair"], sigma, rep, policy))
    return out


def run(job):
    run_id, world, zone, pair, sigma, rep, policy = job
    path = glob.glob(os.path.join(PROJECT, "artifacts", run_id, "lidar_registration/submaps",
                                  f"pair_{pair:03d}_*.npz"))[0]
    d = np.load(path)
    truth = np.asarray(d["ground_truth_pose"], float)
    rng = np.random.default_rng(zlib.crc32(f"{run_id}:{pair}:{sigma}:{rep}".encode()))
    # same seed for every policy: the three policies see the identical initial guess
    dx, dy = rng.normal(0, sigma / math.sqrt(2), 2)   # RMS distance = sigma
    offset = np.array([dx, dy, math.radians(rng.normal(0, YAW_NOISE_DEG))])
    initial = compose(truth, offset)
    window = POLICIES[policy](sigma)
    config = {**CONFIG, "registration": {**CONFIG["registration"], "search_translation_m": window}}
    result = register_submaps(d["source_points"], d["target_points"], initial, config)
    err = between(truth, result.pose)
    et, ey = float(np.hypot(err[0], err[1])), abs(math.degrees(err[2]))
    return {"run": run_id, "world": world, "zone": zone, "pair": pair, "sigma": sigma, "rep": rep,
            "policy": policy, "window_m": window, "initial_err_m": round(float(math.hypot(dx, dy)), 3), "err_m": round(et, 3), "err_deg": round(ey, 2),
            "correct": et <= CORRECT_M and ey <= CORRECT_DEG, "gate": bool(result.accepted),
            "overlap": round(result.overlap_ratio, 3), "rmse": round(result.inlier_rmse_m, 3),
            "margin": round(result.score_margin_ratio, 4), "reasons": ";".join(result.rejection_reasons),
            # target-in-source poses, so each case can be drawn on the map
            "initial_pose": " ".join(f"{v:.4f}" for v in initial), "estimated_pose": " ".join(f"{v:.4f}" for v in result.pose)}


def main():
    work = jobs()
    print(len(work), "registrations", flush=True)
    rows = []
    with Pool(int(os.environ.get("WORKERS", "16"))) as pool:
        for i, row in enumerate(pool.imap_unordered(run, work, chunksize=4)):
            rows.append(row)
            if (i + 1) % 200 == 0:
                print(f"{i + 1}/{len(work)}", flush=True)
    rows.sort(key=lambda r: (r["run"], r["pair"], r["sigma"], r["rep"], r["policy"]))
    with open(os.path.join(HERE, "registrations_window.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("written", len(rows), flush=True)


if __name__ == "__main__":
    main()
