"""Zone-wise outcome of inter-drone registration on the 4-zone trailing flights.

Inputs (read-only): runs.json (this folder), sim_assets/worlds/zoned_corridor_w*_layout.json,
artifacts/<run>/lidar_registration/{registrations.csv, submaps/*.npz, trajectories/B0.csv}.
Ground truth is used only for zone labels, the same-place test and correctness labels.
Scores reuse the definitions of experiments/2026-09-27_alias_predictor_offline/analyze.py.

  python3 analyze_zones.py        # needs numpy
"""
import collections
import csv
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "../.."))
BOUNDARY_M = 7.0          # submaps (5 m half length) near a zone edge mix two zone types
SAME_PLACE_OVERLAP = 0.2  # GT-aligned overlap at or above this = the two submaps see the same place


def T(p, pts):
    c, s = np.cos(p[2]), np.sin(p[2])
    return pts @ np.array([[c, -s], [s, c]]).T + p[:2]


def nn_overlap(a, b, r=0.3):
    if len(a) == 0 or len(b) == 0:
        return 0.0
    d = np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)).min(1)
    return float((d < r).mean())


def normals(pts, rad=0.6):
    d2 = ((pts[:, None, :] - pts[None, :, :]) ** 2).sum(-1)
    n, ok = np.zeros_like(pts), np.zeros(len(pts), bool)
    for i in range(len(pts)):
        nb = pts[d2[i] < rad * rad]
        if len(nb) < 3:
            continue
        w, v = np.linalg.eigh(np.cov((nb - nb.mean(0)).T))
        if w[1] > 0:
            n[i], ok[i] = v[:, 0], True
    return n, ok


def alpha(pts):
    """Translation degeneracy score (Nobili-style alignability); low = degenerate."""
    n, ok = normals(pts)
    if ok.sum() < 10:
        return 0.0
    w = np.linalg.eigvalsh(n[ok].T @ n[ok])
    return float(w[0] / w[1])


class Grid:
    def __init__(self, pts, res=0.1, rad=0.3):
        self.res, self.lo = res, pts.min(0) - 40
        shape = np.ceil((pts.max(0) + 40 - self.lo) / res).astype(int) + 1
        occ = np.zeros(shape, bool)
        ij = np.floor((pts - self.lo) / res).astype(int)
        occ[ij[:, 0], ij[:, 1]] = True
        k = int(round(rad / res))
        self.hit = np.zeros_like(occ)
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                if a * a + b * b <= k * k:
                    self.hit[max(a, 0):shape[0] + min(a, 0), max(b, 0):shape[1] + min(b, 0)] |= \
                        occ[max(-a, 0):shape[0] + min(-a, 0), max(-b, 0):shape[1] + min(-b, 0)]
        self.shape = shape

    def score(self, batch):
        ij = np.floor((batch - self.lo) / self.res).astype(int)
        inb = (ij[..., 0] >= 0) & (ij[..., 1] >= 0) & (ij[..., 0] < self.shape[0]) & (ij[..., 1] < self.shape[1])
        ij = np.where(inb[..., None], ij, 0)
        return (self.hit[ij[..., 0], ij[..., 1]] & inb).mean(-1)


def alias_score(grid, S, P0, window_x, window_y, exclude=2.5, exclude_yaw=10):
    """Best match of submap S elsewhere in drone1's own map within the window / match in place."""
    own = float(grid.score(T(P0, S)[None])[0])
    dxs = np.arange(-window_x, window_x + 0.01, 0.5)
    dys = np.arange(-window_y, window_y + 0.01, 0.5)
    gx, gy = np.meshgrid(dxs, dys, indexing="ij")
    off_all = np.column_stack([gx.ravel(), gy.ravel()])
    best = 0.0
    for th in np.deg2rad(np.arange(-30, 31, 5)):
        c, s = np.cos(P0[2] + th), np.sin(P0[2] + th)
        base = S @ np.array([[c, -s], [s, c]]).T + P0[:2]
        far = (np.abs(off_all[:, 0]) > exclude) | (np.abs(off_all[:, 1]) > exclude) | (abs(np.rad2deg(th)) >= exclude_yaw)
        off = off_all[far]
        for k in range(0, len(off), 800):
            best = max(best, float(grid.score(base[None] + off[k:k + 800][:, None, :]).max()))
    return best / own if own > 0 else 0.0


def auroc(score, wrong):
    score, wrong = np.asarray(score, float), np.asarray(wrong, bool)
    pos, neg = score[wrong], score[~wrong]
    if not len(pos) or not len(neg):
        return float("nan")
    gt = (pos[:, None] > neg[None, :]).sum()
    eq = (pos[:, None] == neg[None, :]).sum()
    return float((gt + 0.5 * eq) / (len(pos) * len(neg)))


def zone_of(x, zones):
    for z in zones:
        if z["x_min"] + BOUNDARY_M <= x <= z["x_max"] - BOUNDARY_M:
            return z["type"]
    return "boundary" if 15.0 <= x <= 135.0 else "outside"


def main():
    runs = json.load(open(os.path.join(HERE, "runs.json")))["valid"]
    rows = []
    for item in runs:
        run, world = item["run"], item["world"]
        layout = json.load(open(os.path.join(PROJECT, f"sim_assets/worlds/{world}_layout.json")))
        base = os.path.join(PROJECT, "artifacts", run, "lidar_registration")
        regs = {}
        for r in csv.DictReader(open(os.path.join(base, "registrations.csv"))):
            regs.setdefault(r["pair"], r)
        kf = {}
        for r in csv.DictReader(open(os.path.join(base, "trajectories/B0.csv"))):
            if r["vehicle"] == "drone1":
                kf[int(r["keyframe"])] = (np.array([float(r["x"]), float(r["y"]), float(r["yaw"])]), float(r["gt_x"]))
        subs = {str(int(os.path.basename(f)[5:8])): np.load(f) for f in glob.glob(os.path.join(base, "submaps/*.npz"))}
        placed = {}
        for pid, d in subs.items():
            k = int(regs[pid]["source_keyframe"])
            placed[k] = T(kf[k][0], d["source_points"])
        grid = Grid(np.vstack(list(placed.values())))
        for pid, d in sorted(subs.items(), key=lambda x: int(x[0])):
            r = regs[pid]
            k = int(r["source_keyframe"])
            S = d["source_points"]
            gt_overlap = nn_overlap(T(d["ground_truth_pose"], d["target_points"]), S)
            rows.append({
                "run": run, "world": world, "pair": int(pid), "source_kf": k,
                "gt_x": kf[k][1], "zone": zone_of(kf[k][1], layout["zones"]),
                "n_pts": int(len(S)), "alpha": alpha(S),
                "alias_30m": alias_score(grid, S, kf[k][0], 30.0, 6.0),
                "alias_6m": alias_score(grid, S, kf[k][0], 6.0, 6.0),
                "gt_overlap": gt_overlap, "same_place": gt_overlap >= SAME_PLACE_OVERLAP,
                "correct": r["evaluation_pose_match_correct"] == "True",
                "geometry_pass": r["geometry_pass"] == "True",
                "consistent": r["geometry_pass"] == "True" and r["support_pairs"] not in ("", "[]"),
                "rejection": r["rejection_reasons"],
                "score_margin": float(r["score_margin_ratio"] or 0),
                "est_err_m": float(r["estimated_translation_error_m"] or "nan"),
            })
        print(run, world, len(subs), "pairs", file=sys.stderr)
    json.dump(rows, open(os.path.join(HERE, "pairs.json"), "w"), indent=1)

    table = {}
    for zone in ("RU", "RR", "SU", "SR", "boundary", "outside"):
        sel = [x for x in rows if x["zone"] == zone]
        same = [x for x in sel if x["same_place"]]
        table[zone] = {
            "pairs": len(sel), "same_place": len(same),
            "same_place_wrong": sum(not x["correct"] for x in same),
            "gate_pass": sum(x["geometry_pass"] for x in sel),
            "gate_pass_wrong": sum(x["geometry_pass"] and not x["correct"] for x in sel),
            "gate_pass_consistent": sum(x["consistent"] for x in sel),
            "gate_pass_consistent_wrong": sum(x["consistent"] and not x["correct"] for x in sel),
            "same_place_correct_rejected": sum(x["correct"] and not x["geometry_pass"] for x in same),
            "alpha_median": float(np.median([x["alpha"] for x in sel])) if sel else float("nan"),
            "alias_6m_median": float(np.median([x["alias_6m"] for x in sel])) if sel else float("nan"),
        }
    same = [x for x in rows if x["same_place"] and x["zone"] in ("RU", "RR", "SU", "SR")]
    wrong = [not x["correct"] for x in same]
    scores = {"alpha (low = risky)": [-x["alpha"] for x in same],
              "alias +-30 m": [x["alias_30m"] for x in same],
              "alias +-6 m": [x["alias_6m"] for x in same],
              "points (few = risky)": [-x["n_pts"] for x in same]}
    summary = {"pairs": len(rows), "same_place_in_zones": len(same), "same_place_wrong": int(sum(wrong)),
               "by_zone": table,
               "auroc_same_place_in_zones": {k: auroc(v, wrong) for k, v in scores.items()},
               "by_run": dict(collections.Counter(x["run"] for x in rows))}
    json.dump(summary, open(os.path.join(HERE, "summary.json"), "w"), indent=1, ensure_ascii=False)
    print(json.dumps(summary, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
