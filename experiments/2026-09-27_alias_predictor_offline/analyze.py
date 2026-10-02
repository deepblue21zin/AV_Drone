"""Offline check: can drone1's submap alone predict wrong inter-drone matches?

Inputs (read-only): artifacts/<run>/lidar_registration/{registrations.csv, submaps/*.npz,
trajectories/B0.csv}. Ground truth is used ONLY for labels and the same-place filter.
"""
import csv, glob, json, os, sys
import numpy as np

ART = '/home3/deepblue/work/AV_Drone/artifacts/'
RUNS = ['20260911T091146Z_bed606a28e99', '20260911T122110Z_21f6a1d0f520', '20260911T161139Z_9abd773a46f1']
OUT = os.path.dirname(os.path.abspath(__file__))

def T(p, pts):
    c, s = np.cos(p[2]), np.sin(p[2]); R = np.array([[c, -s], [s, c]]); return pts @ R.T + p[:2]

def nn_overlap(a, b, r=0.3):
    if len(a) == 0 or len(b) == 0: return 0.0
    d = np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)).min(1); return float((d < r).mean())

# ---------- features from ONE submap (drone1 side) ----------
def normals(pts, rad=0.6):
    d2 = ((pts[:, None, :] - pts[None, :, :]) ** 2).sum(-1)
    N = np.zeros_like(pts); ok = np.zeros(len(pts), bool)
    for i in range(len(pts)):
        nb = pts[d2[i] < rad * rad]
        if len(nb) < 3: continue
        C = np.cov((nb - nb.mean(0)).T); w, v = np.linalg.eigh(C)
        if w[1] <= 0: continue
        N[i] = v[:, 0]; ok[i] = True
    return N, ok

def degeneracy(pts):
    """Point-to-line information matrix at identity (Denniston-style single-cloud)."""
    N, ok = normals(pts)
    p, n = pts[ok], N[ok]
    if len(p) < 10: return 0.0, 0.0
    r = np.median(np.linalg.norm(p - p.mean(0), axis=1)) + 1e-6
    J = np.column_stack([n[:, 0], n[:, 1], (p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]) / r])
    H = J.T @ J
    wt = np.linalg.eigvalsh(H[:2, :2]); wf = np.linalg.eigvalsh(H)
    return float(wt[0] / wt[1]), float(wf[0] / wf[-1])   # translation alpha, full alpha

def clusters(pts, link=0.5):
    """Connected components -> count cylinder-like blobs (extent < 1.2 m) and long segments."""
    n = len(pts); parent = list(range(n))
    def f(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    d2 = ((pts[:, None, :] - pts[None, :, :]) ** 2).sum(-1)
    I, J = np.nonzero(np.triu(d2 < link * link, 1))
    for i, j in zip(I, J):
        a, b = f(i), f(j)
        if a != b: parent[a] = b
    roots = {}
    for i in range(n): roots.setdefault(f(i), []).append(i)
    cyl = seg = 0
    for idx in roots.values():
        if len(idx) < 3: continue
        q = pts[idx]; ext = np.linalg.norm(q.max(0) - q.min(0))
        if ext < 1.2: cyl += 1
        elif ext > 3.0: seg += 1
    return cyl, seg

# ---------- alias score against drone1's own map ----------
class Grid:
    def __init__(self, pts, res=0.1, rad=0.3):
        self.res = res; self.lo = pts.min(0) - 40; hi = pts.max(0) + 40
        shape = np.ceil((hi - self.lo) / res).astype(int) + 1
        occ = np.zeros(shape, bool)
        ij = np.floor((pts - self.lo) / res).astype(int); occ[ij[:, 0], ij[:, 1]] = True
        k = int(round(rad / res)); dil = np.zeros_like(occ)
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                if dx * dx + dy * dy > k * k: continue
                dil[max(dx, 0):shape[0] + min(dx, 0), max(dy, 0):shape[1] + min(dy, 0)] |= occ[max(-dx, 0):shape[0] + min(-dx, 0), max(-dy, 0):shape[1] + min(-dy, 0)]
        self.dil = dil; self.shape = shape
    def score(self, pts_batch):  # (P, N, 2) -> fraction of points hitting map
        ij = np.floor((pts_batch - self.lo) / self.res).astype(int)
        inb = (ij[..., 0] >= 0) & (ij[..., 1] >= 0) & (ij[..., 0] < self.shape[0]) & (ij[..., 1] < self.shape[1])
        ij = np.where(inb[..., None], ij, 0)
        hit = self.dil[ij[..., 0], ij[..., 1]] & inb
        return hit.mean(-1)

def alias_score(grid, S, P0, excl_t=2.5, excl_y=10):
    """Best match of submap S elsewhere in drone1 map / match at its own place."""
    own = float(grid.score(T(P0, S)[None])[0])
    dxs = np.arange(-30, 30.01, 0.5); dys = np.arange(-6, 6.01, 0.5); ths = np.deg2rad(np.arange(-30, 31, 5))
    best = 0.0; best_off = None
    for th in ths:
        c, s = np.cos(P0[2] + th), np.sin(P0[2] + th); R = np.array([[c, -s], [s, c]])
        base = S @ R.T + P0[:2]
        DX, DY = np.meshgrid(dxs, dys, indexing='ij'); off = np.column_stack([DX.ravel(), DY.ravel()])
        far = (np.abs(off[:, 0]) > excl_t) | (np.abs(off[:, 1]) > excl_t) | (abs(np.rad2deg(th)) >= excl_y)
        off = off[far]
        if not len(off): continue
        for k in range(0, len(off), 800):
            o = off[k:k + 800]
            sc = grid.score(base[None, :, :] + o[:, None, :])
            m = int(sc.argmax())
            if sc[m] > best: best = float(sc[m]); best_off = (float(o[m, 0]), float(o[m, 1]), float(np.rad2deg(th)))
    return own, best, (best / own if own > 0 else 0.0), best_off

def auroc(score, wrong):
    score = np.asarray(score, float); wrong = np.asarray(wrong, bool)
    pos, neg = score[wrong], score[~wrong]
    if not len(pos) or not len(neg): return float('nan')
    gt = (pos[:, None] > neg[None, :]).sum(); eq = (pos[:, None] == neg[None, :]).sum()
    return float((gt + 0.5 * eq) / (len(pos) * len(neg)))

def spearman(a, b):
    ra = np.argsort(np.argsort(a)); rb = np.argsort(np.argsort(b)); return float(np.corrcoef(ra, rb)[0, 1])

rows_out = []
for run in RUNS:
    base = ART + run + '/lidar_registration/'
    regs = {}
    for r in csv.DictReader(open(base + 'registrations.csv')): regs.setdefault(r['pair'], r)
    kf = {}
    for r in csv.DictReader(open(base + 'trajectories/B0.csv')):
        if r['vehicle'] == 'drone1': kf[int(r['keyframe'])] = (np.array([float(r['x']), float(r['y']), float(r['yaw'])]), float(r['progress_m']))
    subs = {}
    for f in glob.glob(base + 'submaps/*.npz'):
        pid = str(int(os.path.basename(f)[5:8])); subs[pid] = np.load(f)
    # drone1 map in its raw (believed) frame from all distinct source keyframes of this run
    placed = {}
    for pid, d in subs.items():
        k = int(regs[pid]['source_keyframe']); placed[k] = T(kf[k][0], d['source_points'])
    M = np.vstack(list(placed.values()))
    grid = Grid(M)
    for pid, d in sorted(subs.items(), key=lambda x: int(x[0])):
        r = regs[pid]; k = int(r['source_keyframe']); S = d['source_points']
        at, af = degeneracy(S); cyl, seg = clusters(S)
        own, best, ratio, off = alias_score(grid, S, kf[k][0])
        gt_ov = nn_overlap(T(d['ground_truth_pose'], d['target_points']), S)
        err = float(r['estimated_translation_error_m']) if r['estimated_translation_error_m'] else float('nan')
        rows_out.append(dict(run=run[:16], pair=int(pid), source_kf=k, progress_m=kf[k][1], n_pts=len(S),
            alpha_t=at, alpha_full=af, cyl=cyl, seg=seg, alias_own=own, alias_best=best, alias_ratio=ratio,
            alias_off=off, gt_overlap=gt_ov, correct=r['evaluation_pose_match_correct'] == 'True',
            accepted=r['geometry_pass'] == 'True', est_err_m=err, reg_overlap=float(r['overlap_ratio'] or 0),
            reg_rmse=float(r['inlier_rmse_m'] or 0)))
    print(run, len(subs), 'pairs, map pts', len(M), file=sys.stderr)

json.dump(rows_out, open(os.path.join(OUT, 'features.json'), 'w'), indent=1)

R = rows_out; wrong = np.array([not x['correct'] for x in R])
same = np.array([x['gt_overlap'] >= 0.2 for x in R])
feats = {
    '퇴화 점수 α (병진, 낮을수록 위험)': [-x['alpha_t'] for x in R],
    '퇴화 점수 α (전체, 낮을수록 위험)': [-x['alpha_full'] for x in R],
    '원통 개수 (적을수록 위험)': [-x['cyl'] for x in R],
    '점 개수 (적을수록 위험)': [-x['n_pts'] for x in R],
    '착각 위험 점수 (다른 곳 최고 일치 / 제자리 일치)': [x['alias_ratio'] for x in R],
}
summary = {'n': len(R), 'wrong': int(wrong.sum()), 'correct': int((~wrong).sum()), 'same_place': int(same.sum()),
           'same_place_wrong': int((wrong & same).sum()), 'diff_place_correct': int((~wrong & ~same).sum()), 'auroc': {}}
for name, sc in feats.items():
    sc = np.array(sc)
    summary['auroc'][name] = {'all': auroc(sc, wrong), 'per_run': [auroc(sc[[x['run'] == r[:16] for x in R]], wrong[[x['run'] == r[:16] for x in R]]) for r in RUNS]}
c = ~wrong
errs = np.array([x['est_err_m'] for x in R])
summary['spearman_correct_err'] = {
    'alpha_t': spearman(np.array([x['alpha_t'] for x in R])[c], errs[c]),
    'alias_ratio': spearman(np.array([x['alias_ratio'] for x in R])[c], errs[c]),
    'cyl': spearman(np.array([x['cyl'] for x in R])[c], errs[c]),
    'reg_overlap(post, 참고)': spearman(np.array([x['reg_overlap'] for x in R])[c], errs[c]),
}
json.dump(summary, open(os.path.join(OUT, 'summary.json'), 'w'), indent=1, ensure_ascii=False)
print(json.dumps(summary, indent=1, ensure_ascii=False))
