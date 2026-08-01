#!/usr/bin/env python3
# Step 1 - FINAL: 25 maps vs 25 manifests + drift from odom logs + encounter-vs-coverage
import glob
import re
import json
import numpy as np

D = "/home/user/isaac/slam_batch"
MANI = "/mnt/user-data/uploads/isaac_project/runs"   # the manifests (geometry unchanged)
import importlib.util
em = {}
exec(open("/home/user/isaac/evaluate_map.py").read().split("def main")[0], em)


def newest(files):
    best = {}
    for f in files:
        s = re.search(r"(?:odom|run)_(\d+)_", f).group(1)
        st = re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)
        if s not in best or st > best[s][1]:
            best[s] = (f, st)
    return {s: f for s, (f, _) in best.items()}


odom_f = newest(glob.glob(f"{D}/odom_202607230*.csv"))
run_f = newest(glob.glob(f"{D}/run_202607230*.csv"))

rows = []
print(f"{'seed':>5} {'prec':>6} {'surf':>6} {'fIoU':>6} {'unk':>5} | {'driftMed':>8} {'driftMax':>8} | "
      f"{'enc%':>5} {'dist':>6}")
for pgm in sorted(glob.glob(f"{D}/world_202607230*.pgm")):
    seed = re.search(r"world_(\d+)\.pgm", pgm).group(1)
    yaml_p = pgm.replace(".pgm", ".yaml")
    occ, free, res, origin = em["load_map"](yaml_p)
    gt, obstacles = em["render_truth"](f"{MANI}/world_{seed}.csv", res, occ.shape, origin)
    gtd1 = em["dilate"](gt, 1)
    best = (-1.0, 0, 0)
    k = int(0.4 / res)
    for dy in range(-k, k + 1, 2):
        for dx in range(-k, k + 1, 2):
            sc = float((np.roll(np.roll(occ, dy, 0), dx, 1) & gtd1).sum())
            if sc > best[0]:
                best = (sc, dy, dx)
    _, dy, dx = best
    occ_a = np.roll(np.roll(occ, dy, 0), dx, 1)
    free_a = np.roll(np.roll(free, dy, 0), dx, 1)
    gt_d = em["dilate"](gt, 2)
    occ_d = em["dilate"](occ_a, 2)
    gt_edge = gt & ~(np.roll(gt, 1, 0) & np.roll(gt, -1, 0) &
                     np.roll(gt, 1, 1) & np.roll(gt, -1, 1))
    prec = float((occ_a & gt_d).sum()) / max(occ_a.sum(), 1)
    surf = float((gt_edge & occ_d).sum()) / max(gt_edge.sum(), 1)
    free_gt = ~em["dilate"](gt, 3)
    fiou = float((free_a & free_gt).sum()) / max((free_a | free_gt).sum(), 1)
    unk = 1.0 - (occ.sum() + free.sum()) / occ.size
    shift = float(np.hypot(dx * res, dy * res))

    od = np.genfromtxt(odom_f[seed], delimiter=",", skip_header=2)
    derr = np.hypot(od[:, 0] - od[:, 3], od[:, 1] - od[:, 4])
    dmed, dmax = float(np.median(derr)), float(derr.max())

    m = np.genfromtxt(run_f[seed], delimiter=",", skip_header=2)
    t, x, y = m[:, 0], m[:, 1], m[:, 2]
    P = [(m[:, 4], m[:, 5]), (m[:, 6], m[:, 7]), (m[:, 8], m[:, 9])]
    mind = np.min([np.hypot(x - px, y - py) for px, py in P], axis=0)
    enc = float((mind < 1.2).mean())
    dist = float(np.hypot(np.diff(x), np.diff(y)).sum())

    rows.append(dict(seed=seed, prec=prec, surf=surf, fiou=fiou, unk=unk, shift=shift,
                     dmed=dmed, dmax=dmax, enc=enc, dist=dist,
                     obs_n=len(obstacles), pgm=pgm, yaml=yaml_p))
    print(f"{seed[-3:]:>5} {prec*100:5.1f}% {surf*100:5.1f}% {fiou*100:5.1f}% {unk*100:4.1f}% | "
          f"{dmed:7.2f}m {dmax:7.2f}m | {enc*100:4.1f}% {dist:6.0f}")

P_ = [r["prec"] for r in rows]
S_ = [r["surf"] for r in rows]
DX = [r["dmax"] for r in rows]
E_ = [r["enc"] for r in rows]
print(f"\n=== 25 MAPS (overnight, walking people, noise 3/5) ===")
print(f"precision: median {np.median(P_)*100:.1f}%  [{min(P_)*100:.1f} - {max(P_)*100:.1f}]")
print(f"surface coverage: median {np.median(S_)*100:.1f}%  [{min(S_)*100:.1f} - {max(S_)*100:.1f}]  "
      f"IQR [{np.percentile(S_,25)*100:.1f} - {np.percentile(S_,75)*100:.1f}]")
print(f"free-IoU: median {np.median([r['fiou'] for r in rows])*100:.1f}%")
print(f"alignment shift: max {max(r['shift'] for r in rows):.2f} m")
print(f"drift (measured): median-of-medians {np.median([r['dmed'] for r in rows]):.2f} m, "
      f"max {max(DX):.2f} m")
cc = np.corrcoef(E_, S_)[0, 1]
print(f"encounter-time <-> coverage correlation: r = {cc:+.2f} "
      f"({'humans slow down exploration' if cc < -0.3 else 'no strong relation' if abs(cc) < 0.3 else 'positive?'})")
cc2 = np.corrcoef(DX, P_)[0, 1]
print(f"max-drift <-> precision correlation: r = {cc2:+.2f} (ablation @ n=25)")
json.dump(rows, open("/home/user/isaac/slam_batch_rows.json", "w"))
print("saved slam_batch_rows.json")
