#!/usr/bin/env python3
# Experiment 3: the 25 maps against their manifests, drift from the odometry logs, and time
# near people vs coverage. Writes runs/slam_batch_rows.json.
import os
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import glob
import re
import json
import numpy as np

D = RUNS_DIR
MANI = RUNS_DIR   # the manifests (geometry unchanged)
import evaluate_map as em


def newest(files):
    best = {}
    for f in files:
        s = re.search(r"(?:odom|run)_(\d+)_", f).group(1)
        st = re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)
        if s not in best or st > best[s][1]:
            best[s] = (f, st)
    return {s: f for s, (f, _) in best.items()}


# mapping batch only (28/7 13:05-22:31). The same seeds were reused by Nav2 tests and missions from 29/7.
def _mapping(fs):
    return [f for f in fs if "20260728" <= os.path.basename(f)[-19:-4] < "20260729"]


odom_f = newest(_mapping(glob.glob(f"{D}/odom_202607230*.csv")))
run_f = newest(_mapping(glob.glob(f"{D}/run_202607230*.csv")))

rows = []
print(f"{'seed':>5} {'prec':>6} {'surf':>6} {'fIoU':>6} {'unk':>5} | {'driftMed':>8} {'driftMax':>8} | "
      f"{'enc%':>5} {'dist':>6}")
for pgm in sorted(glob.glob(f"{D}/world_202607230*.pgm")):
    seed = re.search(r"world_(\d+)\.pgm", pgm).group(1)
    yaml_p = pgm.replace(".pgm", ".yaml")
    r = em.evaluate(yaml_p, f"{MANI}/world_{seed}.csv")
    prec, surf, fiou, unk = r["prec"], r["surf_cov"], r["free_iou"], r["unknown"]
    obstacles = r["obstacles"]
    shift = float(np.hypot(r["dx"], r["dy"]))

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
                     obs_n=len(obstacles), pgm=os.path.basename(pgm), yaml=os.path.basename(yaml_p)))
    print(f"{seed[-3:]:>5} {prec*100:5.1f}% {surf*100:5.1f}% {fiou*100:5.1f}% {unk*100:4.1f}% | "
          f"{dmed:7.2f}m {dmax:7.2f}m | {enc*100:4.1f}% {dist:6.0f}")

P_ = [r["prec"] for r in rows]
S_ = [r["surf"] for r in rows]
DX = [r["dmax"] for r in rows]
E_ = [r["enc"] for r in rows]
print("\n=== 25 MAPS (one unattended session, walking people, noise 3/5) ===")
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
json.dump(rows, open(RUNS_DIR + "/slam_batch_rows.json", "w"))
print("saved slam_batch_rows.json")
