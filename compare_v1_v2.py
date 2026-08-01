#!/usr/bin/env python3
# THE BIG COMPARISON: v1 vs v2.9.1, paired per seed, 25 worlds
# Uniform offline deadlock rule: net displacement < 0.5 m over a 120 sim-s window,
# "terminal" if it lasts until the end of the run's data.
import glob
import re
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
V1_CUTOFF = "20260723_19"   # files with timestamp >= 23/7 19:xx = v1

def door_geometry(seed):
    doors = []
    for line in open(f"{RUNS}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "door":
            doors.append(dict(x=float(p[2]), y=float(p[3]), g=float(p[4]), axis=p[5]))
    pxw = next(d["x"] for d in doors if d["axis"] == "x")
    pyw = next(d["y"] for d in doors if d["axis"] == "y")
    return doors, pxw, pyw

def analyze(csv, pxw, pyw):
    d = np.genfromtxt(csv, delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    seg = np.hypot(np.diff(x), np.diff(y))
    dur, dist = float(t[-1]), float(seg.sum())
    # crossings (on both walls)
    cross = 0
    for coord, pos in ((x, pxw), (y, pyw)):
        s = coord > pos
        cross += int((s[:-1] != s[1:]).sum())
    # deadlock: first t where net<0.5m for 120 consecutive sim-s
    W = 1200  # 120 s @ 10 Hz
    dl_t = None
    i = 0
    while i + W < len(t):
        if np.hypot(x[i + W] - x[i], y[i + W] - y[i]) < 0.5:
            dl_t = float(t[i])
            break
        i += 50
    terminal = False
    if dl_t is not None:
        # does it stay still until the end? (net from episode start to the end)
        j = np.searchsorted(t, dl_t)
        terminal = bool(np.hypot(x[-1] - x[j], y[-1] - y[j]) < 0.8)
    return dur, dist, cross, dl_t, terminal

def newest_per_seed(files):
    best = {}
    for f in files:
        seed = re.search(r"run_(\d+)_", f).group(1)
        stamp = re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)
        if seed not in best or stamp > best[seed][1]:
            best[seed] = (f, stamp)
    return {s: f for s, (f, _) in best.items()}

all_files = glob.glob(f"{RUNS}/run_202607230*.csv")
v1_files = newest_per_seed([f for f in all_files if re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) >= V1_CUTOFF])
v2_files = newest_per_seed([f for f in all_files if re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) < V1_CUTOFF])

print(f"{'seed':>12} | {'v2 dist':>8} {'v2 cr':>5} {'v2 dl':>6} | {'v1 dist':>8} {'v1 cr':>5} {'v1 dl@':>7} {'term':>4}")
rows = []
for seed in sorted(v2_files):
    doors, pxw, pyw = door_geometry(seed)
    d2, dist2, c2, dl2, term2 = analyze(v2_files[seed], pxw, pyw)
    d1, dist1, c1, dl1, term1 = analyze(v1_files[seed], pxw, pyw)
    rows.append(dict(seed=seed, d2=d2, dist2=dist2, c2=c2, dl2=dl2, term2=term2,
                     d1=d1, dist1=dist1, c1=c1, dl1=dl1, term1=term1))
    print(f"{seed:>12} | {dist2:8.1f} {c2:5d} {('%.0f' % dl2) if dl2 else '—':>6} | "
          f"{dist1:8.1f} {c1:5d} {('%.0f' % dl1) if dl1 else '—':>7} {'YES' if term1 else '—':>4}")

v1_dl = [r for r in rows if r["dl1"] is not None]
v1_term = [r for r in rows if r["term1"]]
v2_dl = [r for r in rows if r["dl2"] is not None]
v2_term = [r for r in rows if r["term2"]]
print(f"\n=== AGGREGATES ({len(rows)} world pairs) ===")
print(f"v1:     deadlock events {len(v1_dl)}/25, TERMINAL {len(v1_term)}/25, "
      f"median time to deadlock {np.median([r['dl1'] for r in v1_dl]):.0f}s" if v1_dl else "v1: none")
print(f"v2.9.1: deadlock events {len(v2_dl)}/25, TERMINAL {len(v2_term)}/25")
print(f"distance: v1 total {sum(r['dist1'] for r in rows):.0f} m, v2 total {sum(r['dist2'] for r in rows):.0f} m")
print(f"crossings: v1 {sum(r['c1'] for r in rows)}, v2 {sum(r['c2'] for r in rows)}")
print(f"m/active minute: v1 {sum(r['dist1'] for r in rows)/sum(r['d1'] for r in rows)*60:.1f}, "
      f"v2 {sum(r['dist2'] for r in rows)/sum(r['d2'] for r in rows)*60:.1f}")
import json
json.dump(rows, open("/home/user/isaac/compare_rows.json", "w"))
print("saved compare_rows.json")
