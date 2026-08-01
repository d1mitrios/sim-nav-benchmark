#!/usr/bin/env python3
# Batch analysis: crossings + attempts per door/width, dance episodes, aggregates
import glob
import re
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"

def load_world(seed):
    doors = []
    for line in open(f"{RUNS}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "door":
            # door,door_S_A,dx,dy,g,axis,0
            doors.append(dict(name=p[1], x=float(p[2]), y=float(p[3]), g=float(p[4]), axis=p[5]))
    pxw = next(d["x"] for d in doors if d["axis"] == "x")
    pyw = next(d["y"] for d in doors if d["axis"] == "y")
    return doors, pxw, pyw

results = []   # (seed, door_name, g, attempts, successes)
tot_time = tot_path = 0.0
dance_total = []

for rf in sorted(glob.glob(f"{RUNS}/run_202607230*.csv")):
    seed = re.search(r"run_(\d+)_", rf).group(1)
    doors, pxw, pyw = load_world(seed)
    d = np.genfromtxt(rf, delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    seg = np.hypot(np.diff(x), np.diff(y))
    tot_time += t[-1]
    tot_path += seg.sum()

    # dance episodes (60s net<0.35 while travelling >2.5m)
    W = 600
    i = 0
    while i + W < len(t):
        net = np.hypot(x[i + W] - x[i], y[i + W] - y[i])
        if net < 0.35 and seg[i:i + W].sum() > 2.5:
            dance_total.append((seed, t[i]))
            i += W
        else:
            i += 100

    # crossings: sign changes with respect to each wall, attributed to a door of the correct span
    for wall_axis, wall_pos in (("x", pxw), ("y", pyw)):
        coord = x if wall_axis == "x" else y
        other = y if wall_axis == "x" else x
        other_wall = pyw if wall_axis == "x" else pxw
        side = coord > wall_pos
        ci = np.where(side[:-1] != side[1:])[0]
        wall_doors = [d_ for d_ in doors if d_["axis"] == wall_axis]
        for d_ in wall_doors:
            d_.setdefault("succ", 0)
        for i2 in ci:
            oc = (other[i2] + other[i2 + 1]) / 2
            # which door of the span: the nearest along the wall
            dd = min(wall_doors, key=lambda q: abs(oc - (q["y"] if wall_axis == "x" else q["x"])))
            dd["succ"] += 1

    # attempts: entering the keepout (1.2m) with approach <0.6m from the door center
    for d_ in doors:
        dist = np.hypot(x - d_["x"], y - d_["y"])
        close = dist < 0.6
        # count approach "episodes" (consecutive close intervals)
        edges = np.diff(close.astype(int))
        n_appr = int((edges == 1).sum()) + (1 if close[0] else 0)
        results.append((seed, d_["name"], d_["g"], n_appr, d_.get("succ", 0)))

print(f"=== TOTALS: {tot_time/3600:.2f} h sim, {tot_path:.0f} m, dance episodes: {len(dance_total)} ===")
for s, t0 in dance_total:
    print(f"  dance: seed {s} @ t={t0:.0f}s")
print(f"{'seed':>12} {'door':>10} {'W':>5} {'appr':>5} {'cross':>6}")
tot_cross = 0
for s, n, g, a, c in results:
    tot_cross += c
    mark = " <<<" if c > 0 else ""
    print(f"{s:>12} {n:>10} {g:>5.2f} {a:>5} {c:>6}{mark}")
print(f"total crossings: {tot_cross}  ({tot_cross/(tot_time/3600):.1f}/h)")

# bins for the curve
bins = [(0.50, 0.60), (0.60, 0.66), (0.66, 0.72), (0.72, 0.80), (0.80, 0.95)]
print("\n=== SUCCESS vs WIDTH (doors with >=1 approach <0.6m) ===")
for lo, hi in bins:
    sel = [(a, c) for _, _, g, a, c in results if lo <= g < hi]
    appr = sum(a for a, _ in sel)
    doors_n = len(sel)
    doors_touched = sum(1 for a, c in sel if a > 0 or c > 0)
    doors_crossed = sum(1 for _, c in sel if c > 0)
    cr = sum(c for _, c in sel)
    print(f"  [{lo:.2f}-{hi:.2f}): {doors_n} doors, {doors_touched} approached, "
          f"{doors_crossed} crossed, {cr} total crossings")
