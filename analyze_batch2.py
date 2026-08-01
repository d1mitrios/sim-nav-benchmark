#!/usr/bin/env python3
# Batch #2 (night of 24-25/7, v3 watchdog): FULL 25x20min dynamic runs.
# Paired static vs dynamic-2 + reference to the truncated batch-1 + RTF/lag diagnosis.
import glob
import re
import json
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
W90 = 900


def load_world(seed):
    doors, persons = [], []
    for line in open(f"{RUNS}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "door":
            doors.append(dict(name=p[1], x=float(p[2]), y=float(p[3]), g=float(p[4]), axis=p[5]))
        elif p[0] == "person":
            persons.append((float(p[2]), float(p[3])))
    pxw = next(d["x"] for d in doors if d["axis"] == "x")
    pyw = next(d["y"] for d in doors if d["axis"] == "y")
    return doors, pxw, pyw, persons


def newest_per_seed(files):
    best = {}
    for f in files:
        seed = re.search(r"run_(\d+)_", f).group(1)
        stamp = re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)
        if seed not in best or stamp > best[seed][1]:
            best[seed] = (f, stamp)
    return {s: f for s, (f, _) in best.items()}


def analyze(csv, doors, pxw, pyw, persons_static):
    d = np.genfromtxt(csv, delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    seg = np.hypot(np.diff(x), np.diff(y))
    r = dict(dur=float(t[-1]), dist=float(seg.sum()))
    cross = {q["name"]: 0 for q in doors}
    for wall_axis, wall_pos in (("x", pxw), ("y", pyw)):
        coord = x if wall_axis == "x" else y
        other = y if wall_axis == "x" else x
        side = coord > wall_pos
        ci = np.where(side[:-1] != side[1:])[0]
        wd = [q for q in doors if q["axis"] == wall_axis]
        for i2 in ci:
            oc = (other[i2] + other[i2 + 1]) / 2
            dd = min(wd, key=lambda q: abs(oc - (q["y"] if wall_axis == "x" else q["x"])))
            cross[dd["name"]] += 1
    r["cross_per_door"] = cross
    r["cross"] = sum(cross.values())
    # excursion-still (90s) + terminal + dance
    still = 0
    i = 0
    while i + W90 < len(t):
        if np.hypot(x[i:i + W90] - x[i], y[i:i + W90] - y[i]).max() < 0.5:
            still += 1
            i += W90
        else:
            i += 100
    r["still"] = still
    k0 = np.searchsorted(t, t[-1] - 90.0)
    exc = np.hypot(x[k0:] - x[-1], y[k0:] - y[-1])
    r["terminal"] = bool(len(exc) and exc.max() < 0.5)
    W, dance = 600, 0
    i = 0
    while i + W < len(t):
        if np.hypot(x[i + W] - x[i], y[i + W] - y[i]) < 0.35 and seg[i:i + W].sum() > 2.5:
            dance += 1
            i += W
        else:
            i += 100
    r["dance"] = dance
    # social
    if d.shape[1] >= 10:
        P = [(d[:, 4], d[:, 5]), (d[:, 6], d[:, 7]), (d[:, 8], d[:, 9])]
        r["walkers"] = sum(1 for px, py in P
                           if float(np.hypot(px - px[0], py - py[0]).max()) > 1.0)
        mind = np.min([np.hypot(x - px, y - py) for px, py in P], axis=0)
        r["min_clear"] = float(mind.min())
        r["pers_frac"] = float((mind < 1.2).mean())
        r["intim_frac"] = float((mind < 0.5).mean())
        r["near_contact"] = int(((mind[1:] < 0.35) & (mind[:-1] >= 0.35)).sum())
    else:
        mind = np.min([np.hypot(x - px, y - py) for px, py in persons_static], axis=0)
        r["walkers"] = 0
        r["min_clear"] = float(mind.min())
        r["pers_frac"] = float((mind < 1.2).mean())
        r["intim_frac"] = float((mind < 0.5).mean())
        r["near_contact"] = 0
    return r


def stamp(f):
    return re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)


all_files = glob.glob(f"{RUNS}/run_202607230*.csv")
st_files = newest_per_seed([f for f in all_files if stamp(f) < "20260723_19"])
d1_files = newest_per_seed([f for f in all_files if "20260724_15" <= stamp(f) < "20260724_22"])
d2_files = newest_per_seed([f for f in all_files if stamp(f) >= "20260724_22"])
print(f"static {len(st_files)} | dyn1 {len(d1_files)} | dyn2 {len(d2_files)}")

rows = []
print(f"{'seed':>5} | {'ST m/min':>8} {'cr':>3} | {'D2 dur':>6} {'m/min':>6} {'cr':>3} {'TD':>3} "
      f"{'stil':>4} {'dnc':>4} {'wlk':>3} {'minC':>5} {'<1.2':>5} {'<0.5':>5} {'nc':>3} {'RTF':>5}")
for seed in sorted(d2_files):
    doors, pxw, pyw, persons = load_world(seed)
    S = analyze(st_files[seed], doors, pxw, pyw, persons)
    D = analyze(d2_files[seed], doors, pxw, pyw, persons)
    rows.append(dict(seed=seed, S=S, D=D, doors=[(q["name"], q["g"]) for q in doors]))
    print(f"{seed[-3:]:>5} | {S['dist']/S['dur']*60:8.1f} {S['cross']:3d} | "
          f"{D['dur']:6.0f} {D['dist']/D['dur']*60:6.1f} {D['cross']:3d} "
          f"{'X' if D['terminal'] else '-':>3} {D['still']:4d} {D['dance']:4d} {D['walkers']:3d} "
          f"{D['min_clear']:5.2f} {D['pers_frac']*100:4.1f}% {D['intim_frac']*100:4.1f}% "
          f"{D['near_contact']:3d} {D['dur']/1201:5.2f}")

S_ = [r["S"] for r in rows]
D_ = [r["D"] for r in rows]
sd, dd = sum(s["dur"] for s in S_), sum(x_["dur"] for x_ in D_)
sm, dm = sum(s["dist"] for s in S_), sum(x_["dist"] for x_ in D_)
sc, dc = sum(s["cross"] for s in S_), sum(x_["cross"] for x_ in D_)
print(f"\n=== BATCH #2 vs STATIC ({len(rows)} pairs, ALL full 20min) ===")
print(f"sim: static {sd/3600:.2f} h, dyn2 {dd/3600:.2f} h")
print(f"m/min: static {sm/sd*60:.1f} (median {np.median([s['dist']/s['dur']*60 for s in S_]):.1f}), "
      f"dyn2 {dm/dd*60:.1f} (median {np.median([x_['dist']/x_['dur']*60 for x_ in D_]):.1f})")
print(f"crossings: static {sc} ({sc/(sd/3600):.1f}/h), dyn2 {dc} ({dc/(dd/3600):.1f}/h)")
print(f"terminal: {sum(x_['terminal'] for x_ in D_)}/25, excursion-still: {sum(x_['still'] for x_ in D_)}, "
      f"dance: dyn2 {sum(x_['dance'] for x_ in D_)} vs static {sum(s['dance'] for s in S_)}")
mc = [x_["min_clear"] for x_ in D_]
print(f"social: min clearance median {np.median(mc):.2f} (min {min(mc):.2f}), "
      f"<1.2m {np.mean([x_['pers_frac'] for x_ in D_])*100:.1f}%, "
      f"<0.5m {np.mean([x_['intim_frac'] for x_ in D_])*100:.1f}%, "
      f"near-contacts {sum(x_['near_contact'] for x_ in D_)}")
print(f"walkers: {sorted(x_['walkers'] for x_ in D_)}")
rtf = [x_["dur"] / 1201 for x_ in D_]
print(f"overnight RTF per run (1→25): " + " ".join(f"{v:.2f}" for v in rtf))
print(f"RTF first 5: {np.mean(rtf[:5]):.2f} → last 5: {np.mean(rtf[-5:]):.2f}")

# crossings per door width (paired)
bins = [(0.50, 0.60), (0.60, 0.66), (0.66, 0.72), (0.72, 0.80), (0.80, 0.95)]
print("\n=== CROSSINGS per width (static -> dyn2) ===")
for lo, hi in bins:
    s_cr = d_cr = nd = 0
    for r in rows:
        for name, g in r["doors"]:
            if lo <= g < hi:
                nd += 1
                s_cr += r["S"]["cross_per_door"][name]
                d_cr += r["D"]["cross_per_door"][name]
    print(f"  [{lo:.2f}-{hi:.2f}): {nd} doors, {s_cr} -> {d_cr}")

json.dump([{"seed": r["seed"], "S": r["S"], "D": r["D"], "doors": r["doors"]} for r in rows],
          open("/home/user/isaac/batch2_rows.json", "w"))
print("\nsaved batch2_rows.json")
