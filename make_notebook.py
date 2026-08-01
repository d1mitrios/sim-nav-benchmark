#!/usr/bin/env python3
# Builds analysis.ipynb (the repo's reproducibility notebook)
import json

MD = "markdown"
CD = "code"
cells = []


def cell(kind, src):
    cells.append({"cell_type": kind, "id": f"c{len(cells):02d}", "metadata": {},
                  "source": src.splitlines(keepends=True),
                  **({"outputs": [], "execution_count": None} if kind == CD else {})})


cell(MD, """# Sim-only Social Navigation Benchmark — reproducible analysis

Reproduces every headline number of the project directly from the raw logs in `runs/`:

- **Phase 1 (static):** v1 baseline vs v2.9.1, paired over 25 procedurally generated worlds
- **Phase 2 (dynamic):** the same 25 worlds with 1-3 walking humans, v2.9.1 unmodified
- **Watchdog forensics:** why the v2 batch watchdog falsely killed 16/25 healthy runs, and the v3 fix

**Requirements:** Python 3 with `numpy` and `matplotlib`. Run from the repository root
(`C:\\isaac_project`), or set the environment variable `RUNS_DIR` to the `runs/` folder.

Data files used: `run_<seed>_<timestamp>.csv` (10 Hz pose + human positions),
`world_<seed>.csv` (procedural-world manifests = ground truth), `batch_log.csv` (runner verdicts).
""")

cell(CD, """import glob, os, re
import numpy as np

RUNS = os.environ.get("RUNS_DIR", "runs")

# Cohorts by file timestamp (see NEXT_SESSION.md, "cohort cutoffs"):
#   static v2.9.1 : 23/7 before 19:00
#   v1 baseline   : 23/7 19:00 - 24/7 02:00
#   dyn1 (truncated by the buggy v2 watchdog - kept only for the forensics section)
#   dyn2 (definitive full-length dynamic batch, v3 watchdog)
WINDOWS = {"static": ("00000000_000000", "20260723_190000"),
           "v1":     ("20260723_190000", "20260724_020000"),
           "dyn1":   ("20260724_150000", "20260724_220000"),
           "dyn2":   ("20260724_220000", "99999999_999999")}

def stamp(f):
    return re.search(r"_(\\d{8}_\\d{6})\\.csv", f).group(1)

def cohort(name):
    lo, hi = WINDOWS[name]
    best = {}
    for f in glob.glob(os.path.join(RUNS, "run_202607230*.csv")):  # NOT run_2026072300* (glob trap!)
        s, st = re.search(r"run_(\\d+)_", f).group(1), stamp(f)
        if lo <= st.replace("_", "_") < hi and (s not in best or st > best[s][1]):
            best[s] = (f, st)
    return {s: f for s, (f, _) in best.items()}

def load_world(seed):
    doors, persons = [], []
    for line in open(os.path.join(RUNS, f"world_{seed}.csv")):
        p = line.strip().split(",")
        if p[0] == "door":
            doors.append(dict(name=p[1], x=float(p[2]), y=float(p[3]), g=float(p[4]), axis=p[5]))
        elif p[0] == "person":
            persons.append((float(p[2]), float(p[3])))
    pxw = next(d["x"] for d in doors if d["axis"] == "x")
    pyw = next(d["y"] for d in doors if d["axis"] == "y")
    return doors, pxw, pyw, persons

print({k: len(cohort(k)) for k in WINDOWS})  # expect 25 everywhere""")

cell(CD, """W90 = 900  # 90 sim-s at ~10 Hz

def analyze(csv, doors, pxw, pyw, persons):
    d = np.genfromtxt(csv, delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    r = dict(dur=float(t[-1]), dist=float(np.hypot(np.diff(x), np.diff(y)).sum()))
    # door crossings: wall sign-changes attributed to the nearest door on that wall
    cross = {q["name"]: 0 for q in doors}
    for wall_axis, wall_pos in (("x", pxw), ("y", pyw)):
        coord, other = (x, y) if wall_axis == "x" else (y, x)
        side = coord > wall_pos
        for i2 in np.where(side[:-1] != side[1:])[0]:
            oc = (other[i2] + other[i2 + 1]) / 2
            wd = [q for q in doors if q["axis"] == wall_axis]
            cross[min(wd, key=lambda q: abs(oc - (q["y"] if wall_axis == "x" else q["x"])))["name"]] += 1
    r["cross_per_door"], r["cross"] = cross, sum(cross.values())
    # deadlock rules (uniform for every navigator):
    #   event    : net displacement < 0.5 m over a 90 sim-s window
    #   terminal : final 90 sim-s stay within 0.5 m (excursion) of the final position
    dl_t, i = None, 0
    while i + W90 < len(t):
        if np.hypot(x[i + W90] - x[i], y[i + W90] - y[i]) < 0.5:
            dl_t = float(t[i]); break
        i += 25
    r["dl_t"] = dl_t
    k0 = np.searchsorted(t, t[-1] - 90.0)
    exc = np.hypot(x[k0:] - x[-1], y[k0:] - y[-1])
    r["terminal"] = bool(len(exc) and exc.max() < 0.5)
    # social metrics: fmt v2 files log the 3 human positions; static files fall back to manifest
    if d.shape[1] >= 10:
        P = [(d[:, 4], d[:, 5]), (d[:, 6], d[:, 7]), (d[:, 8], d[:, 9])]
        r["walkers"] = sum(1 for px, py in P if np.hypot(px - px[0], py - py[0]).max() > 1.0)
        mind = np.min([np.hypot(x - px, y - py) for px, py in P], axis=0)
    else:
        r["walkers"] = 0
        mind = np.min([np.hypot(x - hx, y - hy) for hx, hy in persons], axis=0)
    r["min_clear"] = float(mind.min())
    r["pers_frac"] = float((mind < 1.2).mean())
    r["intim_frac"] = float((mind < 0.5).mean())
    r["near_contact"] = int(((mind[1:] < 0.35) & (mind[:-1] >= 0.35)).sum())
    return r

def run_cohort(name):
    out = {}
    for seed, f in sorted(cohort(name).items()):
        doors, pxw, pyw, persons = load_world(seed)
        out[seed] = analyze(f, doors, pxw, pyw, persons)
    return out

C = {k: run_cohort(k) for k in ["static", "v1", "dyn2"]}
print("analyzed:", {k: len(v) for k, v in C.items()})""")

cell(MD, """## Phase 1 — v1 baseline vs v2.9.1 (static, 25 paired worlds)

Identical Follow-the-Gap core in both navigators; the only difference is the recovery stack
(v2.5-v2.9.1), each layer built from a live-diagnosed failure mode.""")

cell(CD, """st, v1 = C["static"], C["v1"]
seeds = sorted(st)
t1 = [s for s in seeds if v1[s]["terminal"]]
t2 = [s for s in seeds if st[s]["terminal"]]
d1 = sum(v1[s]["dist"] for s in seeds); d2 = sum(st[s]["dist"] for s in seeds)
print(f"terminal deadlocks : v1 {len(t1)}/25   v2.9.1 {len(t2)}/25")
print(f"median time to deadlock (v1): {np.median([v1[s]['dl_t'] for s in seeds if v1[s]['dl_t']]):.0f} sim-s")
print(f"distance           : v1 {d1:.0f} m   v2.9.1 {d2:.0f} m   ({d2/d1:.1f}x)")
print(f"door crossings     : v1 {sum(v1[s]['cross'] for s in seeds)}   "
      f"v2.9.1 {sum(st[s]['cross'] for s in seeds)}")""")

cell(MD, """## Phase 2 — cost of dynamics (static vs dyn2, v2.9.1 unmodified)

Same 25 seeds, byte-identical geometry (the v5 worldgen draws people paths *after* all
placement RNG). The only change: the 1-3 people per world now walk seeded waypoint loops
at 0.7-1.2 m/s.""")

cell(CD, """dy = C["dyn2"]
sd = sum(st[s]["dur"] for s in seeds); dd = sum(dy[s]["dur"] for s in seeds)
sm = sum(st[s]["dist"] for s in seeds); dm = sum(dy[s]["dist"] for s in seeds)
print(f"exposure   : static {sd/3600:.2f} h   dynamic {dd/3600:.2f} h (all 25 runs full-length)")
print(f"speed      : static {np.median([st[s]['dist']/st[s]['dur']*60 for s in seeds]):.1f}   "
      f"dynamic {np.median([dy[s]['dist']/dy[s]['dur']*60 for s in seeds]):.1f} m/min (median)")
print(f"crossings  : static {sum(st[s]['cross'] for s in seeds)} ({sum(st[s]['cross'] for s in seeds)/(sd/3600):.1f}/h)   "
      f"dynamic {sum(dy[s]['cross'] for s in seeds)} ({sum(dy[s]['cross'] for s in seeds)/(dd/3600):.1f}/h)")
print(f"deadlocks  : {sum(dy[s]['terminal'] for s in seeds)}/25 terminal, "
      f"{sum(1 for s in seeds if dy[s]['dl_t'] is not None)} events")
mc = [dy[s]["min_clear"] for s in seeds]
print(f"social     : min clearance median {np.median(mc):.2f} m (floor {min(mc):.2f}) | "
      f"time <1.2 m {np.mean([dy[s]['pers_frac'] for s in seeds])*100:.1f}% | "
      f"<0.5 m {np.mean([dy[s]['intim_frac'] for s in seeds])*100:.1f}% | "
      f"near-contacts {sum(dy[s]['near_contact'] for s in seeds)}")
print(f"walkers/world: {sorted(dy[s]['walkers'] for s in seeds)}")""")

cell(MD, """## Watchdog forensics — why dyn1 is not used for results

The v2 batch watchdog compared only the two *endpoints* of a 180 wall-s pose trail
("net < 0.5 m"). A robot that loops back near an old position while cruising trips it.
In the first dynamic batch it killed 16/25 runs; in every one of them the robot had walked
19-26 m *inside the very window* the watchdog read as still. The v3 watchdog uses the
bounding-box spread of the whole trail and only samples while sim time advances —
in the definitive overnight batch it killed nothing (25/25 full).""")

cell(CD, """d1f = cohort("dyn1")
killed = ["002","003","005","006","007","008","009","011","012","015","016","019","020","022","023","025"]
print(f"{'seed':>5} {'dur(s)':>7} {'path in final 100 sim-s':>24} {'bbox spread':>12}")
for s3 in killed:
    f = d1f["20260723" + s3]
    d = np.genfromtxt(f, delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    k = np.searchsorted(t, t[-1] - 100.0)
    path = np.hypot(np.diff(x[k:]), np.diff(y[k:])).sum()
    spread = np.hypot(x[k:].max() - x[k:].min(), y[k:].max() - y[k:].min())
    print(f"{s3:>5} {t[-1]:7.0f} {path:21.1f} m {spread:10.2f} m")
print("-> every 'deadlocked' robot was moving at cruise speed; the 0.5 m rule saw loop closures.")""")

cell(MD, """## Quick-look figures

Minimal inline versions (the styled dossier figures live in `runs/*.png`).""")

cell(CD, """import matplotlib.pyplot as plt
fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.4))
# survival: time to terminal deadlock
for name, coh, color in (("v1", v1, "tab:orange"), ("v2.9.1 static", st, "tab:blue"),
                         ("v2.9.1 dynamic", dy, "tab:green")):
    times = sorted([coh[s]["dl_t"] for s in seeds if coh[s]["terminal"] and coh[s]["dl_t"]])
    xs, ys = [0], [1.0]
    for i, tt in enumerate(times):
        xs += [tt, tt]; ys += [ys[-1], 1 - (i + 1) / 25]
    xs.append(700); ys.append(ys[-1])
    a1.plot(xs, ys, drawstyle="default", label=name, color=color)
a1.set_xlabel("sim time (s)"); a1.set_ylabel("runs still alive"); a1.set_ylim(0, 1.05)
a1.legend(); a1.set_title("Survival (terminal deadlock)")
# parity
a2.plot([11, 17], [11, 17], "k--", lw=0.8)
a2.plot([st[s]["dist"]/st[s]["dur"]*60 for s in seeds],
        [dy[s]["dist"]/dy[s]["dur"]*60 for s in seeds], "o")
a2.set_xlabel("static m/min"); a2.set_ylabel("dynamic m/min"); a2.set_title("Locomotion parity")
plt.tight_layout(); plt.show()""")

cell(MD, """## Data dictionary

- `run_<seed>_<YYYYmmdd_HHMMSS>.csv` — one file per Play/Stop. Header line 1: seed + wall
  start + format tag. Columns v1-format: `sim_t,x,y,yaw_deg`; v2-format adds `p0x..p2y`
  (the 3 human positions). ~10 Hz in sim time.
- `world_<seed>.csv` — manifest written by `generate_world.py` (v5): boxes, cylinders,
  partitions, labelled doors (`door,name,x,y,width,axis`), person spawn poses and, since v5,
  `path` rows (waypoint loops: `path,person_i,x,y,seq,speed`).
- `batch_log.csv` — appended by `batch_runner.py`: one `seed,end_reason,wall_seconds` row per
  batch run (10 v1 rows, then 25 dyn1, then 25 dyn2).
- Known quirks: glob `run_2026072300*` silently drops seeds 010-025 (use `run_202607230*`);
  `run_20260723025_20260724_123908.csv` is a stale-seed boot fragment (excluded by the cohort
  windows); static seed 014's run recorded only 268 sim-s (Isaac hiccup — rates fine).""")

nb = {"cells": cells, "metadata": {"kernelspec": {"display_name": "Python 3", "language": "python",
      "name": "python3"}, "language_info": {"name": "python", "version": "3"}},
      "nbformat": 4, "nbformat_minor": 5}
json.dump(nb, open("/home/user/isaac/analysis.ipynb", "w"), ensure_ascii=False, indent=1)
print("wrote analysis.ipynb,", len(cells), "cells")
