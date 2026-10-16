#!/usr/bin/env python3
# What did the first watchdog kill in the 16 "terminal_deadlock" runs of the first dynamic batch?
# Its rule compared two points (pose now vs pose 180 wall-s earlier), so it fires falsely
# when the robot comes back near an old position while moving.
# Check per killed run, over the final ~100 sim-s (about 180 wall-s at the batch's RTF):
#   spread  = bounding-box diagonal (how much space it covered)
#   path    = path length (was it moving?)
#   v_mean  = mean speed
#   two_pt  = net endpoint displacement (what the runner "saw")
#   h_min   = min human distance
# Plus, over the whole run: back-and-forth episodes (60 s, net < 0.35 m and path > 2.5 m)
# and still episodes (90 s within 0.5 m). Writes runs/watchdog_kills.json.
import os
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import glob
import re
import numpy as np

RUNS = RUNS_DIR
DYN_LO = "20260724_15"
# seeds the first watchdog killed in the first dynamic batch (runs/batch_log.csv rows 11-35)
import csv
_LOG = list(csv.reader(open(os.path.join(RUNS_DIR, "batch_log.csv"))))[1:]
KILLED = [r[0][-3:] for r in _LOG[10:35] if r[1] == "terminal_deadlock"]

files = {re.search(r"run_(\d+)_", f).group(1): f
         for f in sorted(glob.glob(f"{RUNS}/run_202607230*.csv"))
         if re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) >= DYN_LO
         and re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) < "20260724_22"}

print(f"{'seed':>5} {'dur':>5} | final ~100 sim-s: {'spread':>7} {'path':>6} {'v_mean':>7} "
      f"{'two_pt':>7} {'h_min':>6} | {'dance':>5} {'exc-still':>9}")
out = []
for s3 in KILLED:
    seed = "20260723" + s3
    d = np.genfromtxt(files[seed], delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    P = [(d[:, 4], d[:, 5]), (d[:, 6], d[:, 7]), (d[:, 8], d[:, 9])]
    mind = np.min([np.hypot(x - px, y - py) for px, py in P], axis=0)
    k = np.searchsorted(t, t[-1] - 100.0)
    xs, ys = x[k:], y[k:]
    spread = float(np.hypot(xs.max() - xs.min(), ys.max() - ys.min()))
    path = float(np.hypot(np.diff(xs), np.diff(ys)).sum())
    v_mean = path / (t[-1] - t[k]) if t[-1] > t[k] else 0.0
    two_pt = float(np.hypot(xs[-1] - xs[0], ys[-1] - ys[0]))
    h_min = float(mind[k:].min())

    # back-and-forth episodes over the whole run
    W, dance = 600, 0
    seg = np.hypot(np.diff(x), np.diff(y))
    i = 0
    while i + W < len(t):
        if np.hypot(x[i + W] - x[i], y[i + W] - y[i]) < 0.35 and seg[i:i + W].sum() > 2.5:
            dance += 1
            i += W
        else:
            i += 100
    # excursion-still: 90s where max excursion from the window start < 0.5
    W9, still = 900, 0
    i = 0
    while i + W9 < len(t):
        if np.hypot(x[i:i + W9] - x[i], y[i:i + W9] - y[i]).max() < 0.5:
            still += 1
            i += W9
        else:
            i += 100
    print(f"{s3:>5} {t[-1]:5.0f} | {spread:7.2f} {path:6.1f} {v_mean:7.2f} "
          f"{two_pt:7.2f} {h_min:6.2f} | {dance:5d} {still:9d}")
    out.append(dict(seed=seed, dur=float(t[-1]), spread=spread, path=path, two_pt=two_pt))

import json
json.dump(out, open(os.path.join(RUNS_DIR, "watchdog_kills.json"), "w"), indent=1)
print(f"saved watchdog_kills.json ({len(out)} killed runs, path in the final window "
      f"{min(o['path'] for o in out):.1f}-{max(o['path'] for o in out):.1f} m)")
