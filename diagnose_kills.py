#!/usr/bin/env python3
# What exactly did the runner kill in the 16 "terminal_deadlock" dynamic runs?
# Suspicion: the runner's rule is TWO-POINT (pose now vs pose 180 wall-s earlier)
# -> it fires falsely when the robot RETURNS near an old position while moving.
# Check per killed run, over the final ~100 sim-s interval (=180 wall-s):
#   spread  = bounding-box diagonal (how much space it covered)
#   path    = path length (was it moving?)
#   v_mean  = mean speed
#   two_pt  = net endpoint displacement (what the runner "saw")
#   h_min   = min human distance
# Plus: dance episodes (60s net<0.35 & path>2.5m) and excursion-still (90s, exc<0.5) over the whole run.
import glob
import re
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
DYN_LO = "20260724_15"
KILLED = ["002", "003", "005", "006", "007", "008", "009", "011", "012",
          "015", "016", "019", "020", "022", "023", "025"]

files = {re.search(r"run_(\d+)_", f).group(1): f
         for f in glob.glob(f"{RUNS}/run_202607230*.csv")
         if re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) >= DYN_LO}

print(f"{'seed':>5} {'dur':>5} | final ~100 sim-s: {'spread':>7} {'path':>6} {'v_mean':>7} "
      f"{'two_pt':>7} {'h_min':>6} | {'dance':>5} {'exc-still':>9}")
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

    # dance over the whole run
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
