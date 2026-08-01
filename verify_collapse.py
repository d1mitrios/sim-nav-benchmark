#!/usr/bin/env python3
# Verification of the "amcl collapse after the first crossing" hypothesis:
# join missions epochs <-> metrics CSV (ground truth) and for each mission end
# compute the robot's TRUE position and its distance from the goal.
# - SUCCEEDED with a large true distance = fake success (localization delusion)
# - home-ABORT: where the robot really was when it "could not" go to (0,0)
# The metrics wall_start is LOCAL time -> offset scan so that RTF∈[0.2,0.9] (known trap).
import glob
import math
import time
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
SEEDS = [f"202607230{i:02d}" for i in range(1, 10)]


def load_metrics(seed):
    f = sorted(glob.glob(f"{RUNS}/run_{seed}_20260730_1*.csv"))[-1]
    hdr = open(f).readline()
    ws = hdr.split("wall_start=")[1].split(" fmt")[0]
    t0 = time.mktime(time.strptime(ws, "%Y-%m-%d %H:%M:%S"))
    d = np.genfromtxt(f, delimiter=",", skip_header=2)
    return f, t0, d


def pick_offset(t0, sim_end, e_first, e_last):
    # we want an offset such that (e_last - (t0+off)) gives an RTF in [0.2, 0.9]
    for off in (0, -3600, -7200, -10800, -14400, 3600):
        wall = (e_last - (t0 + off))
        if wall <= 0:
            continue
        rtf = sim_end / wall
        if 0.2 <= rtf <= 0.95:
            return off, rtf
    return None, None


def sim_at(epoch, t0, off, rtf):
    return (epoch - (t0 + off)) * rtf


print(f"{'seed':13s} {'mission':11s} {'res':9s} {'goal':>14s} {'true pos @end':>16s} {'d(goal)':>8s}")
for seed in SEEDS:
    mf = sorted(glob.glob(f"{RUNS}/missions_{seed}_20260730_1*.csv"))[-1]
    rows = [l.strip().split(",") for l in open(mf).readlines()[1:]]
    _, t0, d = load_metrics(seed)
    sim_end = d[-1, 0]
    e_first = float(rows[0][6])
    e_last = float(rows[-1][7])
    off, rtf = pick_offset(t0, sim_end, e_first, e_last)
    if off is None:
        print(f"{seed}: no offset found (t0={t0}, sim_end={sim_end:.0f})")
        continue
    for r in rows:
        nm, gx, gy, att, res, dur, e0, e1 = r[0], float(r[1]), float(r[2]), r[3], r[4], r[5], float(r[6]), float(r[7])
        s1 = sim_at(e1, t0, off, rtf)
        i = np.searchsorted(d[:, 0], s1)
        i = min(max(i, 0), len(d) - 1)
        x, y = d[i, 1], d[i, 2]
        dist = math.hypot(x - gx, y - gy)
        mark = ""
        if res == "SUCCEEDED" and dist > 0.8:
            mark = "  <-- FAKE SUCCESS"
        print(f"{seed:13s} {nm:11s} {res+'/'+att:9s} ({gx:5.2f},{gy:6.2f}) "
              f"({x:6.2f},{y:6.2f}) {dist:7.2f}m{mark}")
    # total motion in the run
    path = np.sum(np.hypot(np.diff(d[:, 1]), np.diff(d[:, 2])))
    print(f"{seed:13s} [run: sim {sim_end:.0f}s, path {path:.1f} m, RTF {rtf:.2f}, offset {off/3600:.0f}h]")
    print()
