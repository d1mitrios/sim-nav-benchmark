#!/usr/bin/env python3
# Safety of the October mission experiments: did the robot reach its goals by touching things?
# Usage: python3 safety_missions.py
# The revised configuration takes the collision monitor out of the command path, so the question
# has to be answered from ground truth. For every tour of the four arms (see stats_missions.py)
# the script places the robot's true body (chassis 0.40 x 0.30 m, wheels out to 0.195 m from the
# centreline at x -0.15..-0.05, camera to x 0.22, all relative to the chassis centre that the
# metrics logger records) at every second logged pose (5 Hz in simulated time) and measures the
# signed distance from its outline to the manifest geometry (walls, partitions, boxes, cylinders)
# and to the people (0.25 m cylinders at their logged positions). Reported per arm:
#  - static contact: share of tours whose body came within 1 cm of a static obstacle, and the
#    simulated seconds spent that close (the physics engine keeps bodies from overlapping, so a
#    touch shows as a distance of about zero)
#  - person contact: the same with the people, plus walker pushes (contacts.py)
#  - the median over tours of the minimum static clearance
# Writes runs/missions_safety.json.
import os
import sys
import json
import math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_missions2 as am
import contacts
from stats_missions import ARMS

TOUCH_M = 0.01


def body_outline(step=0.02):
    pts = []

    def rect(x0, x1, y0, y1):
        for x in np.arange(x0, x1 + 1e-9, step):
            pts.extend([(x, y0), (x, y1)])
        for y in np.arange(y0, y1 + 1e-9, step):
            pts.extend([(x0, y), (x1, y)])
    rect(-0.20, 0.20, -0.15, 0.15)            # chassis
    rect(-0.15, -0.05, 0.155, 0.195)          # left wheel
    rect(-0.15, -0.05, -0.195, -0.155)        # right wheel
    rect(0.20, 0.22, -0.025, 0.025)           # camera
    return np.array(pts)


BODY = body_outline()


def load_geometry(seed):
    boxes, cyls = [], []
    for line in open(f"{am.RUNS}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "box":
            boxes.append([float(v) for v in p[2:7]])
        elif p[0] == "cyl":
            cyls.append([float(p[2]), float(p[3]), float(p[4])])
    for bx, by, sx, sy in [(0, 10.0, 20.5, 0.5), (0, -10.0, 20.5, 0.5), (10.0, 0, 0.5, 20.5), (-10.0, 0, 0.5, 20.5)]:
        boxes.append([bx, by, sx, sy, 0.0])
    return np.array(boxes), np.array(cyls) if cyls else np.zeros((0, 3))


def sdf_static(px, py, boxes, cyls):
    """Signed distance of each point (arrays) to the nearest static obstacle (negative inside)."""
    cx, cy, sx, sy, yaw = [boxes[:, k][:, None] for k in range(5)]
    th = np.radians(yaw)
    dx, dy = px[None, :] - cx, py[None, :] - cy
    lx = np.cos(th) * dx + np.sin(th) * dy
    ly = -np.sin(th) * dx + np.cos(th) * dy
    qx, qy = np.abs(lx) - sx / 2, np.abs(ly) - sy / 2
    outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    inside = np.minimum(np.maximum(qx, qy), 0)
    d = (outside + inside).min(axis=0)
    if len(cyls):
        dc = np.hypot(px[None, :] - cyls[:, 0:1], py[None, :] - cyls[:, 1:2]) - cyls[:, 2:3]
        d = np.minimum(d, dc.min(axis=0))
    return d


def tour_safety(seed, mfile):
    rows = am.read_missions(mfile)
    rf = am.run_for(seed, am.ts_of(mfile))
    if not rf or any(not r["sim_end"] for r in rows):
        return None
    d = np.genfromtxt(rf, delimiter=",", skip_header=2)
    s0, s1 = float(rows[0]["sim_start"]), float(rows[-1]["sim_end"])
    seg = d[(d[:, 0] >= s0) & (d[:, 0] <= s1)][::2]
    if len(seg) < 3:
        return None
    boxes, cyls = load_geometry(seed)
    stat, pers = [], []
    for row in seg:
        t, x, y, yaw = row[0], row[1], row[2], math.radians(row[3])
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + c * BODY[:, 0] - s * BODY[:, 1]
        py = y + s * BODY[:, 0] + c * BODY[:, 1]
        stat.append(float(sdf_static(px, py, boxes, cyls).min()))
        if seg.shape[1] >= 10:
            pd = min(float((np.hypot(px - row[4 + 2 * k], py - row[5 + 2 * k]) - 0.25).min()) for k in range(3))
            pers.append(pd)
    stat, pers = np.array(stat), np.array(pers) if pers else np.array([9.9])
    dt = float(np.median(np.diff(seg[:, 0])))
    full = d[(d[:, 0] >= s0) & (d[:, 0] <= s1)]
    pushes = contacts.events(full[:, 0], full[:, 1], full[:, 2],
                             [(full[:, 4 + 2 * k], full[:, 5 + 2 * k]) for k in range(3)]) if full.shape[1] >= 10 else []
    def episodes(a):
        on = a < TOUCH_M
        return int(on[0]) + int(((~on[:-1]) & on[1:]).sum())
    path = float(np.sum(np.hypot(np.diff(full[:, 1]), np.diff(full[:, 2]))))
    return dict(min_static=float(stat.min()), static_touch_s=float((stat < TOUCH_M).sum() * dt),
                static_episodes=episodes(stat), min_person=float(pers.min()),
                person_touch_s=float((pers < TOUCH_M).sum() * dt), person_episodes=episodes(pers),
                pushes=len(pushes), tour_s=float(s1 - s0), path_m=path)


def main():
    out = {}
    for name, (cfg, mk, d0, d1, md5) in ARMS.items():
        files = am.pick_files(cfg, mk, d0, d1, set(am.BENCH_SEEDS), md5)
        res = {}
        for seed, f in sorted(files.items()):
            r = tour_safety(seed, f)
            if r:
                res[seed] = r
        if not res:
            continue
        v = list(res.values())
        n = len(v)
        st = sum(1 for r in v if r["static_touch_s"] > 0)
        pt = sum(1 for r in v if r["person_touch_s"] > 0)
        summ = dict(tours=n, tours_static_touch=st, static_touch_s=round(sum(r["static_touch_s"] for r in v), 1),
                    median_min_static=round(float(np.median([r["min_static"] for r in v])), 3),
                    tours_person_touch=pt, person_touch_s=round(sum(r["person_touch_s"] for r in v), 1),
                    pushes=sum(r["pushes"] for r in v), tour_s=round(sum(r["tour_s"] for r in v)),
                    km=round(sum(r["path_m"] for r in v) / 1000.0, 2),
                    static_episodes=sum(r["static_episodes"] for r in v),
                    person_episodes=sum(r["person_episodes"] for r in v), per_tour=res)
        out[name] = summ
        km = max(summ["km"], 1e-6)
        print(f"{name:10s} tours {n:2d}, {summ['km']:.2f} km | static: touch in {st:2d} tours, "
              f"{summ['static_episodes']} episodes ({summ['static_episodes'] / km:.1f}/km), "
              f"{summ['static_touch_s']:.0f} of {summ['tour_s']} sim-s, median min clearance "
              f"{summ['median_min_static']:.3f} m | people: touch in {pt:2d} tours, "
              f"{summ['person_episodes']} episodes ({summ['person_episodes'] / km:.1f}/km), "
              f"walker pushes {summ['pushes']} ({summ['pushes'] / km:.1f}/km)")
    json.dump(out, open(os.path.join(am.RUNS, "missions_safety.json"), "w"), indent=1)
    print("results -> runs/missions_safety.json")


if __name__ == "__main__":
    main()
