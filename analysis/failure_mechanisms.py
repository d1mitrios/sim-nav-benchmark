#!/usr/bin/env python3
# Which mechanism ended each failed goal attempt? (experiments 6, 7, 8 and 5b)
# Usage: python3 failure_mechanisms.py
# Every attempt that did not end in a real arrival gets one primary label, the first that applies:
#  refused       the planner had no route: from the robot's true position the goal is not reachable
#                on the map Nav2 used at the configuration's planner radius (breadth-first search,
#                unknown = free)
#  stop circle   the robot stood still with the centre of the lidar closer than 0.23 m to an
#                obstacle or a person (v9's collision monitor stops every motion there) while the
#                controller kept commanding motion
#  localization  the AMCL estimate was more than 1 m from the true position at the end
#  deadband      the robot moved less than 0.5 m while more than half of the final commands were
#                slower than the drive executes (0.03 m/s and 0.15 rad/s)
#  walker        a walker pushed the robot (contacts.py) or stood within 0.5 m at the end
#  phantom       SLAM maps only: the shortest route crossed unexplored cells that are blocked in
#                truth (unknown_paths.py)
#  gave up       none of the above, the attempt ABORTED after the robot moved at least 3 m (the
#                behaviour tree ran out of recoveries on the way)
#  out of time   none of the above, TIMEOUT after the robot moved at least 3 m
#  stuck         none of the above, the robot moved less than 0.5 m
#  other         none of these
# Each failed attempt also records whether it ended within 1 m of a door centre.
# Writes runs/missions_failures.json.
import os
import sys
import csv
import json
import math
import numpy as np
from collections import Counter
from scipy.ndimage import binary_dilation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_missions2 as am
import evaluate_map as em
import contacts
from stats_missions import ARMS
from safety_missions import load_geometry, sdf_static
from unknown_paths import bfs_path, disk

ORDER = ["refused", "stop circle", "localization", "deadband", "walker", "phantom", "gave up",
         "out of time", "stuck", "other"]


def label_attempts(arm):
    cfg, mk, d0, d1, md5 = ARMS[arm]
    files = am.pick_files(cfg, mk, d0, d1, set(am.BENCH_SEEDS), md5)
    radius = am.planner_radius(cfg, md5 or "")
    offset = am.amcl_offset(cfg, md5 or "")
    labels = []
    for seed, f in sorted(files.items()):
        rf = am.run_for(seed, am.ts_of(f))
        d = np.genfromtxt(rf, delimiter=",", skip_header=2)
        mpath = f"{am.RUNS}/truth_{seed}.yaml" if mk == "truth" else f"{am.RUNS}/world_{seed}.yaml"
        occ, free, res, origin = em.load_map(mpath)
        rc = int(round(radius / res))
        blocked = binary_dilation(occ, structure=disk(rc))
        phantom = None
        if mk == "slam":
            gt, _ = em.render_truth(f"{am.RUNS}/world_{seed}.csv", res, occ.shape, origin)
            phantom = (~occ & ~free) & binary_dilation(gt, structure=disk(rc)) & ~blocked
        boxes, cyls = load_geometry(seed)
        doors = [(float(p[2]), float(p[3])) for p in (l.strip().split(",") for l in open(f"{am.RUNS}/world_{seed}.csv"))
                 if p[0] == "door"]
        amcl = list(csv.DictReader(open(f.replace("missions_", "amcl_")))) if os.path.exists(
            f.replace("missions_", "amcl_")) else []
        cmd = list(csv.DictReader(open(f.replace("missions_", "cmdvel_")))) if os.path.exists(
            f.replace("missions_", "cmdvel_")) else []

        def cell(x, y):
            return int(round((y - origin[1]) / res)), int(round((x - origin[0]) / res))
        for r in am.read_missions(f):
            if not r["sim_end"]:
                continue
            s0, s1 = float(r["sim_start"]), float(r["sim_end"])
            gx, gy = float(r["goal_x"]), float(r["goal_y"])
            i1, x1, y1 = am.truth_at(d, s1)
            if r["result"] == "SUCCEEDED" and math.hypot(x1 - gx, y1 - gy) <= am.STRICT_M:
                continue
            i0, x0, y0 = am.truth_at(d, s0)
            seg = d[i0:i1 + 1]
            moved = float(np.sum(np.hypot(np.diff(seg[:, 1]), np.diff(seg[:, 2])))) if len(seg) > 1 else 0.0
            # refused
            route = bfs_path(blocked, cell(x0, y0), cell(gx, gy))
            lab = None
            if route is None:
                lab = "refused"
            # stop circle (only meaningful where the collision monitor is active, v9)
            if lab is None and cfg == "nav2_params_v9":
                c_static = float(sdf_static(np.array([x1]), np.array([y1]), boxes, cyls)[0])
                c_person = min(math.hypot(x1 - d[i1, 4 + 2 * k], y1 - d[i1, 5 + 2 * k]) - 0.25 for k in range(3))
                ctl = [c for c in cmd if c["stage"] == "controller" and c["sim_t"] and s1 - 30 <= float(c["sim_t"]) <= s1
                       and (abs(float(c["vx"])) > 1e-3 or abs(float(c["wz"])) > 1e-3)]
                if min(c_static, c_person) < 0.23 and moved < 0.5 and ctl:
                    lab = "stop circle"
            # localization
            if lab is None:
                a = [x for x in amcl if x["sim_t"] and s0 <= float(x["sim_t"]) <= s1]
                if a:
                    yaw = math.radians(float(a[-1]["yaw_deg"]))
                    ax = float(a[-1]["x"]) + offset * math.cos(yaw)
                    ay = float(a[-1]["y"]) + offset * math.sin(yaw)
                    _, tx, ty = am.truth_at(d, float(a[-1]["sim_t"]))
                    if math.hypot(ax - tx, ay - ty) > 1.0:
                        lab = "localization"
            # deadband
            if lab is None and moved < 0.5:
                mon = [c for c in cmd if c["stage"] == "monitor" and c["sim_t"] and s1 - 30 <= float(c["sim_t"]) <= s1]
                slow = [c for c in mon if (abs(float(c["vx"])) > 1e-3 or abs(float(c["wz"])) > 1e-3)
                        and abs(float(c["vx"])) < 0.03 and abs(float(c["wz"])) < 0.15]
                if mon and len(slow) > 0.5 * len(mon):
                    lab = "deadband"
            # walker
            if lab is None and seg.shape[1] >= 10 and len(seg) > 2:
                ev = contacts.events(seg[:, 0], seg[:, 1], seg[:, 2],
                                     [(seg[:, 4 + 2 * k], seg[:, 5 + 2 * k]) for k in range(3)])
                near = min(math.hypot(x1 - d[i1, 4 + 2 * k], y1 - d[i1, 5 + 2 * k]) for k in range(3))
                if ev or near < 0.5:
                    lab = "walker"
            # phantom passage
            if lab is None and phantom is not None and route is not None:
                if any(phantom[y, x] for y, x in route):
                    lab = "phantom"
            if lab is None:
                if r["result"] == "ABORTED" and moved >= 3.0:
                    lab = "gave up"
                elif r["result"] == "TIMEOUT" and moved >= 3.0:
                    lab = "out of time"
                elif moved < 0.5:
                    lab = "stuck"
            at_door = min(math.hypot(x1 - dx, y1 - dy) for dx, dy in doors) < 1.0 if doors else False
            labels.append(dict(seed=seed, mission=r["mission"], attempt=r["attempt"], result=r["result"],
                               label=lab or "other", moved=round(moved, 2), at_door=bool(at_door)))
    return labels


def main():
    out = {}
    for arm in ARMS:
        labels = label_attempts(arm)
        if not labels:
            continue
        c = Counter(x["label"] for x in labels)
        out[arm] = dict(counts={k: c.get(k, 0) for k in ORDER}, attempts=labels)
        doors = sum(x["at_door"] for x in labels)
        print(f"{arm:10s} failed attempts {len(labels):3d} ({doors} ended at a door): " +
              ", ".join(f"{k} {c.get(k, 0)}" for k in ORDER if c.get(k, 0)))
    json.dump(out, open(os.path.join(am.RUNS, "missions_failures.json"), "w"), indent=1)
    print("results -> runs/missions_failures.json")


if __name__ == "__main__":
    main()
