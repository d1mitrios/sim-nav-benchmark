#!/usr/bin/env python3
# Do the failures on SLAM maps come from paths through unexplored space? (experiments 7 and 5b)
# Usage: python3 unknown_paths.py [arm]   (arm from stats_missions.ARMS, default rev_slam)
# Nav2 is configured with track_unknown_space: false, so every cell the mapping run never saw is
# free for the planner, and the global costmap has no live obstacle layer. Hypothesis: a goal
# attempt fails when the planned route leads through unexplored space that is in truth blocked,
# because nothing ever marks that route as invalid. Test, per goal attempt: from the robot's true
# position at the start of the attempt, take the shortest route to the goal on the SLAM map as the
# planner sees it (unknown = free, obstacles grown by the planner radius, 4-connected breadth-first
# search). The attempt has a "phantom passage" when that route enters cells that are unknown on
# the map but blocked in truth (obstacles grown by the same radius). The script compares how often
# failed and successful attempts have one, and how far a failed attempt ended from the first
# phantom cell on its route. It also counts the goals that stay reachable when unknown cells are
# treated as blocked. The shortest route stands in for NavFn's cheapest route, which also weighs
# the inflation cost, so this is a test of the mechanism, not a replay of the planner.
import os
import sys
import math
import numpy as np
from collections import deque
from scipy.ndimage import binary_dilation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_missions2 as am
import evaluate_map as em
from stats_missions import ARMS


def disk(rc):
    yy, xx = np.ogrid[-rc:rc + 1, -rc:rc + 1]
    return (yy * yy + xx * xx) <= rc * rc


def bfs_path(blocked, start, goal):
    h, w = blocked.shape
    sy, sx = start
    gy, gx = goal
    if not (0 <= gy < h and 0 <= gx < w) or blocked[gy, gx]:
        return None
    # start inside grown obstacles: step to the nearest free cell, like the planner's tolerance
    if blocked[sy, sx]:
        ys, xs = np.where(~blocked[max(0, sy - 8):sy + 9, max(0, sx - 8):sx + 9])
        if not len(ys):
            return None
        k = np.argmin((ys - min(sy, 8)) ** 2 + (xs - min(sx, 8)) ** 2)
        sy, sx = max(0, sy - 8) + ys[k], max(0, sx - 8) + xs[k]
    prev = -np.ones(blocked.shape, np.int64)
    prev[sy, sx] = sy * w + sx
    q = deque([(sy, sx)])
    while q:
        y, x = q.popleft()
        if (y, x) == (gy, gx):
            break
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and prev[ny, nx] < 0 and not blocked[ny, nx]:
                prev[ny, nx] = y * w + x
                q.append((ny, nx))
    if prev[gy, gx] < 0:
        return None
    path, cur = [], gy * w + gx
    while True:
        y, x = divmod(cur, w)
        path.append((y, x))
        if cur == prev[y, x]:
            break
        cur = prev[y, x]
    return path[::-1]


def main():
    arm = sys.argv[1] if len(sys.argv) > 1 else "rev_slam"
    cfg, mk, d0, d1, md5 = ARMS[arm]
    files = am.pick_files(cfg, mk, d0, d1, set(am.BENCH_SEEDS), md5)
    radius = am.planner_radius(cfg, md5 or "")
    stats = {"fail": [0, 0], "succ": [0, 0]}
    end_dist, reach_known, n_goals = [], 0, 0
    for seed, f in sorted(files.items()):
        occ, free, res, origin = em.load_map(f"{am.RUNS}/world_{seed}.yaml")
        unknown = ~occ & ~free
        gt, _ = em.render_truth(f"{am.RUNS}/world_{seed}.csv", res, occ.shape, origin)
        rc = int(round(radius / res))
        map_blocked = binary_dilation(occ, structure=disk(rc))
        truth_blocked = binary_dilation(gt, structure=disk(rc))
        phantom = unknown & truth_blocked & ~map_blocked
        known_blocked = binary_dilation(occ | unknown, structure=disk(rc))
        rf = am.run_for(seed, am.ts_of(f))
        d = np.genfromtxt(rf, delimiter=",", skip_header=2)

        def cell(x, y):
            return int(round((y - origin[1]) / res)), int(round((x - origin[0]) / res))
        seen_goals = set()
        for r in am.read_missions(f):
            if not r["sim_start"] or not r["sim_end"]:
                continue
            gx, gy = float(r["goal_x"]), float(r["goal_y"])
            _, x0, y0 = am.truth_at(d, float(r["sim_start"]))
            _, x1, y1 = am.truth_at(d, float(r["sim_end"]))
            path = bfs_path(map_blocked, cell(x0, y0), cell(gx, gy))
            if path is None:
                continue
            ph = [i for i, (y, x) in enumerate(path) if phantom[y, x]]
            key = "succ" if r["result"] == "SUCCEEDED" else "fail"
            stats[key][0] += 1
            stats[key][1] += bool(ph)
            if key == "fail" and ph:
                py, px = path[ph[0]]
                wx, wy = origin[0] + px * res, origin[1] + py * res
                end_dist.append(math.hypot(x1 - wx, y1 - wy))
            if r["mission"] not in seen_goals:
                seen_goals.add(r["mission"])
                n_goals += 1
                reach_known += bfs_path(known_blocked, cell(0.0, 0.0), cell(gx, gy)) is not None
    f_n, f_ph = stats["fail"]
    s_n, s_ph = stats["succ"]
    print(f"{arm}: planner radius {radius:.2f} m, {len(files)} worlds")
    print(f"  failed attempts with a phantom passage on the planned route: {f_ph}/{f_n}"
          f" ({100 * f_ph / max(f_n, 1):.0f}%)")
    print(f"  successful attempts with one: {s_ph}/{s_n} ({100 * s_ph / max(s_n, 1):.0f}%)")
    if end_dist:
        e = np.array(end_dist)
        print(f"  failed attempts with a phantom passage ended a median {np.median(e):.2f} m from its first "
              f"cell ({(e < 1.0).sum()} of {len(e)} within 1 m)")
    print(f"  goals reachable from the spawn with unknown cells blocked: {reach_known}/{n_goals}")


if __name__ == "__main__":
    main()
