#!/usr/bin/env python3
# Offline feasibility PREDICTION for the mission batch (Step 2):
# for each seed of the mapping batch, computes the 4 tour goals (SAME logic as
# mission_runner v4: build_tour + adjust_goal) and checks reachability from the spawn
# with BFS on (a) the SLAM map (unknown=free, like track_unknown_space:false) and
# (b) ground truth, with inflation = planner robot_radius 0.30. Plus mapped door widths.
# Output: missions_feasibility.json — join with the overnight batch results
# (prediction-vs-practice per mission = validation of the whole tier-2 pipeline).
import json
import math
import numpy as np
from collections import deque
from scipy.ndimage import binary_dilation

em = {}
exec(open("/home/user/isaac/evaluate_map.py").read().split("def main")[0], em)

MAN_DIR = "/mnt/user-data/uploads/isaac_project/runs"
MAP_DIR = "/home/user/isaac/slam_batch"
R_PLAN = 0.30
SEEDS = [f"202607230{i:02d}" for i in range(1, 26)]


def load_walls(seed):
    pxw = pyw = None
    obs = []
    for line in open(f"{MAN_DIR}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "door" and p[5] == "x":
            pxw = float(p[2])
        elif p[0] == "door" and p[5] == "y":
            pyw = float(p[3])
        elif p[0] == "box" and not p[1].startswith("partition"):
            obs.append((float(p[2]), float(p[3]),
                        0.5 * math.hypot(float(p[4]), float(p[5]))))
        elif p[0] == "cyl":
            obs.append((float(p[2]), float(p[3]), float(p[4])))
    return pxw, pyw, obs


def adjust_goal(gx, gy, obs, pxw, pyw):
    sx, sy = gx > pxw, gy > pyw
    for r in [0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4, 2.7, 3.0]:
        for k in range(12):
            a = 2.0 * math.pi * k / 12.0
            x, y = gx + r * math.cos(a), gy + r * math.sin(a)
            if abs(x) > 8.6 or abs(y) > 8.6:
                continue
            if (x > pxw) != sx or (y > pyw) != sy:
                continue
            if abs(x - pxw) < 0.9 or abs(y - pyw) < 0.9:
                continue
            if all(math.hypot(x - ox, y - oy) - orr >= 0.6 for ox, oy, orr in obs):
                return x, y
    return gx, gy


def build_tour(pxw, pyw):
    xE, xW = (pxw + 10.0) / 2.0, (pxw - 10.0) / 2.0
    yN, yS = (pyw + 10.0) / 2.0, (pyw - 10.0) / 2.0
    ex = "W" if 0.0 < pxw else "E"
    ey = "S" if 0.0 < pyw else "N"
    other_x = "E" if ex == "W" else "W"
    other_y = "N" if ey == "S" else "S"
    cx = {"E": xE, "W": xW}
    cy = {"N": yN, "S": yS}
    return [("adjacent-x", cx[other_x], cy[ey]),
            ("adjacent-y", cx[ex], cy[other_y]),
            ("diagonal", cx[other_x], cy[other_y]),
            ("home", 0.0, 0.0)]


def disk(r_cells):
    yy, xx = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
    return (yy * yy + xx * xx) <= r_cells * r_cells


def reach_from(blocked, cx, cy):
    h, w = blocked.shape
    if blocked[cy, cx]:
        for r in range(1, 25):
            sub = ~blocked[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
            ys, xs = np.where(sub)
            if len(ys):
                cy, cx = max(0, cy - r) + ys[0], max(0, cx - r) + xs[0]
                break
    seen = np.zeros_like(blocked, bool)
    q = deque([(cy, cx)])
    seen[cy, cx] = True
    while q:
        y, x = q.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and not seen[ny, nx] and not blocked[ny, nx]:
                seen[ny, nx] = True
                q.append((ny, nx))
    return seen


def door_mapped_width(occ, res, origin, dx_, dy_, axis):
    def w2p(x, y):
        return int(round((x - origin[0]) / res)), int(round((y - origin[1]) / res))
    best = gap = 0
    for t in np.arange(-2.0, 2.0, res):
        wx, wy = (dx_, dy_ + t) if axis == "x" else (dx_ + t, dy_)
        hit = False
        for s in np.arange(-0.30, 0.31, res):
            qx, qy = (wx + s, wy) if axis == "x" else (wx, wy + s)
            px, py = w2p(qx, qy)
            if 0 <= py < occ.shape[0] and 0 <= px < occ.shape[1] and occ[py, px]:
                hit = True
                break
        if not hit:
            gap += 1
            best = max(best, gap)
        else:
            gap = 0
    return best * res


out = []
rc = int(round(R_PLAN / 0.05))
for seed in SEEDS:
    occ, free, res, origin = em["load_map"](f"{MAP_DIR}/world_{seed}.yaml")
    gt, _ = em["render_truth"](f"{MAN_DIR}/world_{seed}.csv", res, occ.shape, origin)
    pxw, pyw, obs = load_walls(seed)
    tour = [(nm, *adjust_goal(gx, gy, obs, pxw, pyw))
            for nm, gx, gy in build_tour(pxw, pyw)]
    st = disk(rc)
    mb = binary_dilation(occ, structure=st)
    tb = binary_dilation(gt, structure=st)

    def w2p(x, y):
        return int(round((x - origin[0]) / res)), int(round((y - origin[1]) / res))
    sx, sy = w2p(0.0, 0.0)
    rm = reach_from(mb, sx, sy)
    rt = reach_from(tb, sx, sy)
    goals = {}
    for nm, gx, gy in tour:
        px, py = w2p(gx, gy)
        goals[nm] = {"x": round(gx, 2), "y": round(gy, 2),
                     "map_ok": bool(rm[py, px]), "truth_ok": bool(rt[py, px])}
    doors = []
    for line in open(f"{MAN_DIR}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "door":
            wmap = door_mapped_width(occ, res, origin, float(p[2]), float(p[3]), p[5])
            doors.append({"name": p[1], "true_w": float(p[4]),
                          "mapped_w": round(wmap, 2)})
    out.append({"seed": seed, "pxw": pxw, "pyw": pyw, "goals": goals, "doors": doors})
    nm_ok = sum(g["map_ok"] for g in goals.values())
    nt_ok = sum(g["truth_ok"] for g in goals.values())
    print(f"{seed}: map {nm_ok}/4  truth {nt_ok}/4   " +
          " ".join(f"{d['name']}={d['true_w']:.2f}->{d['mapped_w']:.2f}" for d in doors))

json.dump(out, open("/home/user/isaac/missions_feasibility.json", "w"), indent=1)
m_tot = sum(sum(g["map_ok"] for g in w["goals"].values()) for w in out)
t_tot = sum(sum(g["truth_ok"] for g in w["goals"].values()) for w in out)
xg = [g for w in out for nm, g in w["goals"].items() if nm != "home"]
print(f"\nTOTAL: map-feasible {m_tot}/100 goals, truth-feasible {t_tot}/100")
print(f"cross-room goals (without home): map {sum(g['map_ok'] for g in xg)}/75, "
      f"truth {sum(g['truth_ok'] for g in xg)}/75")
dd = [(d["true_w"], d["mapped_w"]) for w in out for d in w["doors"]]
narrow = [tw - mw for tw, mw in dd]
print(f"doors: n={len(dd)}, mapped narrowing median {np.median(narrow):.2f} m, "
      f"sealed for r=0.30: true {sum(1 for t, _ in dd if t < 0.60)}, "
      f"mapped {sum(1 for _, m in dd if m < 0.60)}")
