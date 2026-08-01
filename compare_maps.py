#!/usr/bin/env python3
# Paired comparison of maps of the same world: walking vs frozen people (world 659927).
# Where are each map's phantom cells? (near person spawns? on top of the paths?)
import math
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; RED = "#e34948"; AQUA = "#1baf7a"

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
import importlib.util
spec = importlib.util.spec_from_file_location("em", "/home/user/isaac/evaluate_map.py")
em = importlib.util.module_from_spec(spec)
import sys
sys.argv = ["em", "x", "y"]
exec(open("/home/user/isaac/evaluate_map.py").read().split("def main")[0], em.__dict__)

MANIFEST = f"{RUNS}/world_659927.csv"
persons, paths = [], {}
for line in open(MANIFEST):
    p = line.strip().split(",")
    if p[0] == "person":
        persons.append((float(p[2]), float(p[3])))
    elif p[0] == "path":
        paths.setdefault(p[1], []).append((int(p[4]), float(p[2]), float(p[3])))


def load_aligned(tag):
    occ, free, res, origin = em.load_map(f"{RUNS}/world_659927{tag}.yaml")
    gt, _ = em.render_truth(MANIFEST, res, occ.shape, origin)
    gtd = em.dilate(gt, 1)
    best = (-1.0, 0, 0)
    k = int(0.4 / res)
    for dy in range(-k, k + 1, 2):
        for dx in range(-k, k + 1, 2):
            sc = float((np.roll(np.roll(occ, dy, 0), dx, 1) & gtd).sum())
            if sc > best[0]:
                best = (sc, dy, dx)
    _, dy, dx = best
    return (np.roll(np.roll(occ, dy, 0), dx, 1), np.roll(np.roll(free, dy, 0), dx, 1),
            gt, res, origin)


occ_w, free_w, gt, res, origin = load_aligned("")
occ_s, free_s, gt2, res2, origin2 = load_aligned("_static")
# common frame: same size? (origins differ slightly — I work in world coords per-map)
H, W = occ_w.shape


def world_mask(shape, res_, origin_, centers, r):
    Hh, Ww = shape
    jj, ii = np.meshgrid(np.arange(Ww), np.arange(Hh))
    wx = jj * res_ + origin_[0]
    wy = ii * res_ + origin_[1]
    m = np.zeros(shape, bool)
    for cx, cy in centers:
        m |= (wx - cx) ** 2 + (wy - cy) ** 2 < r * r
    return m


def path_mask(shape, res_, origin_, r):
    Hh, Ww = shape
    jj, ii = np.meshgrid(np.arange(Ww), np.arange(Hh))
    wx = jj * res_ + origin_[0]
    wy = ii * res_ + origin_[1]
    m = np.zeros(shape, bool)
    for name, wp in paths.items():
        wp = [q[1:] for q in sorted(wp)]
        for (x1, y1), (x2, y2) in zip(wp, wp[1:] + wp[:1]):
            L = math.hypot(x2 - x1, y2 - y1)
            for k in range(int(L / 0.1) + 1):
                cx = x1 + (x2 - x1) * k / max(1, int(L / 0.1))
                cy = y1 + (y2 - y1) * k / max(1, int(L / 0.1))
                m |= (wx - cx) ** 2 + (wy - cy) ** 2 < r * r
    return m


gt_d = em.dilate(gt, 2)
gt_d2 = em.dilate(gt2, 2)
cell_m2 = res * res

for tag, occ_, gtd_, res_, orig_ in (("walking", occ_w, gt_d, res, origin),
                                     ("static ", occ_s, gt_d2, res2, origin2)):
    ph = occ_ & ~gtd_
    spawn = world_mask(occ_.shape, res_, orig_, persons, 0.6)
    onpath = path_mask(occ_.shape, res_, orig_, 0.5)
    n_all = int(ph.sum())
    n_spawn = int((ph & spawn).sum())
    n_path = int((ph & onpath & ~spawn).sum())
    n_else = n_all - n_spawn - n_path
    print(f"{tag}: phantom {n_all} cells ({n_all*cell_m2:.2f} m^2) | "
          f"in spawn disks: {n_spawn} | on the paths: {n_path} | elsewhere: {n_else}")

# figure: 3 panels
fig = plt.figure(figsize=(13.8, 5.4), dpi=170)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(1, 3, wspace=0.06, left=0.02, right=0.98, top=0.80, bottom=0.03)
titles = ["A · walking people (~40 min)", "B · same world, people frozen (~40 min)",
          "C · phantom cells (not in ground truth)"]
for i, ax in enumerate([fig.add_subplot(gs[k]) for k in range(3)]):
    ax.set_facecolor(SURFACE)
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.set_title(titles[i], color=INK, fontsize=11, loc="left")
    if i < 2:
        occ_, free_ = (occ_w, free_w) if i == 0 else (occ_s, free_s)
        base = np.full(occ_.shape, 0.94)
        base[free_] = 1.0
        base[occ_] = 0.15
        ax.imshow(base, cmap="gray", origin="lower", vmin=0, vmax=1)
        yy, xx = np.where(gt if i == 0 else gt2)
        ax.scatter(xx, yy, s=0.05, c=RED, alpha=0.25)
    else:
        base = np.full(occ_w.shape, 1.0)
        ax.imshow(base, cmap="gray", origin="lower", vmin=0, vmax=1)
        yy, xx = np.where(gt)
        ax.scatter(xx, yy, s=0.05, c="#dddcd4", alpha=0.9, zorder=2)
        pw = occ_w & ~gt_d
        ps = occ_s & ~gt_d2
        yy, xx = np.where(ps)
        ax.scatter(xx, yy, s=1.2, c=BLUE, alpha=0.8, zorder=3, label="static run")
        yy, xx = np.where(pw)
        ax.scatter(xx, yy, s=1.2, c=ORANGE, alpha=0.8, zorder=4, label="walking run")
        for cx, cy in persons:
            j = (cx - origin[0]) / res
            i2 = (cy - origin[1]) / res
            c = plt.Circle((j, i2), 0.6 / res, fill=False, ec=INK, lw=1.0, ls=":", zorder=5)
            ax.add_patch(c)
        ax.legend(loc="lower right", fontsize=8.5, frameon=True,
                  facecolor=SURFACE, edgecolor=GRID, markerscale=6)

fig.suptitle("SLAM under motion — same world mapped twice: what do walking humans cost the map?",
             color=INK, fontsize=14, x=0.02, ha="left", y=0.975)
fig.text(0.02, 0.905,
         "world 659927 · v2.9.1 wanders, slam_toolbox maps, odometry noise 3%/5% · "
         "precision 85.6% vs 85.8% · dotted circles: person spawn spots",
         color=INK2, fontsize=10)
fig.savefig("/home/user/isaac/slam_walking_vs_static.png", facecolor=PAGE, bbox_inches="tight")
print("saved slam_walking_vs_static.png")
