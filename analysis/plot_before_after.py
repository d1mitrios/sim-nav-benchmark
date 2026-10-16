#!/usr/bin/env python3
# Before/after: v1 vs v2.9.1 in the same world (seed 20260723013, which has the narrowest door crossed, 0.57 m)
import os
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import glob
import re
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import transforms, gridspec

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BASE = "#c3c2b7"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; RED = "#e34948"
SEQ = ["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"]
cmap = LinearSegmentedColormap.from_list("seqblue", SEQ)

RUNS = RUNS_DIR
SEED = "20260723013"

files = sorted(glob.glob(f"{RUNS}/run_{SEED}_*.csv"))
st = [f for f in files if re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) < "20260723_19"]
v1 = [f for f in files if "20260723_19" <= re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) < "20260724_02"]
f_v2, f_v1 = st[-1], v1[-1]

boxes, cyls, parts, doors, persons = [], [], [], [], []
for line in open(f"{RUNS}/world_{SEED}.csv"):
    p = line.strip().split(",")
    if p[0] == "box" and p[1].startswith("partition"):
        parts.append((float(p[2]), float(p[3]), float(p[4]), float(p[5])))
    elif p[0] == "box":
        boxes.append((float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6])))
    elif p[0] == "cyl":
        cyls.append((float(p[2]), float(p[3]), float(p[4])))
    elif p[0] == "door":
        doors.append((p[1], float(p[2]), float(p[3]), float(p[4]), p[5]))
    elif p[0] == "person":
        persons.append((float(p[2]), float(p[3])))

def rot_rect(a, cx, cy, sx, sy, yaw, fc, ec, z=2):
    r = Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=fc, edgecolor=ec, lw=0.8, zorder=z)
    r.set_transform(transforms.Affine2D().rotate_deg_around(cx, cy, yaw) + a.transData)
    a.add_patch(r)

def draw_world(a):
    for bx, by, sx, sy in [(0, 10, 20, 0.5), (0, -10, 20, 0.5), (10, 0, 0.5, 20), (-10, 0, 0.5, 20)]:
        rot_rect(a, bx, by, sx, sy, 0, BASE, BASE)
    for bx, by, sx, sy, yaw in boxes:
        rot_rect(a, bx, by, sx, sy, yaw, "#d8d7d0", BASE)
    for cx_, cy_, r_ in cyls:
        a.add_patch(Circle((cx_, cy_), r_, facecolor="#d8d7d0", edgecolor=BASE, lw=0.8, zorder=2))
    for px_, py_, sx, sy in parts:
        rot_rect(a, px_, py_, sx, sy, 0, "#a49bb0", "#7d7590", z=3)
    for hx, hy in persons:
        a.add_patch(Circle((hx, hy), 0.22, facecolor=MUTED, edgecolor=INK, lw=0.8, zorder=5))
    a.set_xlim(-10.8, 10.8); a.set_ylim(-10.8, 10.8)
    a.set_aspect("equal")

# crossings per door for the v2 run (for the labels)
d2 = np.genfromtxt(f_v2, delimiter=",", skip_header=2)
t2, x2, y2 = d2[:, 0], d2[:, 1], d2[:, 2]
pxw = next(dx for nm, dx, dy, g, ax_ in doors if ax_ == "x")
pyw = next(dy for nm, dx, dy, g, ax_ in doors if ax_ == "y")
cross = {nm: 0 for nm, *_ in doors}
for wall_axis, wall_pos in (("x", pxw), ("y", pyw)):
    coord = x2 if wall_axis == "x" else y2
    other = y2 if wall_axis == "x" else x2
    side = coord > wall_pos
    for i2 in np.where(side[:-1] != side[1:])[0]:
        oc = (other[i2] + other[i2 + 1]) / 2
        wd = [(nm, dx, dy) for nm, dx, dy, g, ax_ in doors if ax_ == wall_axis]
        nm = min(wd, key=lambda q: abs(oc - (q[2] if wall_axis == "x" else q[1])))[0]
        cross[nm] += 1

d1 = np.genfromtxt(f_v1, delimiter=",", skip_header=2)
t1, x1, y1 = d1[:, 0], d1[:, 1], d1[:, 2]
seg1 = np.hypot(np.diff(x1), np.diff(y1))
# v1 terminal-deadlock onset, same rule as compare_v1_v2.py (read from compare_rows.json)
_C = json.load(open(os.path.join(RUNS_DIR, "compare_rows.json")))
_row = next(r for r in _C if r["seed"] == SEED)
dl_i = int(np.searchsorted(t1, _row["term_t1"]))
d1_dist = float(seg1[:dl_i].sum())

fig = plt.figure(figsize=(13.4, 7.6), dpi=170)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(1, 2, wspace=0.10, left=0.045, right=0.975, top=0.815, bottom=0.055)
axL = fig.add_subplot(gs[0]); axR = fig.add_subplot(gs[1])
for a in (axL, axR):
    a.set_facecolor(SURFACE)
    a.grid(True, color=GRID, lw=0.7)
    a.tick_params(colors=MUTED, labelsize=8.5)
    for s in a.spines.values():
        s.set_color(GRID)
    draw_world(a)

_t1 = [r["term_t1"] for r in _C if r["term1"]]
_g = [g for nm, dx, dy, g, ax_ in doors]
_narrow = min(g for nm, dx, dy, g, ax_ in doors if cross[nm])
fig.suptitle("Same world, same follow-the-gap core: v1 vs v2.9.1",
             color=INK, fontsize=15.5, x=0.045, ha="left", y=0.965)
fig.text(0.045, 0.905,
         f"World {SEED} (four rooms, doors {min(_g):.2f}-{max(_g):.2f} m), identical spawn. "
         f"Across all {len(_C)} worlds: "
         f"v1 reached a terminal deadlock in {len(_t1)} (median onset {np.median(_t1):.0f} s), "
         f"v2.9.1 in {sum(r['term2'] for r in _C)}.",
         color=INK2, fontsize=10.5)
fig.text(0.045, 0.870,
         "v2.9.1 adds a recovery stack and graded person braking to the same core. Each "
         "recovery layer answers a failure observed in an earlier version.",
         color=INK2, fontsize=10.5, style="italic")

# ---- L: v1 ----
axL.plot(x1[:dl_i + 1], y1[:dl_i + 1], color=ORANGE, lw=2.2, zorder=4, solid_capstyle="round")
axL.plot(x1[0], y1[0], "o", ms=10, mfc="none", mec=INK, mew=1.5, zorder=6)
axL.annotate("spawn", (x1[0], y1[0]), textcoords="offset points", xytext=(10, 8),
             fontsize=9.5, color=INK, zorder=8)
axL.plot(x1[dl_i], y1[dl_i], "x", ms=16, mec=RED, mew=3.2, zorder=7)
axL.plot(x1[dl_i], y1[dl_i], "o", ms=17, mfc="none", mec=RED, mew=1.6, zorder=7)
axL.annotate(f"terminal deadlock from {t1[dl_i]:.0f} s\n{d1_dist:.1f} m travelled, then motionless here\n"
             f"for the remaining {(t1[-1]-t1[dl_i])/60:.0f} sim-min",
             (x1[dl_i], y1[dl_i]), xytext=(-10.0, -6.6), textcoords="data", ha="left",
             fontsize=10, color=RED, fontweight="bold", zorder=8,
             arrowprops=dict(arrowstyle="->", color=RED, lw=1.2,
                             connectionstyle="arc3,rad=0.15"))
axL.set_title(f"v1: bare follow-the-gap ({t1[-1] / 60:.0f} sim-min)", color=INK, fontsize=12.5, loc="left")
axL.text(0.03, 0.03, f"{len(_t1)}/{len(_C)} worlds: terminal deadlock", transform=axL.transAxes,
         color=RED, fontsize=10.5, fontweight="bold",
         bbox=dict(boxstyle="round,pad=0.35", fc=SURFACE, ec=GRID))

# ---- R: v2.9.1 ----
pts = np.c_[x2, y2].reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
axR.add_collection(LineCollection(segs, cmap=cmap, array=t2[:-1], linewidths=1.9,
                                  zorder=4, capstyle="round"))
axR.plot(x2[0], y2[0], "o", ms=10, mfc="none", mec=INK, mew=1.5, zorder=6)
for nm, dx, dy, g, ax_ in doors:
    c = cross[nm]
    txt = f"{g:.2f} m" + (f": crossed {c}×" if c else "")
    col = INK if c else MUTED
    w = "bold" if g < 0.6 and c else "normal"
    off = (-12, 10) if dx > 5.5 else (12, 10)
    axR.annotate(txt, (dx, dy), textcoords="offset points", xytext=off, fontsize=9,
                 ha="right" if dx > 5.5 else "left", color=col, fontweight=w, zorder=8,
                 bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec=GRID, alpha=0.9))
axR.set_title(f"v2.9.1: follow-the-gap + recovery stack ({t2[-1] / 60:.0f} sim-min)", color=INK,
              fontsize=12.5, loc="left")
axR.text(0.03, 0.03,
         f"{np.hypot(np.diff(x2), np.diff(y2)).sum():.0f} m · {sum(cross.values())} door crossings · "
         f"{'no terminal deadlock' if not _row['term2'] else 'terminal deadlock'} · "
         f"{_narrow:.2f} m door crossed",
         transform=axR.transAxes, color=INK, fontsize=10,
         bbox=dict(boxstyle="round,pad=0.35", fc=SURFACE, ec=GRID))

fig.savefig(RUNS_DIR + "/before_after_seed013.png", facecolor=PAGE, bbox_inches="tight")
print(f"v1: dl@{t1[dl_i]:.0f}s, {d1_dist:.1f}m | v2: {np.hypot(np.diff(x2), np.diff(y2)).sum():.0f}m, "
      f"crossings {cross}")
print("saved before_after_seed013.png")
