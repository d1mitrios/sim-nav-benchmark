#!/usr/bin/env python3
# Figure 2: rooms mode — path + partition + doors + crossing points (world 759639)
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import transforms

CSV = "/mnt/user-data/uploads/isaac_project/runs/run_759639_20260722_145906.csv"

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BASE = "#c3c2b7"
ORANGE = "#eb6834"; AQUA = "#1baf7a"
SEQ = ["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"]
cmap = LinearSegmentedColormap.from_list("seqblue", SEQ)

PY = -3.70
partitions = [(-6.98, -3.70, 6.05, 0.40), (-0.24, -3.70, 5.71, 0.40), (6.73, -3.70, 6.54, 0.40)]
doors = [(-3.52, -3.70, 0.86), (3.04, -3.70, 0.85)]
boxes = [(-6.51, 1.20, 1.46, 1.98, 12.6), (-8.07, 6.04, 1.67, 1.05, 179.1),
         (4.97, 5.66, 2.03, 0.72, 88.5), (4.85, -6.28, 1.30, 1.64, 13.1),
         (-0.35, 4.27, 3.00, 1.98, 84.7), (1.92, -7.89, 2.87, 0.53, 135.9),
         (7.89, 7.79, 2.26, 1.46, 30.3), (6.14, -0.30, 2.78, 1.42, 56.7),
         (3.29, 1.78, 2.21, 0.61, 154.2), (2.71, 7.64, 1.36, 1.25, 43.8),
         (-5.78, 8.33, 0.88, 1.99, 138.7)]
cyls = [(-6.83, -7.74, 0.58), (-3.64, 5.15, 0.81), (-5.40, -6.27, 0.45),
        (8.32, -8.02, 1.20), (-3.55, -0.23, 1.15)]
persons = [(-0.71, 8.14), (8.31, 3.94), (-3.13, -6.91)]

d = np.genfromtxt(CSV, delimiter=",", skip_header=2)
t, x, y = d[:, 0], d[:, 1], d[:, 2]
dur = t[-1]
path_len = float(np.hypot(np.diff(x), np.diff(y)).sum())
side = y > PY
cross = np.where(side[:-1] != side[1:])[0]

fig, ax = plt.subplots(figsize=(11, 11.6), dpi=180)
fig.patch.set_facecolor(PAGE)
ax.set_facecolor(SURFACE)

def rot_rect(cx, cy, sx, sy, yaw, fc, ec, lw=0.8, z=2):
    r = Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=fc, edgecolor=ec, lw=lw, zorder=z)
    r.set_transform(transforms.Affine2D().rotate_deg_around(cx, cy, yaw) + ax.transData)
    ax.add_patch(r)

for bx, by, sx, sy in [(0, 10, 20, 0.5), (0, -10, 20, 0.5), (10, 0, 0.5, 20), (-10, 0, 0.5, 20)]:
    rot_rect(bx, by, sx, sy, 0, BASE, BASE)
for bx, by, sx, sy, yaw in boxes:
    rot_rect(bx, by, sx, sy, yaw, "#d8d7d0", BASE)
for cx_, cy_, r_ in cyls:
    ax.add_patch(Circle((cx_, cy_), r_, facecolor="#d8d7d0", edgecolor=BASE, lw=0.8, zorder=2))
# partition in darker purple-grey (structural element, not clutter)
for px_, py_, sx, sy in partitions:
    rot_rect(px_, py_, sx, sy, 0, "#a49bb0", "#7d7590", z=3)
for i, (dx_, dy_, g) in enumerate(doors):
    ax.annotate(f"door_{i}: {g:.2f} m", (dx_, dy_), textcoords="offset points",
                xytext=(0, -22 if i == 0 else 14), ha="center", fontsize=11, color=INK,
                fontweight="bold", zorder=6,
                bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE, ec=GRID, alpha=0.9))
for i, (px, py) in enumerate(persons):
    ax.plot(px, py, "o", ms=11, mfc=AQUA, mec=INK, mew=1.2, zorder=5)
    ax.annotate(f"person {i}", (px, py), textcoords="offset points", xytext=(10, -14),
                fontsize=9.5, color=INK2, zorder=6)

pts = np.c_[x, y].reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap=cmap, array=t[:-1], linewidths=2.0, zorder=4, capstyle="round")
ax.add_collection(lc)
ax.plot(x[0], y[0], "o", ms=11, mfc="none", mec=INK, mew=1.6, zorder=6)
ax.annotate("start (0,0)", (x[0], y[0]), textcoords="offset points", xytext=(10, 10), fontsize=10, color=INK, zorder=6)
ax.plot(x[-1], y[-1], "o", ms=9, mfc=SEQ[-1], mec=INK, mew=1.0, zorder=6)
ax.annotate("end", (x[-1], y[-1]), textcoords="offset points", xytext=(10, -14), fontsize=10, color=INK, zorder=6)
# crossing points: red ring + time label
for i in cross:
    cxp, cyp = (x[i] + x[i + 1]) / 2, (y[i] + y[i + 1]) / 2
    ax.plot(cxp, cyp, "o", ms=16, mfc="none", mec="#e34948", mew=2.4, zorder=7)
    ax.annotate(f"crossing @ t={t[i]:.0f}s", (cxp, cyp), textcoords="offset points",
                xytext=(16, 16), fontsize=10.5, color="#e34948", fontweight="bold", zorder=7)

cb = fig.colorbar(lc, ax=ax, fraction=0.037, pad=0.02)
cb.set_label("sim time (s)", color=MUTED, fontsize=10)
cb.ax.tick_params(colors=MUTED, labelsize=9)
cb.outline.set_edgecolor(GRID)

ax.set_xlim(-10.8, 10.8); ax.set_ylim(-10.8, 10.8)
ax.set_aspect("equal")
ax.grid(True, color=GRID, lw=0.7)
ax.tick_params(colors=MUTED, labelsize=9)
for s in ax.spines.values():
    s.set_color(GRID)
ax.set_title("Rooms mode — world 759639: two rooms, doors are the only passages (v2.8)",
             color=INK, fontsize=14, loc="left", y=1.045)
north = side.mean()
stats = (f"duration {dur/60:.1f} min (sim, snapshot)  ·  path {path_len:.1f} m  ·  "
         f"crossings {len(cross)} (door_1, 0.08 m off-center)  ·  "
         f"room time N {north*100:.0f}% / S {(1-north)*100:.0f}%")
ax.text(0, 1.014, stats, transform=ax.transAxes, color=INK2, fontsize=10.5, va="bottom")

fig.tight_layout()
out = "/home/user/isaac/trajectory_759639_rooms.png"
fig.savefig(out, facecolor=PAGE, bbox_inches="tight")
print(out)
