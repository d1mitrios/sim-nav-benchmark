#!/usr/bin/env python3
# First Figure: run trajectory over the map of world 983120 (dense, 4 gates)
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import transforms

CSV = "/mnt/user-data/uploads/isaac_project/runs/run_983120_20260722_134822.csv"

# --- palette (dataviz reference, light mode) ---
SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BASE = "#c3c2b7"
ORANGE = "#eb6834"          # gates (identity)
AQUA = "#1baf7a"            # persons (identity)
SEQ = ["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"]  # sim time: light->dark
cmap = LinearSegmentedColormap.from_list("seqblue", SEQ)

# --- world 983120 manifest (from runs/world_983120.csv) ---
gates = [  # cx, cy, g, L, yaw, wall_a(x,y), wall_b(x,y)
    (-0.53, -5.00, 0.77, 1.65, 60.5, (-1.04, -4.71), (-0.02, -5.28)),
    (-7.07, -6.85, 0.68, 1.90, 172.2, (-7.14, -7.39), (-6.99, -6.32)),
    (6.34, -1.89, 0.88, 2.00, 13.7, (6.19, -1.27), (6.49, -2.51)),
    (-2.55, 6.84, 0.90, 1.93, 78.8, (-3.19, 6.96), (-1.91, 6.71)),
]
GATE_T = 0.40
boxes = [  # x, y, sx, sy, yaw
    (6.27, -8.27, 2.77, 0.88, 115.3), (-2.21, 3.09, 0.96, 1.73, 67.1),
    (3.12, -3.78, 1.06, 1.76, 39.0), (6.95, 8.15, 2.42, 1.00, 87.7),
    (-5.11, 0.28, 1.66, 0.76, 178.5), (-4.40, -4.49, 2.66, 1.14, 135.5),
    (-8.43, -3.34, 2.24, 1.51, 132.6), (-6.67, 8.24, 1.95, 0.89, 164.4),
    (-8.09, 2.97, 1.45, 0.64, 91.4), (-3.65, -8.15, 2.83, 0.74, 132.0),
    (7.48, 3.08, 2.99, 1.09, 71.4), (1.57, 7.23, 2.89, 0.59, 82.6),
]
cyls = [(2.77, 2.51, 1.29), (2.53, -7.08, 1.34), (-5.39, 4.79, 1.19),
        (-8.10, 5.26, 0.79), (-2.21, -1.97, 0.69)]
persons = [(4.46, 5.39), (8.33, -5.63), (-0.44, -7.82)]

# --- data ---
d = np.genfromtxt(CSV, delimiter=",", skip_header=2)
t, x, y = d[:, 0], d[:, 1], d[:, 2]
dur = t[-1]
seg = np.hypot(np.diff(x), np.diff(y))
path_len = float(seg.sum())
speed = seg / np.diff(t)
moving_frac = float((speed > 0.03).mean())
person_min = [float(np.min(np.hypot(x - px, y - py))) for px, py in persons]

def gate_crossed(cx, cy, g):
    return bool(np.min(np.hypot(x - cx, y - cy)) < g / 2 + 0.05)

# --- figure ---
fig, ax = plt.subplots(figsize=(11, 11.6), dpi=180)
fig.patch.set_facecolor(PAGE)
ax.set_facecolor(SURFACE)

def rot_rect(cx, cy, sx, sy, yaw, fc, ec, lw=0.8, z=2):
    r = Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=fc, edgecolor=ec, lw=lw, zorder=z)
    r.set_transform(transforms.Affine2D().rotate_deg_around(cx, cy, yaw) + ax.transData)
    ax.add_patch(r)

# boundary
for bx, by, sx, sy in [(0, 10, 20, 0.5), (0, -10, 20, 0.5), (10, 0, 0.5, 20), (-10, 0, 0.5, 20)]:
    rot_rect(bx, by, sx, sy, 0, BASE, BASE, z=2)
# obstacles (recessive)
for bx, by, sx, sy, yaw in boxes:
    rot_rect(bx, by, sx, sy, yaw, "#d8d7d0", BASE, z=2)
for cx_, cy_, r_ in cyls:
    ax.add_patch(Circle((cx_, cy_), r_, facecolor="#d8d7d0", edgecolor=BASE, lw=0.8, zorder=2))
# gates (orange walls + width label + crossed?)
for cx_, cy_, g, L, yaw, wa, wb in gates:
    for wx, wy in (wa, wb):
        rot_rect(wx, wy, L, GATE_T, yaw, ORANGE, "#c94e20", z=3)
    crossed = gate_crossed(cx_, cy_, g)
    tag = f"{g:.2f} m " + ("✓" if crossed else "—")
    ax.annotate(tag, (cx_, cy_), textcoords="offset points", xytext=(14, 12),
                fontsize=11, color=INK, fontweight="bold", zorder=6,
                bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE, ec=GRID, alpha=0.9))
# persons
for i, (px, py) in enumerate(persons):
    ax.plot(px, py, "o", ms=11, mfc=AQUA, mec=INK, mew=1.2, zorder=5)
    ax.annotate(f"person {i}", (px, py), textcoords="offset points", xytext=(10, -14),
                fontsize=9.5, color=INK2, zorder=6)
# trajectory: sequential ramp by sim time
pts = np.c_[x, y].reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap=cmap, array=t[:-1], linewidths=2.0, zorder=4, capstyle="round")
ax.add_collection(lc)
ax.plot(x[0], y[0], "o", ms=11, mfc="none", mec=INK, mew=1.6, zorder=6)
ax.annotate("start (0,0)", (x[0], y[0]), textcoords="offset points", xytext=(10, 10),
            fontsize=10, color=INK, zorder=6)
ax.plot(x[-1], y[-1], "o", ms=9, mfc=SEQ[-1], mec=INK, mew=1.0, zorder=6)
ax.annotate("end", (x[-1], y[-1]), textcoords="offset points", xytext=(10, -14),
            fontsize=10, color=INK, zorder=6)

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
ax.set_title("First instrumented run — world 983120 (dense, 4 gates), navigator v2.8",
             color=INK, fontsize=14, loc="left", y=1.045)
stats = (f"duration {dur/60:.1f} min (sim)  ·  path {path_len:.1f} m  ·  "
         f"moving {moving_frac*100:.0f}% of time  ·  min person clearance "
         f"{min(person_min):.2f} m  ·  samples {len(t)} @ 10 Hz")
ax.text(0, 1.014, stats, transform=ax.transAxes, color=INK2, fontsize=10.5, va="bottom")

fig.tight_layout()
out = "/home/user/isaac/trajectory_983120_run1.png"
fig.savefig(out, facecolor=PAGE, bbox_inches="tight")
print(out)
print(f"duration={dur:.1f}s path={path_len:.1f}m moving={moving_frac*100:.0f}% "
      f"person_min={['%.2f' % p for p in person_min]}")
for cx_, cy_, g, L, yaw, wa, wb in gates:
    print(f"gate g={g:.2f} crossed={gate_crossed(cx_, cy_, g)} "
          f"min_dist_to_center={np.min(np.hypot(x - cx_, y - cy_)):.2f}")
