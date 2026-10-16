#!/usr/bin/env python3
# Early exploration: one run's trajectory over world 983120 (open layout, dense clutter, 4 gates)
import os
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import transforms

CSV = RUNS_DIR + "/run_983120_20260722_134822.csv"

# --- shared figure palette ---
SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BASE = "#c3c2b7"
ORANGE = "#eb6834"          # gates (identity)
AQUA = "#1baf7a"            # persons (identity)
SEQ = ["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"]  # sim time: light->dark
cmap = LinearSegmentedColormap.from_list("seqblue", SEQ)

# --- world 983120 geometry, read from its manifest ---
import manifest
_W = manifest.load("983120")
gates, boxes, cyls, persons = _W["gates"], _W["boxes"], _W["cyls"], _W["persons"]
GATE_T = 0.40

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
    tag = f"{g:.2f} m " + ("✓" if crossed else "-")
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
ax.set_title("First instrumented run - world 983120 (dense, 4 gates), navigator v2.8",
             color=INK, fontsize=14, loc="left", y=1.045)
stats = (f"duration {dur/60:.1f} min (sim)  ·  path {path_len:.1f} m  ·  "
         f"moving {moving_frac*100:.0f}% of time  ·  min person clearance "
         f"{min(person_min):.2f} m  ·  samples {len(t)} @ 10 Hz")
ax.text(0, 1.014, stats, transform=ax.transAxes, color=INK2, fontsize=10.5, va="bottom")

fig.tight_layout()
out = RUNS_DIR + "/trajectory_983120_run1.png"
fig.savefig(out, facecolor=PAGE, bbox_inches="tight")
print("saved " + os.path.basename(out))
print(f"duration={dur:.1f}s path={path_len:.1f}m moving={moving_frac*100:.0f}% "
      f"person_min={['%.2f' % p for p in person_min]}")
for cx_, cy_, g, L, yaw, wa, wb in gates:
    print(f"gate g={g:.2f} crossed={gate_crossed(cx_, cy_, g)} "
          f"min_dist_to_center={np.min(np.hypot(x - cx_, y - cy_)):.2f}")
