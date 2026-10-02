#!/usr/bin/env python3
# Figure, experiment 2: dynamic run, robot + 3 walkers + close encounters (world 192691)
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
from matplotlib import transforms, gridspec

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BASE = "#c3c2b7"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; AQUA = "#1baf7a"; MAGENTA = "#e87ba4"; RED = "#e34948"
SEQ = ["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"]
cmap = LinearSegmentedColormap.from_list("seqblue", SEQ)
PCOL = [ORANGE, AQUA, MAGENTA]

CSV = RUNS_DIR + "/run_192691_20260724_124202.csv"
import manifest
_W = manifest.load("192691")
partitions, boxes, cyls = _W["partitions"], _W["boxes"], _W["cyls"]
doors = [(q["x"], q["y"], q["w"], q["name"].split("_")[1]) for q in _W["doors"]]

d = np.genfromtxt(CSV, delimiter=",", skip_header=2)
t, x, y = d[:, 0], d[:, 1], d[:, 2]
P = [(d[:, 4], d[:, 5]), (d[:, 6], d[:, 7]), (d[:, 8], d[:, 9])]
per = [np.hypot(x - px, y - py) for px, py in P]
mind = np.min(per, axis=0)
imin = int(np.argmin(mind))
robot_m = float(np.hypot(np.diff(x), np.diff(y)).sum())
human_km = [float(np.hypot(np.diff(px), np.diff(py)).sum()) / 1000 for px, py in P]


def zone(th):
    """Share of time below a distance threshold and the number of entries into that zone."""
    b = mind < th
    return b.mean() * 100, int((np.diff(b.astype(int)) == 1).sum() + b[0])


f12, n12 = zone(1.2)
f05, n05 = zone(0.5)

fig = plt.figure(figsize=(11, 14.6), dpi=170)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(2, 1, height_ratios=[3.4, 1.0], hspace=0.16)
ax = fig.add_subplot(gs[0])
ax2 = fig.add_subplot(gs[1])
for a in (ax, ax2):
    a.set_facecolor(SURFACE)
    a.grid(True, color=GRID, lw=0.7)
    a.tick_params(colors=MUTED, labelsize=9)
    for s in a.spines.values():
        s.set_color(GRID)

def rot_rect(a, cx, cy, sx, sy, yaw, fc, ec, z=2):
    r = Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=fc, edgecolor=ec, lw=0.8, zorder=z)
    r.set_transform(transforms.Affine2D().rotate_deg_around(cx, cy, yaw) + a.transData)
    a.add_patch(r)

for bx, by, sx, sy in [(0, 10, 20, 0.5), (0, -10, 20, 0.5), (10, 0, 0.5, 20), (-10, 0, 0.5, 20)]:
    rot_rect(ax, bx, by, sx, sy, 0, BASE, BASE)
for bx, by, sx, sy, yaw in boxes:
    rot_rect(ax, bx, by, sx, sy, yaw, "#d8d7d0", BASE)
for cx_, cy_, r_ in cyls:
    ax.add_patch(Circle((cx_, cy_), r_, facecolor="#d8d7d0", edgecolor=BASE, lw=0.8, zorder=2))
for px_, py_, sx, sy in partitions:
    rot_rect(ax, px_, py_, sx, sy, 0, "#a49bb0", "#7d7590", z=3)
for dx_, dy_, g, nm in doors:
    ax.annotate(f"{nm}: {g:.2f} m", (dx_, dy_), textcoords="offset points", xytext=(14, 10),
                fontsize=9.5, color=INK2, zorder=6,
                bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec=GRID, alpha=0.9))

# human trajectories (thin, identity colors)
for i, (px, py) in enumerate(P):
    ax.plot(px, py, color=PCOL[i], lw=1.1, alpha=0.65, zorder=3)
    ax.plot(px[-1], py[-1], "o", ms=10, mfc=PCOL[i], mec=INK, mew=1.1, zorder=6)
    ax.annotate(f"person {i}", (px[-1], py[-1]), textcoords="offset points", xytext=(10, -13),
                fontsize=9.5, color=INK2, zorder=6)

# robot trajectory (sequential)
pts = np.c_[x, y].reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap=cmap, array=t[:-1], linewidths=1.8, zorder=4, capstyle="round")
ax.add_collection(lc)
ax.plot(x[0], y[0], "o", ms=10, mfc="none", mec=INK, mew=1.5, zorder=6)
ax.annotate("robot start", (x[0], y[0]), textcoords="offset points", xytext=(10, 10), fontsize=9.5, color=INK, zorder=6)

# the closest encounter
ax.plot(x[imin], y[imin], "o", ms=15, mfc="none", mec=RED, mew=2.2, zorder=7)
ax.annotate(f"closest pass: {mind[imin]:.2f} m at t = {t[imin]:.0f} s", (x[imin], y[imin]),
            textcoords="offset points", xytext=(14, 14), fontsize=10, color=RED, fontweight="bold", zorder=7)

ax.set_xlim(-10.8, 10.8); ax.set_ylim(-10.8, 10.8)
ax.set_aspect("equal")
ax.set_title("Experiment 2: world 192691, robot among 3 walking people (v2.9.1, unmodified)",
             color=INK, fontsize=13, loc="left", y=1.035)
stats = (f"{t[-1]/60:.0f} sim-min · robot {robot_m:,.0f} m · people {min(human_km):.1f}-{max(human_km):.1f} km each · "
         f"distances centre to centre · <1.2 m: {f12:.1f}% of time ({n12} entries) · <0.5 m: {f05:.1f}% ({n05})")
ax.text(0, 1.008, stats, transform=ax.transAxes, color=INK2, fontsize=9.5, va="bottom")

# strip: min robot-human distance over time
ax2.axhspan(0, 0.5, color="#fdeaea", zorder=1)
ax2.axhspan(0.5, 1.2, color="#fdf3ea", zorder=1)
ax2.plot(t, mind, color=BLUE, lw=1.0, zorder=4)
ax2.axhline(1.2, color=MUTED, lw=0.9, ls=":")
ax2.axhline(0.5, color=MUTED, lw=0.9, ls="--")
ax2.text(t[-1] * 0.995, 1.24, "personal 1.2 m", color=MUTED, fontsize=8.5, ha="right", va="bottom")
ax2.text(t[-1] * 0.995, 0.54, "intimate 0.5 m", color=MUTED, fontsize=8.5, ha="right", va="bottom")
ax2.set_xlim(0, t[-1])
ax2.set_ylim(0, 6)
ax2.set_xlabel("sim time (s)", color=INK2, fontsize=10)
ax2.set_ylabel("distance to nearest\nperson (m)", color=INK2, fontsize=9.5)

fig.savefig(RUNS_DIR + "/dynamic_192691_social.png", facecolor=PAGE, bbox_inches="tight")
print("saved dynamic_192691_social.png")
