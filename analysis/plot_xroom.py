#!/usr/bin/env python3
# Experiment 2 with a cross-room walker: contested doorways, three conditions, episode anatomy, the closest pass
import os
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import glob
import json
import re
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
GRID = "#e1e0d9"; BASE = "#b9b8ae"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; AQUA = "#1baf7a"; RED = "#e34948"; MAGENTA = "#e87ba4"
SEQ = ["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"]
cmap = LinearSegmentedColormap.from_list("seqblue", SEQ)

RUNS = RUNS_DIR
rows = json.load(open(RUNS_DIR + "/xroom_rows.json"))
CONDS = ("S", "D2", "D3")
mpm = [float(np.median([r[c]["dist"] / r[c]["dur"] * 60 for r in rows])) for c in CONDS]
crh = [sum(r[c]["cross"] for r in rows) / sum(r[c]["dur"] for r in rows) * 3600 for c in CONDS]
nc = [sum(r[c]["near_contact"] for r in rows) for c in CONDS]
ct = [sum(r[c]["contacts"] for r in rows) for c in CONDS]          # walker pushed the robot
ct_r = [sum(r[c]["contacts"] > 0 for r in rows) for c in CONDS]
dead = [sum(bool(r[c]["terminal"]) for r in rows) for c in CONDS]
ep2, ep3 = sum(r["C2n"] for r in rows), sum(r["C3"]["n_ep"] for r in rows)
x2, x3 = sum(r["C2x"] for r in rows), sum(r["C3"]["x_cross"] for r in rows)
n_cross = sum(e["crossed"] for r in rows for e in r["eps"])
n_wait = sum(r["C3"]["ep_wait"] for r in rows)
near = min(((e["min_h"], e["dur"], r["seed"]) for r in rows for e in r["eps"]), key=lambda q: q[0])
SEED = near[2]
xd = next(line.strip().split(",") for line in open(f"{RUNS_DIR}/world_{SEED}.csv") if line.startswith("xdoor"))
pi, dx, dy, g, door_side = int(xd[1].split("_")[1]), float(xd[2]), float(xd[3]), float(xd[4]), xd[5]

fig = plt.figure(figsize=(13.6, 6.9), dpi=170)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(1, 3, width_ratios=[1.0, 1.05, 1.0], wspace=0.26,
                       left=0.055, right=0.975, top=0.795, bottom=0.09)
axA = fig.add_subplot(gs[0]); axB = fig.add_subplot(gs[1]); axC = fig.add_subplot(gs[2])
for a in (axA, axB, axC):
    a.set_facecolor(SURFACE)
    a.grid(True, color=GRID, lw=0.7)
    a.tick_params(colors=MUTED, labelsize=8.5)
    for s in a.spines.values():
        s.set_color(GRID)

fig.suptitle("Experiment 2, cross-room walkers: contested doorways, same 25 worlds, v2.9.1 unmodified",
             color=INK, fontsize=14.5, x=0.055, ha="left", y=0.965)
fig.text(0.055, 0.905,
         f"Door-zone encounters {ep2} → {ep3} episodes · robot kept using the contested doors "
         f"({x2} vs {x3} crossings) · {sum(dead)} deadlocks in the 75 runs of the three conditions",
         color=INK2, fontsize=10)

# ---- A: 3 conditions ----
conds = ["static", "dyn2", "dyn3"]
colors = [BASE, BLUE, ORANGE]
xx = np.arange(3)
axA.bar(xx - 0.27, mpm, width=0.24, color=colors, zorder=3, edgecolor=SURFACE)
axA.bar(xx, crh, width=0.24, color=colors, zorder=3, alpha=0.65, edgecolor=SURFACE)
axA.bar(xx + 0.27, ct, width=0.24, color=RED, zorder=3, alpha=0.25, edgecolor=SURFACE)
for i in range(3):
    axA.text(xx[i] - 0.27, mpm[i] + 0.3, f"{mpm[i]:.1f}", ha="center", fontsize=8.5, color=INK)
    axA.text(xx[i], crh[i] + 0.3, f"{crh[i]:.1f}", ha="center", fontsize=8.5, color=INK)
    axA.text(xx[i] + 0.27, ct[i] + 0.3, str(ct[i]), ha="center", fontsize=8.5,
             color=RED if ct[i] else MUTED)
axA.set_xticks(xx, ["static", "walking\n(same-room)", "walking\n+ cross-room"], fontsize=9)
axA.set_ylim(0, max(mpm + crh + ct) * 1.2)
axA.set_title("A · Three conditions, deadlocks " + " / ".join(str(v) for v in dead), color=INK, fontsize=11.5, loc="left")
axA.text(0.03, 0.965, "bars: speed (m/min) · door crossings (/h) · walker pushes",
         transform=axA.transAxes, fontsize=8.2, color=INK2, va="top")

# ---- B: episode anatomy ----
axB.axhspan(0.0, 0.35, color="#fdeaea", zorder=1)
axB.axhspan(0.35, 0.5, color="#fdf3ea", zorder=1)
for r in rows:
    for e in r["eps"]:
        col = AQUA if e["crossed"] else BLUE
        ms = 9 if e["min_h"] < 0.35 else 6.5
        axB.plot(e["dur"], e["min_h"], "o", ms=ms, mfc=RED if e["min_h"] < 0.35 else col,
                 mec=SURFACE, mew=0.9, alpha=0.9, zorder=4)
axB.annotate(f"…{SEED[-3:]}: {near[0]:.2f} m", (near[1], near[0]), textcoords="offset points", xytext=(8, 14),
             fontsize=9.5, color=RED, fontweight="bold",
             arrowprops=dict(arrowstyle="->", color=RED, lw=1.0))
axB.axhline(0.5, color=MUTED, lw=0.9, ls="--")
xmaxB = np.ceil(max(e["dur"] for r in rows for e in r["eps"]) * 1.15)
axB.text(xmaxB * 0.97, 0.51, "intimate 0.5 m", color=MUTED, fontsize=8, ha="right", va="bottom")
axB.set_xlim(0, xmaxB); axB.set_ylim(0.15, 1.45)
axB.set_xlabel("episode duration (s)", color=INK2, fontsize=9.5)
axB.set_ylabel("min robot-human distance in episode (m)", color=INK2, fontsize=9.5)
axB.set_title(f"B · {ep3} door-zone encounters", color=INK, fontsize=11.5, loc="left")
axB.plot([], [], "o", ms=6.5, mfc=AQUA, mec=SURFACE, label=f"robot crossed during/after ({n_cross})")
axB.plot([], [], "o", ms=6.5, mfc=BLUE, mec=SURFACE, label="robot stayed its side")
axB.plot([], [], "o", ms=9, mfc=RED, mec=SURFACE, label="near-contact (<0.35 m)")
axB.legend(loc="upper right", fontsize=8, frameon=True, facecolor=SURFACE, edgecolor=GRID)

# ---- C: zoom on the closest pass ----
f3 = [f for f in glob.glob(f"{RUNS}/run_{SEED}_20260726_*.csv")
      if re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) >= "20260726_14"]
d = np.genfromtxt(sorted(f3)[-1], delimiter=",", skip_header=2)
t, x, y = d[:, 0], d[:, 1], d[:, 2]
hx, hy = d[:, 4 + 2 * pi], d[:, 5 + 2 * pi]
hum = np.hypot(x - hx, y - hy)
zone = (np.hypot(x - dx, y - dy) < 2.5)
imin = int(np.argmin(np.where(zone, hum, 9.9)))
w0, w1 = np.searchsorted(t, t[imin] - 18), np.searchsorted(t, t[imin] + 18)

boxes, cyls, parts = [], [], []
for line in open(f"{RUNS}/world_{SEED}.csv"):
    p = line.strip().split(",")
    if p[0] == "box" and p[1].startswith("partition"):
        parts.append((float(p[2]), float(p[3]), float(p[4]), float(p[5])))
    elif p[0] == "box":
        boxes.append((float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6])))
    elif p[0] == "cyl":
        cyls.append((float(p[2]), float(p[3]), float(p[4])))

def rot_rect(a, cx, cy, sx, sy, yaw, fc, ec, z=2):
    r = Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=fc, edgecolor=ec, lw=0.8, zorder=z)
    r.set_transform(transforms.Affine2D().rotate_deg_around(cx, cy, yaw) + a.transData)
    a.add_patch(r)

for bx, by, sx, sy, yaw in boxes:
    rot_rect(axC, bx, by, sx, sy, yaw, "#d8d7d0", BASE)
for cx_, cy_, r_ in cyls:
    axC.add_patch(Circle((cx_, cy_), r_, facecolor="#d8d7d0", edgecolor=BASE, lw=0.8, zorder=2))
for px_, py_, sx, sy in parts:
    rot_rect(axC, px_, py_, sx, sy, 0, "#a49bb0", "#7d7590", z=3)

pts = np.c_[x[w0:w1], y[w0:w1]].reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
axC.add_collection(LineCollection(segs, cmap=cmap, array=t[w0:w1 - 1], linewidths=2.4,
                                  zorder=4, capstyle="round"))
axC.plot(hx[w0:w1], hy[w0:w1], color=MAGENTA, lw=2.0, alpha=0.85, zorder=4)
axC.plot(hx[w1 - 1], hy[w1 - 1], "o", ms=9, mfc=MAGENTA, mec=INK, mew=1.0, zorder=6)
axC.plot(x[imin], y[imin], "o", ms=15, mfc="none", mec=RED, mew=2.4, zorder=7)
axC.plot([x[imin], hx[imin]], [y[imin], hy[imin]], color=RED, lw=1.4, ls="-", zorder=6)
axC.annotate(f"{hum[imin]:.2f} m\nat t = {t[imin]:.0f} s", ((x[imin] + hx[imin]) / 2, (y[imin] + hy[imin]) / 2),
             textcoords="offset points", xytext=(12, 4), fontsize=10, color=RED,
             fontweight="bold", zorder=8)
axC.annotate(f"door {door_side}: {g:.2f} m", (dx, dy), textcoords="offset points", xytext=(-8, -18),
             fontsize=9, color=INK2, ha="right", zorder=8,
             bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec=GRID, alpha=0.9))
axC.annotate(f"walker at t = {t[w1 - 1]:.0f} s", (hx[w1 - 1], hy[w1 - 1]), textcoords="offset points", xytext=(-8, 8),
             ha="right", fontsize=9, color=MAGENTA, zorder=8)
axC.set_xlim(dx - 4.2, dx + 4.2); axC.set_ylim(dy - 4.2, dy + 4.2)
axC.set_aspect("equal")
axC.set_title(f"C · World …{SEED[-3:]}: the walker pushes the robot (±18 s)", color=INK, fontsize=11.5, loc="left")

# the closest pass of the 75 runs: where the walker came from, and how far the robot was pushed
yaw = d[:, 3]
kb = max(0, imin - 10)    # one second before the closest pass
bear = (np.degrees(np.arctan2(hy[kb] - y[kb], hx[kb] - x[kb])) - yaw[kb] + 180) % 360 - 180
jump = float(np.hypot(x[imin:imin + 21] - x[imin], y[imin:imin + 21] - y[imin]).max())
fig.text(0.055, 0.845,
         f"Closest pass of the 75 runs: {hum[imin]:.2f} m centre to centre in world …{SEED[-3:]}. The cross-room walker, "
         f"which follows its route without yielding,\ncame up behind the robot ({abs(bear):.0f}° off its "
         f"heading a second earlier) at the {g:.2f} m door and pushed it {jump:.1f} m.",
         color=INK2, fontsize=10, style="italic")

fig.text(0.055, 0.018,
         "Episode: robot and cross-room walker both within 1.5 m of the walker's door · "
         f"{n_wait} episodes with the robot halted ≥3 s · contested-door crossings {x2} vs {x3} · "
         f"walkers pushed the robot in {ct_r[1]} (same-room) and {ct_r[2]} (cross-room) of 25 runs.",
         color=MUTED, fontsize=8.3)
fig.savefig(RUNS_DIR + "/phase2b_contested_doorways.png", facecolor=PAGE, bbox_inches="tight")
print(f"seed {SEED[-3:]} min centre distance: {hum[imin]:.3f} m at t={t[imin]:.1f}s "
      f"(robot at {x[imin]:.2f},{y[imin]:.2f})")
print("saved phase2b_contested_doorways.png")
