#!/usr/bin/env python3
# Figure: The dynamic batch in one image — speed parity, social clearance,
# and the forensics of the watchdog's 16 pseudo-kills. Style: dataviz palette (same as Phase 1).
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
GRID = "#e1e0d9"; BASE = "#c3c2b7"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; AQUA = "#1baf7a"; MAGENTA = "#e87ba4"; RED = "#e34948"
SEQ = ["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"]
cmap = LinearSegmentedColormap.from_list("seqblue", SEQ)
PCOL = [ORANGE, AQUA, MAGENTA]

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
rows = json.load(open("/home/user/isaac/dynamic_rows.json"))
killed = [r for r in rows if r["log"] == "terminal_deadlock"]
full = [r for r in rows if r["log"] == "full"]

dyn_files = {re.search(r"run_(\d+)_", f).group(1): f
             for f in glob.glob(f"{RUNS}/run_202607230*.csv")
             if re.search(r"_(\d{8}_\d{6})\.csv", f).group(1) >= "20260724_15"}

fig = plt.figure(figsize=(12.8, 12.6), dpi=170)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(2, 2, height_ratios=[1.0, 1.15], hspace=0.30, wspace=0.22,
                       left=0.07, right=0.965, top=0.885, bottom=0.06)
axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])
axC = fig.add_subplot(gs[1, 0])
axD = fig.add_subplot(gs[1, 1])
for a in (axA, axB, axC, axD):
    a.set_facecolor(SURFACE)
    a.grid(True, color=GRID, lw=0.7)
    a.tick_params(colors=MUTED, labelsize=9)
    for s in a.spines.values():
        s.set_color(GRID)

fig.suptitle("Dynamic batch — 25 paired worlds: v2.9.1 unmodified among walking humans",
             color=INK, fontsize=15, x=0.07, ha="left", y=0.975)
fig.text(0.07, 0.945,
         "Same seeds & geometry as the static batch (md5-verified) · 3.17 h sim · 2 597 m · "
         "0 true deadlocks · dance episodes 2 vs 2 (unchanged) · 0 near-contacts <0.35 m",
         color=INK2, fontsize=10.5)
fig.text(0.07, 0.925,
         "The batch watchdog reported 16/25 \"terminal deadlocks\" — offline excursion analysis "
         "shows every one was a healthy, cruising run (two-point-net false alarms; runner fixed in v3).",
         color=INK2, fontsize=10.5, style="italic")

# ---------- A: locomotion parity ----------
for r in rows:
    sx = r["S"]["dist"] / r["S"]["dur"] * 60
    dy = r["D"]["dist"] / r["D"]["dur"] * 60
    k = r["log"] == "terminal_deadlock"
    axA.plot(sx, dy, "o", ms=8, mfc=ORANGE if k else BLUE, mec=SURFACE, mew=1.0,
             alpha=0.9, zorder=4)
lo, hi = 11.8, 16.4
axA.plot([lo, hi], [lo, hi], color=MUTED, lw=1.0, ls="--", zorder=2)
axA.text(hi - 0.15, hi - 0.5, "y = x", color=MUTED, fontsize=9, ha="right")
axA.set_xlim(lo, hi); axA.set_ylim(lo, hi)
axA.set_aspect("equal")
axA.set_xlabel("static — speed made good (m/min)", color=INK2, fontsize=10)
axA.set_ylabel("dynamic — speed made good (m/min)", color=INK2, fontsize=10)
axA.set_title("A · Locomotion: no cost of dynamics", color=INK, fontsize=12, loc="left")
sm = np.median([r["S"]["dist"] / r["S"]["dur"] * 60 for r in rows])
dm = np.median([r["D"]["dist"] / r["D"]["dur"] * 60 for r in rows])
axA.text(0.04, 0.96, f"median {sm:.1f} → {dm:.1f} m/min", transform=axA.transAxes,
         color=INK, fontsize=10, va="top",
         bbox=dict(boxstyle="round,pad=0.3", fc=SURFACE, ec=GRID))
axA.plot([], [], "o", ms=8, mfc=BLUE, mec=SURFACE, label="run completed (9)")
axA.plot([], [], "o", ms=8, mfc=ORANGE, mec=SURFACE, label="killed by watchdog (16)")
axA.legend(loc="lower right", fontsize=9, frameon=True, facecolor=SURFACE, edgecolor=GRID)

# ---------- B: social clearance ----------
axB.axhspan(0.0, 0.35, color="#fdeaea", zorder=1)
axB.axhspan(0.35, 0.5, color="#fdf3ea", zorder=1)
axB.axhline(0.5, color=MUTED, lw=0.9, ls="--")
axB.axhline(1.2, color=MUTED, lw=0.9, ls=":")
for i, r in enumerate(rows):
    mc = r["D"]["min_clear"]
    k = r["log"] == "terminal_deadlock"
    axB.plot(i + 1, min(mc, 1.55), "o", ms=8, mfc=ORANGE if k else BLUE,
             mec=SURFACE, mew=1.0, zorder=4)
    if mc > 1.6:
        axB.annotate(f"{mc:.1f} m (no encounter)", (i + 1, 1.55), textcoords="offset points",
                     xytext=(6, 6), fontsize=8.5, color=MUTED)
axB.text(24.8, 1.22, "personal 1.2 m", color=MUTED, fontsize=8.5, ha="right", va="bottom")
axB.text(24.8, 0.52, "intimate 0.5 m", color=MUTED, fontsize=8.5, ha="right", va="bottom")
axB.text(24.8, 0.02, "near-contact <0.35 m: never entered", color=RED, fontsize=8.5,
         ha="right", va="bottom")
axB.set_xlim(0, 26); axB.set_ylim(0, 1.65)
axB.set_xlabel("world (seed 20260723001…025)", color=INK2, fontsize=10)
axB.set_ylabel("min robot-human clearance (m)", color=INK2, fontsize=10)
axB.set_title("B · Social clearance per run", color=INK, fontsize=12, loc="left")
axB.text(0.04, 0.96, "time <1.2 m: 3.9% · <0.5 m: 0.5% · near-contacts: 0",
         transform=axB.transAxes, color=INK, fontsize=10, va="top",
         bbox=dict(boxstyle="round,pad=0.3", fc=SURFACE, ec=GRID))

# ---------- C: exemplar killed run (seed ...015) ----------
SEED = "20260723015"
boxes, cyls, parts, doors = [], [], [], []
for line in open(f"{RUNS}/world_{SEED}.csv"):
    p = line.strip().split(",")
    if p[0] == "box" and p[1].startswith("partition"):
        parts.append((float(p[2]), float(p[3]), float(p[4]), float(p[5])))
    elif p[0] == "box":
        boxes.append((float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6])))
    elif p[0] == "cyl":
        cyls.append((float(p[2]), float(p[3]), float(p[4])))
    elif p[0] == "door":
        doors.append((float(p[2]), float(p[3]), float(p[4])))

def rot_rect(a, cx, cy, sx, sy, yaw, fc, ec, z=2):
    r = Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=fc, edgecolor=ec, lw=0.8, zorder=z)
    r.set_transform(transforms.Affine2D().rotate_deg_around(cx, cy, yaw) + a.transData)
    a.add_patch(r)

for bx, by, sx, sy in [(0, 10, 20, 0.5), (0, -10, 20, 0.5), (10, 0, 0.5, 20), (-10, 0, 0.5, 20)]:
    rot_rect(axC, bx, by, sx, sy, 0, BASE, BASE)
for bx, by, sx, sy, yaw in boxes:
    rot_rect(axC, bx, by, sx, sy, yaw, "#d8d7d0", BASE)
for cx_, cy_, r_ in cyls:
    axC.add_patch(Circle((cx_, cy_), r_, facecolor="#d8d7d0", edgecolor=BASE, lw=0.8, zorder=2))
for px_, py_, sx, sy in parts:
    rot_rect(axC, px_, py_, sx, sy, 0, "#a49bb0", "#7d7590", z=3)

d = np.genfromtxt(dyn_files[SEED], delimiter=",", skip_header=2)
t, x, y = d[:, 0], d[:, 1], d[:, 2]
for i, (px, py) in enumerate([(d[:, 4], d[:, 5]), (d[:, 6], d[:, 7]), (d[:, 8], d[:, 9])]):
    if np.hypot(px - px[0], py - py[0]).max() > 1.0:
        axC.plot(px, py, color=PCOL[i], lw=1.0, alpha=0.6, zorder=3)
        axC.plot(px[-1], py[-1], "o", ms=8, mfc=PCOL[i], mec=INK, mew=1.0, zorder=6)
pts = np.c_[x, y].reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
axC.add_collection(LineCollection(segs, cmap=cmap, array=t[:-1], linewidths=2.0,
                                  zorder=4, capstyle="round"))
k0 = np.searchsorted(t, max(0.0, t[-1] - 99.0))
net = float(np.hypot(x[-1] - x[k0], y[-1] - y[k0]))
path_w = float(np.hypot(np.diff(x[k0:]), np.diff(y[k0:])).sum())
axC.plot(x[k0], y[k0], "o", ms=11, mfc="none", mec=INK, mew=1.6, zorder=7)
axC.plot(x[-1], y[-1], "o", ms=13, mfc="none", mec=RED, mew=2.2, zorder=7)
axC.plot([x[k0], x[-1]], [y[k0], y[-1]], color=RED, lw=1.2, ls="--", zorder=6)
axC.annotate(f"window start\n(t={t[k0]:.0f}s)", (x[k0], y[k0]), textcoords="offset points",
             xytext=(10, 8), fontsize=9, color=INK, zorder=8)
axC.annotate(f"killed here (t={t[-1]:.0f}s)\nnet {net:.2f} m — walked {path_w:.0f} m",
             (x[-1], y[-1]), textcoords="offset points", xytext=(10, -26),
             fontsize=9.5, color=RED, fontweight="bold", zorder=8)
axC.set_xlim(-10.8, 10.8); axC.set_ylim(-10.8, 10.8)
axC.set_aspect("equal")
axC.set_title(f"C · What the watchdog killed — seed …015, cruising at "
              f"{d[:, 0][-1] and np.hypot(np.diff(x), np.diff(y)).sum()/t[-1]:.2f} m/s",
              color=INK, fontsize=12, loc="left")

# ---------- D: all 16 kills were healthy ----------
forensics = []
for r in killed:
    dd = np.genfromtxt(dyn_files[r["seed"]], delimiter=",", skip_header=2)
    tt, xx, yy = dd[:, 0], dd[:, 1], dd[:, 2]
    kk = np.searchsorted(tt, tt[-1] - 100.0)
    path = float(np.hypot(np.diff(xx[kk:]), np.diff(yy[kk:])).sum())
    spread = float(np.hypot(xx[kk:].max() - xx[kk:].min(), yy[kk:].max() - yy[kk:].min()))
    forensics.append((r["seed"][-3:], path, spread))
forensics.sort(key=lambda q: q[1])
ypos = np.arange(len(forensics))
axD.barh(ypos, [f[1] for f in forensics], height=0.62, color=BLUE, zorder=3,
         label="path walked in final 100 sim-s")
axD.plot([f[2] for f in forensics], ypos, "o", ms=7, mfc=AQUA, mec=SURFACE, mew=1.0,
         zorder=5, ls="none", label="area covered (bbox diagonal)")
axD.axvline(0.5, color=RED, lw=1.6, ls="--", zorder=4)
axD.annotate("watchdog \"stillness\" threshold: 0.5 m", (0.5, len(forensics) + 0.9),
             xytext=(5.5, len(forensics) + 0.75), fontsize=9, color=RED, va="center",
             arrowprops=dict(arrowstyle="->", color=RED, lw=1.0))
axD.set_yticks(ypos, [f"…{f[0]}" for f in forensics], fontsize=8.5)
axD.set_xlim(0, 28)
axD.set_ylim(-0.7, len(forensics) + 1.5)
axD.set_xlabel("metres in the very window the watchdog read as net <0.5 m", color=INK2, fontsize=10)
axD.set_title("D · All 16 \"deadlock\" kills were healthy runs", color=INK, fontsize=12, loc="left")
axD.legend(loc="lower right", fontsize=9, frameon=True, facecolor=SURFACE, edgecolor=GRID)

fig.text(0.07, 0.012,
         "Rules — deadlock: excursion <0.5 m over 90 sim-s (0 events, both cohorts) · dance: 60 s net <0.35 m while "
         "path >2.5 m (2 static, 2 dynamic) · v2 watchdog compared only trail endpoints (loop closure ⇒ false kill); "
         "v3 uses full-trail spread, sim-gated.",
         color=MUTED, fontsize=8.5)
fig.savefig("/home/user/isaac/dynamic_batch_cost_of_dynamics.png", facecolor=PAGE,
            bbox_inches="tight")
print("saved dynamic_batch_cost_of_dynamics.png")
