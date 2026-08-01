#!/usr/bin/env python3
# THE MONTAGE: 25 worlds mapped in one night + Step 1 statistics
import glob
import json
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"
BLUE = "#2a78d6"; AQUA = "#1baf7a"; ORANGE = "#eb6834"; RED = "#e34948"

D = "/home/user/isaac/slam_batch"
rows = {r["seed"]: r for r in json.load(open("/home/user/isaac/slam_batch_rows.json"))}
em = {}
exec(open("/home/user/isaac/evaluate_map.py").read().split("def main")[0], em)

fig = plt.figure(figsize=(13.2, 17.2), dpi=150)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(6, 5, height_ratios=[1, 1, 1, 1, 1, 0.85], hspace=0.18, wspace=0.06,
                       left=0.03, right=0.97, top=0.90, bottom=0.045)

fig.suptitle("One night, twenty-five worlds — the mapping batch",
             color=INK, fontsize=17, x=0.03, ha="left", y=0.965)
fig.text(0.03, 0.938,
         "v2.9.1 wanders 20 min per procedurally generated world (walking humans, odometry noise 3%/5%) · "
         "slam_toolbox restarted per world by the WSL orchestrator · 25/25 maps saved autonomously",
         color=INK2, fontsize=10.5)
fig.text(0.03, 0.918,
         "grey = mapped occupancy · red = manifest ground truth · label: surface coverage",
         color=INK2, fontsize=10.5, style="italic")

seeds = sorted(rows)
for k, seed in enumerate(seeds):
    ax = fig.add_subplot(gs[k // 5, k % 5])
    r = rows[seed]
    occ, free, res, origin = em["load_map"](f"{D}/world_{seed}.pgm".replace(".pgm", ".yaml"))
    gt, _ = em["render_truth"](f"/mnt/user-data/uploads/isaac_project/runs/world_{seed}.csv",
                               res, occ.shape, origin)
    base = np.full(occ.shape, 0.93)
    base[free] = 1.0
    base[occ] = 0.2
    ax.imshow(base, cmap="gray", origin="lower", vmin=0, vmax=1)
    yy, xx = np.where(gt & ~(np.roll(gt, 1, 0) & np.roll(gt, -1, 0) &
                             np.roll(gt, 1, 1) & np.roll(gt, -1, 1)))
    ax.scatter(xx, yy, s=0.03, c=RED, alpha=0.45)
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color(GRID)
    cov = r["surf"] * 100
    col = AQUA if cov >= 75 else INK2 if cov >= 55 else ORANGE
    ax.set_title(f"…{seed[-3:]} · {cov:.0f}%", color=col, fontsize=10, pad=3)

# ---- statistics strip ----
axH = fig.add_subplot(gs[5, 0:2])
axP = fig.add_subplot(gs[5, 2:4])
axS = fig.add_subplot(gs[5, 4])
for a in (axH, axP, axS):
    a.set_facecolor(SURFACE)
    a.grid(True, color=GRID, lw=0.6)
    a.tick_params(colors=MUTED, labelsize=8)
    for s in a.spines.values():
        s.set_color(GRID)

S_ = [rows[s]["surf"] * 100 for s in seeds]
P_ = [rows[s]["prec"] * 100 for s in seeds]
DX = [rows[s]["dmax"] for s in seeds]
axH.hist(S_, bins=np.arange(40, 100, 6), color=BLUE, edgecolor=SURFACE, zorder=3)
axH.axvline(np.median(S_), color=INK, lw=1.2, ls="--")
axH.set_title(f"surface coverage — median {np.median(S_):.0f}% (47-92%)", fontsize=9.5,
              color=INK, loc="left")
axH.set_xlabel("coverage in 20 min (%)", fontsize=8.5, color=INK2)
axH.text(0.97, 0.92, "the case for Nav2:\nwandering ≠ exploring", transform=axH.transAxes,
         fontsize=8.5, color=ORANGE, ha="right", va="top", style="italic")
axP.hist(P_, bins=np.arange(60, 95, 3.5), color=AQUA, edgecolor=SURFACE, zorder=3)
axP.axvline(np.median(P_), color=INK, lw=1.2, ls="--")
axP.set_title(f"map precision — median {np.median(P_):.0f}%", fontsize=9.5, color=INK, loc="left")
axP.set_xlabel("occupied precision (%)", fontsize=8.5, color=INK2)
axS.plot(DX, P_, "o", ms=5, mfc=BLUE, mec=SURFACE, mew=0.8)
axS.set_title("drift vs precision (r=-0.37)", fontsize=9.5, color=INK, loc="left")
axS.set_xlabel("measured max drift (m)", fontsize=8.5, color=INK2)

fig.text(0.03, 0.012,
         "Measured odometry drift: median 0.28 m, max 1.57 m (per-run odom logs) · alignment residual ≤0.22 m in all 25 · "
         "human encounters vs coverage: r = -0.13 (walking humans do not measurably slow exploration) · "
         "free-space IoU median 76.7%",
         color=MUTED, fontsize=8.5)
fig.savefig("/home/user/isaac/slam_batch_montage.png", facecolor=PAGE, bbox_inches="tight")
print("saved slam_batch_montage.png")
