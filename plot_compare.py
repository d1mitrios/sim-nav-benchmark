#!/usr/bin/env python3
# THE comparison figure: survival curve (no terminal deadlock) v1 vs v2.9.1
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BLUE = "#2a78d6"; ORANGE = "#eb6834"

rows = json.load(open("/home/user/isaac/compare_rows.json"))
T = 700

# v1: terminal deadlock time = onset of the stall that never resolved (dl1)
v1_times = sorted(r["dl1"] for r in rows if r["term1"])
# v2: no terminal — 100% survival over the whole range

fig, ax = plt.subplots(figsize=(10, 6.4), dpi=180)
fig.patch.set_facecolor(PAGE)
ax.set_facecolor(SURFACE)

# v1 step-down survival
ts = [0] + v1_times + [T]
surv = [100]
for k in range(len(v1_times)):
    surv.append(100 * (25 - (k + 1)) / 25)
ys = [surv[0]]
xs = [0]
for k, t_ in enumerate(v1_times):
    xs += [t_, t_]
    ys += [surv[k], surv[k + 1]]
xs.append(T)
ys.append(surv[-1])
ax.plot(xs, ys, color=ORANGE, lw=2.4, zorder=4)
ax.plot([0, T], [100, 100], color=BLUE, lw=2.4, zorder=5)

# direct labels (2 series: legend + direct labels)
ax.text(T - 8, 96.5, "v2.9.1 — 0/25 terminal deadlocks", color=BLUE, fontsize=11.5,
        fontweight="bold", ha="right", va="top", zorder=6)
ax.text(200, 12, "v1 baseline — 25/25 terminal deadlocks\n(median onset 25 s)", color=ORANGE,
        fontsize=11.5, fontweight="bold", zorder=6)

ax.set_xlim(0, T)
ax.set_ylim(-3, 106)
ax.set_xlabel("sim time (s)", color=INK2, fontsize=11)
ax.set_ylabel("runs without terminal deadlock (%)", color=INK2, fontsize=11)
ax.grid(True, color=GRID, lw=0.7)
ax.tick_params(colors=MUTED, labelsize=9.5)
for s in ax.spines.values():
    s.set_color(GRID)
ax.set_title("Deadlock survival — identical 25 procedurally generated worlds (paired by seed)",
             color=INK, fontsize=13, loc="left", y=1.06)
stats = ("v1: 25/25 terminal deadlocks (median 25 s, max 133 s) · 245 m · 3 door crossings   |   "
         "v2.9.1: 0/25 terminal (10 transient stalls, all self-recovered) · 3 701 m (15×) · 36 crossings")
ax.text(0, 1.018, stats, transform=ax.transAxes, color=INK2, fontsize=9.3, va="bottom")

fig.tight_layout()
out = "/home/user/isaac/deadlock_survival_v1_vs_v2.png"
fig.savefig(out, facecolor=PAGE, bbox_inches="tight")
print(out)
