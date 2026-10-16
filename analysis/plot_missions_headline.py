#!/usr/bin/env python3
# Headline figure: real arrivals of the revised Nav2 configuration on the 25 benchmark worlds
# (perfect maps and the robot's own SLAM maps) and on 10 worlds generated after it was frozen.
# Reads runs/missions_stats.json (stats_missions.py) and runs/missions_batch_b.json
# (stats_batch_b.py). Writes runs/missions_headline.png.
import os
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"
BLUE = "#2a78d6"

S = json.load(open(RUNS_DIR + "/missions_stats.json"))["arms"]
B = json.load(open(RUNS_DIR + "/missions_batch_b.json"))["arms"]
BARS = [(S["rev_truth"], "perfect maps", "25 benchmark worlds"),
        (S["rev_slam"], "the robot's own SLAM maps", "same 25 worlds"),
        (B["rev_heldout"], "perfect maps, new worlds", "10 worlds generated after\nthe configuration was frozen")]

fig = plt.figure(figsize=(10.4, 5.6), dpi=150)
fig.patch.set_facecolor(PAGE)
ax = fig.add_axes([0.08, 0.20, 0.88, 0.56])
fig.text(0.08, 0.95, "Goal-directed missions with the revised Nav2 configuration", color=INK, fontsize=15.5,
         va="top")
fig.text(0.08, 0.885,
         "Tours of 4 goals among walking people · 120 simulated seconds per attempt · a real arrival is a "
         "reported success\nwith the robot's true position within 0.6 m of the goal · whiskers: 95% interval "
         "from a bootstrap over worlds",
         color=INK2, fontsize=9.8, va="top")
ax.set_facecolor(SURFACE)
ax.grid(True, axis="y", color=GRID, lw=0.7)
ax.set_axisbelow(True)
for s in ax.spines.values():
    s.set_visible(False)
ax.tick_params(colors=MUTED, labelsize=9.5, length=0)
for i, (a, lab, sub) in enumerate(BARS):
    r = 100 * a["rate"]
    lo, hi = (100 * v for v in a["rate_ci"])
    ax.bar(i, r, width=0.42, color=BLUE, zorder=2)
    ax.plot([i, i], [lo, hi], color=INK2, lw=1.3, zorder=3)
    ax.plot([i - 0.05, i + 0.05], [lo, lo], color=INK2, lw=1.3, zorder=3)
    ax.plot([i - 0.05, i + 0.05], [hi, hi], color=INK2, lw=1.3, zorder=3)
    ax.text(i, hi + 2.5, f"{a['real']}/{a['goals']}", ha="center", va="bottom", color=INK, fontsize=11,
            fontweight="bold")
    ax.text(i, hi + 10.5, f"{r:.0f}%", ha="center", va="bottom", color=INK2, fontsize=9.5)
ax.set_xticks(range(len(BARS)), [f"{lab}\n{sub}" for _, lab, sub in BARS], color=INK2, fontsize=9.8)
ax.set_xlim(-0.6, len(BARS) - 0.4)
ax.set_ylim(0, 112)
ax.set_yticks(range(0, 101, 20))
ax.set_ylabel("goals reached (real arrivals, %)", color=INK2, fontsize=10.5)
out = RUNS_DIR + "/missions_headline.png"
fig.savefig(out, facecolor=PAGE)
print("figure ->", out)
