#!/usr/bin/env python3
# Experiments 6, 7, 8 and 5b: which mechanism ended each failed goal attempt, per arm.
# Reads runs/missions_failures.json (failure_mechanisms.py) and runs/missions_stats.json.
# The ten labels of failure_mechanisms.py are folded into seven groups. Writes
# runs/missions_failures.png.
import os
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"; BASE = "#c3c2b7"
WHITE = "#ffffff"

GROUPS = [  # (labels folded together, legend text, fill, text colour inside the bar)
    (["localization"], "localization lost (AMCL > 1 m off)", "#2a78d6", WHITE),
    (["stop circle"], "collision-monitor stop (v9 only)", "#eb6834", INK),
    (["refused"], "no route on the map", "#1baf7a", INK),
    (["phantom"], "route through unexplored space (SLAM maps)", "#eda100", INK),
    (["deadband"], "commands below the drive deadband", "#e87ba4", INK),
    (["gave up", "out of time"], "ran out of recoveries or time", "#008300", WHITE),
    (["walker", "stuck", "other"], "walker, stuck or other", BASE, INK),
]
ARMS = [("v9_truth", "v9", "ground-truth map"), ("v9_slam", "v9", "SLAM map"),
        ("rev_truth", "revised", "ground-truth map"), ("rev_slam", "revised", "SLAM map")]

F = json.load(open(RUNS_DIR + "/missions_failures.json"))
S = json.load(open(RUNS_DIR + "/missions_stats.json"))["arms"]
missing = {k for g in GROUPS for k in g[0]}
for arm in F.values():
    extra = set(arm["counts"]) - missing
    assert not extra, f"labels without a group: {extra}"

fig = plt.figure(figsize=(12.6, 5.6), dpi=150)
fig.patch.set_facecolor(PAGE)
ax = fig.add_axes([0.20, 0.13, 0.62, 0.55])
fig.text(0.03, 0.955, "Why goal attempts failed", color=INK, fontsize=16.5, va="top")
fig.text(0.03, 0.89,
         "Each attempt that did not end in a real arrival gets one primary label, the first rule that applies "
         "in a fixed order\n(failure_mechanisms.py) · bars show shares, the counts are written inside",
         color=INK2, fontsize=10, va="top")
ax.set_facecolor(SURFACE)
for s in ax.spines.values():
    s.set_visible(False)
ax.tick_params(colors=MUTED, labelsize=9.5, length=0)
ax.set_xlim(0, 100)
ax.set_xticks(range(0, 101, 20), [f"{v}%" for v in range(0, 101, 20)])
for v in range(0, 101, 20):
    ax.axvline(v, color=GRID, lw=0.7, zorder=0)
H = 0.62
rows = [a for a in ARMS if a[0] in F]
for ri, (key, cfg, mk) in enumerate(rows):
    counts = F[key]["counts"]
    total = sum(counts.values())
    left = 0.0
    for labels, _, fill, tcol in GROUPS:
        n = sum(counts.get(k, 0) for k in labels)
        if n == 0:
            continue
        w = 100.0 * n / total
        ax.barh(ri, w - 0.25, left=left, height=H, color=fill, zorder=2)
        if w >= 3.5:
            ax.text(left + w / 2, ri, str(n), ha="center", va="center", color=tcol, fontsize=9)
        left += w
    worlds = S.get(key, {}).get("worlds")
    ax.text(101.5, ri, f"{total} failed attempts\nin {worlds} worlds", va="center", color=INK2, fontsize=9,
            clip_on=False)
ax.set_yticks(range(len(rows)), [f"{cfg} configuration\n{mk}" for _, cfg, mk in rows], color=INK2,
              fontsize=10)
ax.set_ylim(len(rows) - 0.5, -0.5)
fig.legend(handles=[Patch(color=f, label=l) for _, l, f, _ in GROUPS], loc="upper left",
           bbox_to_anchor=(0.03, 0.80), ncol=4, fontsize=9, frameon=False, labelcolor=INK2,
           handlelength=1.2, columnspacing=1.6)
fig.text(0.03, 0.02, "a failed attempt is any attempt without a real arrival (a goal can have several) · "
         "localization is checked against the simulator's true pose", color=MUTED, fontsize=8.6)
out = RUNS_DIR + "/missions_failures.png"
fig.savefig(out, facecolor=PAGE)
print("figure ->", out)
