#!/usr/bin/env python3
# Experiments 6, 7, 8 and 5b: the 2 x 2 design {v9, revised} x {ground-truth map, SLAM map}, paired by world.
# One row per comparison: the 25 worlds as squares shaded by the goals gained or lost in that world,
# and the mean change per world with its 95% bootstrap interval on a common axis.
# Reads runs/missions_stats.json (stats_missions.py). Writes runs/missions_2x2.png.
import os
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
plt.rcParams["font.family"] = "DejaVu Sans"

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")

PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"; ZERO = "#a9a8a0"
# goals gained (blue, light to dark) or lost (orange) in one world, 0 = neutral grey
SHADE = {-2: "#d9532a", -1: "#f2a07e", 0: "#d4d3cc", 1: "#86b6ef", 2: "#5598e7", 3: "#2a78d6", 4: "#184f95"}
BLUE = "#2a78d6"

S = json.load(open(os.path.join(RUNS_DIR, "missions_stats.json")))
A, C = S["arms"], S["comparisons"]

SECTIONS = [
    ("Changing the configuration: the revised one instead of v9", [
        ("rev_truth_vs_v9_truth", "rev_truth", "v9_truth", "on perfect maps"),
        ("rev_slam_vs_v9_slam", "rev_slam", "v9_slam", "on the robot's own SLAM maps")]),
    ("Changing the map: a perfect map instead of the robot's own", [
        ("rev_truth_vs_rev_slam", "rev_truth", "rev_slam", "with the revised configuration"),
        ("v9_truth_vs_v9_slam", "v9_truth", "v9_slam", "with v9")]),
]


SUP = str.maketrans("-0123456789", "\u207b\u2070\u00b9\u00b2\u00b3\u2074\u2075\u2076\u2077\u2078\u2079")


def ptext(p):
    if p < 0.001:
        e = int(np.floor(np.log10(p)))
        return f"p = {p / 10 ** e:.1f} \u00d7 10" + str(e).translate(SUP)
    return f"p = {p:.3f}" if p < 0.01 else f"p = {p:.2f}"


def minus(s):
    return s.replace("-", "−")


fig = plt.figure(figsize=(12.4, 6.6), dpi=150)
fig.patch.set_facecolor(PAGE)
FW, FH = fig.get_figwidth(), fig.get_figheight()
L = 0.045
fig.text(L, 0.95, "The navigation stack set the ceiling, not the map", color=INK, fontsize=17, va="bottom")
fig.text(L, 0.925, "Nav2 missions in the same 25 generated worlds, compared world by world. Tours of 4 goals, "
         "120 simulated seconds per attempt.\nA goal counts only when the robot's true position is within 0.6 m "
         "of it.", color=INK2, fontsize=10, va="top", linespacing=1.4)

# geometry (figure fractions)
X_STRIP = 0.275; SQ = 0.0104; GAP = 0.0026           # squares: width in figure-width units
X_AX0, X_AX1 = 0.655, 0.95                           # forest axis
Y_TOP, Y_BOT = 0.78, 0.215
ROW_Y = [0.70, 0.585, 0.415, 0.30]
SEC_Y = [0.755, 0.49]

fig.text(X_STRIP, 0.8, "the 25 worlds, one square each", color=MUTED, fontsize=9, va="bottom")
fig.text(X_AX0, 0.8, "change in goals reached per world (of 4)", color=MUTED, fontsize=9, va="bottom")

ax = fig.add_axes([X_AX0, Y_BOT, X_AX1 - X_AX0, Y_TOP - Y_BOT])
ax.set_xlim(-1.2, 4.2)
ax.set_ylim(Y_BOT, Y_TOP)                            # data y = figure y
ax.patch.set_alpha(0)
for s in ax.spines.values():
    s.set_visible(False)
ax.set_yticks([])
ax.set_xticks(range(-1, 5))
ax.set_xticklabels([minus(f"{v:+d}") if v else "0" for v in range(-1, 5)])
ax.tick_params(axis="x", colors=MUTED, labelsize=9.5, length=0, pad=6)
for v in range(-1, 5):
    ax.axvline(v, color=ZERO if v == 0 else GRID, lw=1.1 if v == 0 else 0.8, zorder=0)

ri = 0
for si, (sec_title, rows) in enumerate(SECTIONS):
    fig.text(L, SEC_Y[si], sec_title, color=INK, fontsize=12, va="center", fontweight="bold")
    for key, a_k, b_k, label in rows:
        y = ROW_Y[ri]; ri += 1
        c = C[key]
        pa, pb = A[a_k]["per_world"], A[b_k]["per_world"]
        common = sorted(set(pa) & set(pb))
        d = np.array(sorted((pa[s] - pb[s] for s in common), reverse=True))
        nb, ns, nw = int((d > 0).sum()), int((d == 0).sum()), int((d < 0).sum())
        assert (nb, nw) == (c["better"], c["worse"])
        fig.text(L + 0.012, y, label, color=INK, fontsize=11, va="center")
        # the worlds, sorted from the largest gain to the largest loss
        sq_h = SQ * FW / FH
        for k, v in enumerate(d):
            x0 = X_STRIP + k * (SQ + GAP) if k < 25 else None
            fig.patches.append(FancyBboxPatch((x0, y - sq_h / 2 + 0.008), SQ, sq_h,
                                              boxstyle="round,pad=0,rounding_size=0.0022",
                                              mutation_aspect=FW / FH, fc=SHADE[int(v)], ec="none",
                                              transform=fig.transFigure, figure=fig))
        fig.text(X_STRIP, y - sq_h / 2 - 0.012, f"better in {nb} · same in {ns} · worse in {nw}",
                 color=INK2, fontsize=9, va="top")
        # mean and 95% interval
        lo, hi = c["ci"]
        sig = lo > 0 or hi < 0
        col = BLUE if sig else ZERO
        ax.plot([lo, hi], [y + 0.008, y + 0.008], color=col, lw=6, solid_capstyle="round", zorder=2, alpha=0.35)
        ax.scatter([c["mean_diff"]], [y + 0.008], s=90, color=col if sig else INK2, edgecolor=PAGE,
                   linewidth=2, zorder=3)
        PAD = dict(facecolor=PAGE, edgecolor="none", pad=1.5)
        ax.text(hi + 0.16, y + 0.008, minus(f"{c['mean_diff']:+.2f}"), color=INK, fontsize=13,
                fontweight="bold", va="center", bbox=PAD)
        ax.text(hi + 0.16, y - 0.035, ptext(c["p_sign"]), color=MUTED, fontsize=8.5, va="center", bbox=PAD)

# legend for the shades
yl = 0.125
fig.text(L, yl, "goals gained or lost in a world:", color=INK2, fontsize=9, va="center")
xl = L + 0.165
for v in range(-2, 5):
    sq_h = SQ * FW / FH
    fig.patches.append(FancyBboxPatch((xl, yl - sq_h / 2), SQ, sq_h, boxstyle="round,pad=0,rounding_size=0.0022",
                                      mutation_aspect=FW / FH, fc=SHADE[v], ec="none",
                                      transform=fig.transFigure, figure=fig))
    fig.text(xl + SQ + 0.004, yl, minus(f"{v:+d}") if v else "0", color=INK2, fontsize=9, va="center")
    xl += SQ + 0.036
fig.text(X_AX0, yl, "dot: mean · bar: 95% interval (bootstrap over worlds)\np: sign test over the worlds",
         color=INK2, fontsize=9, va="center", linespacing=1.4)
fig.text(L, 0.045, "v9 is the configuration of the first mission tests, the revised configuration changes "
         "8 of its settings. Perfect maps are rendered from the world manifests.",
         color=MUTED, fontsize=8.5, va="center")

out = os.path.join(RUNS_DIR, "missions_2x2.png")
fig.savefig(out, facecolor=PAGE)
print("figure ->", out)
