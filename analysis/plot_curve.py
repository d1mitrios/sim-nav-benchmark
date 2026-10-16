#!/usr/bin/env python3
# Experiment 1: navigator v2.9.1 against the v1 baseline in the same 25 generated worlds, and every
# door v2.9.1 approached, by width, crossed or not.
# Top: terminal deadlocks, distance driven and door crossings, v1 -> v2.9.1.
# Bottom: one dot per approached door at its width, in three width groups (below the 0.66 m nominal
# minimum, 0.66-0.80 m, 0.80-0.95 m) with the share crossed in each.
# Door data from analyze_batch.py (runs/door_width_rows.json), the paired v1 / v2.9.1 totals from
# compare_v1_v2.py (runs/compare_rows.json).
import os
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import json
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Ellipse
plt.rcParams["font.family"] = "DejaVu Sans"

PAGE = "#f9f9f7"; CARD = "#ffffff"; CARD_EDGE = "#e6e5de"; BAND = "#f0efea"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"
BLUE = "#2a78d6"; ORANGE = "#eb6834"

D = json.load(open(os.path.join(RUNS_DIR, "door_width_rows.json")))
C = json.load(open(os.path.join(RUNS_DIR, "compare_rows.json")))
N = len(C)
NOMINAL_MIN = 0.66          # 0.42 m body + 2 x 0.12 m inflation
GROUPS = [(0.50, 0.66, f"below {NOMINAL_MIN:.2f} m, the nominal minimum"),
          (0.66, 0.80, "0.66–0.80 m"), (0.80, 0.95, "0.80–0.95 m")]

doors = D["approached_doors"]
assert len(doors) == D["approached"]

v1 = dict(term=sum(r["term1"] for r in C), dist=sum(r["dist1"] for r in C), cross=sum(r["c1"] for r in C))
v2 = dict(term=sum(r["term2"] for r in C), dist=sum(r["dist2"] for r in C), cross=sum(r["c2"] for r in C))
onset_v1 = float(np.median([r["term_t1"] for r in C if r["term_t1"] is not None]))
cut = [r for r in C if r["d1"] < 300]        # v1 runs the batch watchdog ended early
assert all(r["term_t1"] is not None and r["term_t1"] < r["d1"] for r in cut)
nc = D["narrowest_crossed"]
times = {1: "once", 2: "twice"}.get(nc["crossings"], f'{nc["crossings"]} times')

fig = plt.figure(figsize=(12, 7.4), dpi=150)
fig.patch.set_facecolor(PAGE)
FW, FH = fig.get_figwidth(), fig.get_figheight()
L, R = 0.045, 0.955
renderer = fig.canvas.get_renderer()


def text_width(t):
    return t.get_window_extent(renderer).width / (FW * fig.dpi)


def minus(s):
    return s.replace("-", "−")


def dot(x, y, r, **kw):
    """A round dot of radius r (in figure-width units) at figure coordinates x, y."""
    fig.patches.append(Ellipse((x, y), 2 * r, 2 * r * FW / FH, transform=fig.transFigure, figure=fig, **kw))


fig.text(L, 0.955, "Experiment 1: from a deadlock in every world to 36 door crossings", color=INK,
         fontsize=17, va="top")
fig.text(L, 0.905, f"Reactive navigator v2.9.1 against the v1 baseline, the same follow-the-gap core without "
         f"the recovery layers.\n{N} generated worlds, 20 wall-clock minutes each.", color=INK2, fontsize=10,
         va="top", linespacing=1.45)
# colour key for the cards
kx = 0.78
dot(kx, 0.885, 0.0045, color=ORANGE)
fig.text(kx + 0.011, 0.885, "v1 baseline", color=INK2, fontsize=10, va="center")
dot(kx + 0.105, 0.885, 0.0045, color=BLUE)
fig.text(kx + 0.116, 0.885, "v2.9.1", color=INK2, fontsize=10, va="center")

# ---------- top: three cards, v1 -> v2.9.1 ----------
TOP, H = 0.835, 0.165
GAPX = 0.02
cw = (R - L - 2 * GAPX) / 3
cards = [
    ("Terminal deadlocks", f'{v1["term"]}', f'{v2["term"]}', f"worlds out of {N}"),
    ("Distance driven", f'{v1["dist"]:.0f} m', f'{v2["dist"] / 1000:.2f} km',
     f'{v2["dist"] / v1["dist"]:.0f} times farther'),
    ("Door crossings", f'{v1["cross"]}', f'{v2["cross"]}', f'{v2["cross"] / v1["cross"]:.0f} times more'),
]
for i, (label, a, b, note) in enumerate(cards):
    x0 = L + i * (cw + GAPX)
    fig.patches.append(FancyBboxPatch((x0, TOP - H), cw, H, boxstyle="round,pad=0,rounding_size=0.012",
                                      mutation_aspect=FW / FH, fc=CARD, ec=CARD_EDGE, lw=0.9,
                                      transform=fig.transFigure, figure=fig))
    fig.text(x0 + 0.018, TOP - 0.03, label, color=INK2, fontsize=10.5, va="top")
    yv = TOP - 0.088
    xx = x0 + 0.018
    for k, (val, col, size, weight, ink) in enumerate(((a, ORANGE, 18, "normal", MUTED),
                                                       (b, BLUE, 24, "bold", INK))):
        dot(xx + 0.004, yv, 0.0042, color=col)
        t = fig.text(xx + 0.013, yv, val, color=ink, fontsize=size, fontweight=weight, va="center")
        xx += 0.013 + text_width(t) + 0.012
        if k == 0:
            arrow = fig.text(xx, yv, "→", color=MUTED, fontsize=16, va="center")
            xx += text_width(arrow) + 0.012
    fig.text(x0 + 0.018, TOP - H + 0.022, note, color=INK2, fontsize=9.5, va="bottom")

# ---------- bottom: every approached door, by width ----------
fig.text(L, 0.6, "Every door that v2.9.1 approached, by width", color=INK, fontsize=12.5,
         fontweight="bold", va="center")
# legend
lx = 0.70
dot(lx, 0.6, 0.0055, color=BLUE)
fig.text(lx + 0.012, 0.6, "crossed", color=INK2, fontsize=10, va="center")
lx2 = lx + 0.1
dot(lx2, 0.6, 0.0047, fc=PAGE, ec=ORANGE, lw=1.8)
fig.text(lx2 + 0.012, 0.6, "approached, not crossed", color=INK2, fontsize=10, va="center")

ax = fig.add_axes([L, 0.175, R - L, 0.375])
ax.set_xlim(0.495, 0.955)
ax.set_ylim(-0.6, 5.6)
ax.patch.set_alpha(0)
for s in ax.spines.values():
    s.set_visible(False)
ax.set_yticks([])
ax.set_xticks(np.arange(0.5, 0.951, 0.05))
ax.set_xticklabels([f"{v:.2f}" for v in np.arange(0.5, 0.951, 0.05)])
ax.tick_params(axis="x", colors=MUTED, labelsize=9.5, length=0, pad=4)
ax.axhline(-0.6, color=GRID, lw=1.0)
for gi, (lo, hi, name) in enumerate(GROUPS):
    if gi % 2 == 0:
        ax.axvspan(lo, hi, ymin=0, ymax=1, color=BAND, zorder=0, lw=0)
    sel = [d for d in doors if (lo <= d["width"] < hi) or (hi == GROUPS[-1][1] and d["width"] == hi)]
    k = sum(1 for d in sel if d["crossings"] > 0)
    mid = (lo + hi) / 2
    ax.text(mid, 5.25, f"{100 * k / len(sel):.0f}%", ha="center", va="top", color=INK, fontsize=22,
            fontweight="bold")
    ax.text(mid, 3.85, f"{k} of {len(sel)} crossed", ha="center", va="top", color=INK2, fontsize=10.5)
    ax.text(mid, 3.2, name, ha="center", va="top", color=MUTED, fontsize=9)
# the dots, stacked where widths coincide (to the centimetre)
stack = Counter()
for d in sorted(doors, key=lambda d: (d["width"], -d["crossings"])):
    w = round(d["width"], 2)
    y = stack[w]
    stack[w] += 1
    if d["crossings"] > 0:
        ax.scatter(w, y, s=150, color=BLUE, edgecolor=PAGE, linewidth=1.5, zorder=3)
    else:
        ax.scatter(w, y, s=118, facecolor=PAGE, edgecolor=ORANGE, linewidth=2.0, zorder=3)
ax.text(0.725, -1.3, "door width (m)", ha="center", va="top", color=MUTED, fontsize=9.5)

notes = [
    f"A door counts as approached when the robot came within 0.6 m of its centre. Nominal minimum width "
    f"{NOMINAL_MIN:.2f} m: 0.42 m body plus 0.12 m inflation on each side.",
    f"The narrowest door crossed was {nc['width']:.2f} m wide ({times}). v1 deadlocked after a median "
    f"{onset_v1:.0f} simulated seconds, and in {len(cut)} worlds the batch watchdog then ended its run after "
    f"{min(r['d1'] for r in cut):.0f}–{max(r['d1'] for r in cut):.0f} s.",
]
for k, line in enumerate(notes):
    fig.text(L, 0.06 - k * 0.026, line, color=MUTED, fontsize=8.5, va="bottom")

out = os.path.join(RUNS_DIR, "success_vs_width_batch1.png")
fig.savefig(out, facecolor=PAGE)
print("figure -> " + out)
