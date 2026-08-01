#!/usr/bin/env python3
# THE Figure: Doorway traversal success vs width — 25 procedurally generated worlds
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BLUE = "#2a78d6"

# bins: (lo, hi, approached, crossed)
bins = [(0.50, 0.60, 6, 1), (0.60, 0.66, 1, 1), (0.66, 0.72, 8, 5),
        (0.72, 0.80, 9, 5), (0.80, 0.95, 13, 11)]

def wilson(k, n, z=1.96):
    if n == 0:
        return 0, 0, 0
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0, c - h), min(1, c + h)

fig, ax = plt.subplots(figsize=(10, 6.6), dpi=180)
fig.patch.set_facecolor(PAGE)
ax.set_facecolor(SURFACE)

for lo, hi, n, k in bins:
    mid = (lo + hi) / 2
    p, lo_ci, hi_ci = wilson(k, n)
    ax.errorbar(mid, p * 100, yerr=[[100 * (p - lo_ci)], [100 * (hi_ci - p)]],
                fmt="o", ms=10, mfc=BLUE, mec=BLUE, ecolor=BLUE, elinewidth=1.8,
                capsize=5, capthick=1.8, zorder=5)
    ax.annotate(f"{k}/{n}", (mid, p * 100), textcoords="offset points", xytext=(12, -4),
                fontsize=10.5, color=INK2, zorder=6)

# connecting line (thin, auxiliary)
mids = [(lo + hi) / 2 for lo, hi, _, _ in bins]
ps = [100 * k / n for _, _, n, k in bins]
ax.plot(mids, ps, "-", color=BLUE, lw=1.2, alpha=0.45, zorder=4)

# reference lines
ax.axvline(0.42, color=MUTED, lw=1.2, ls="--", zorder=3)
ax.text(0.423, 6, "robot width 0.42 m", rotation=90, color=MUTED, fontsize=9, va="bottom")
ax.axvline(0.66, color=MUTED, lw=1.2, ls=":", zorder=3)
ax.text(0.663, 6, "nominal min 0.66 m (0.42 + 2×0.12 inflation)", rotation=90,
        color=MUTED, fontsize=9, va="bottom")

# the outlier achievement
ax.annotate("0.57 m: crossed twice via COMMIT crawl", (0.57, 100 / 6), textcoords="offset points",
            xytext=(-8, 26), fontsize=9.5, color=INK, zorder=6,
            arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.0),
            bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE, ec=GRID, alpha=0.9))

ax.set_xlim(0.38, 1.0)
ax.set_ylim(-4, 104)
ax.set_xlabel("door width (m)", color=INK2, fontsize=11)
ax.set_ylabel("traversal success of approached doors (%)", color=INK2, fontsize=11)
ax.grid(True, color=GRID, lw=0.7)
ax.tick_params(colors=MUTED, labelsize=9.5)
for s in ax.spines.values():
    s.set_color(GRID)
ax.set_title("Doorway traversal success vs width — 25 procedurally generated worlds, navigator v2.9.1",
             color=INK, fontsize=13, loc="left", y=1.06)
stats = ("4.55 h sim · 3.70 km · 100 labeled doors (37 approached) · 36 crossings · "
         "5 COMMIT completions · 0 terminal deadlocks · Wilson 95% CI")
ax.text(0, 1.018, stats, transform=ax.transAxes, color=INK2, fontsize=9.5, va="bottom")

fig.tight_layout()
out = "/home/user/isaac/success_vs_width_batch1.png"
fig.savefig(out, facecolor=PAGE, bbox_inches="tight")
print(out)
