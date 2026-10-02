#!/usr/bin/env python3
# Odometry-noise ablation on world 659927: one mapping run per noise tier (0/0 twice, 3/5, 8/10),
# all with walking people and the same navigator. Map metrics come from evaluate_map.evaluate().
# Drift is measured from the per-run odometry log (odom vs truth at 10 Hz). Only the 8/10 run has
# one, the log was added to odom_publisher after the 0/0 and 3/5 runs.
import os
import glob
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import evaluate_map as em

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"
BLUE = "#2a78d6"; AQUA = "#1baf7a"; ORANGE = "#eb6834"

MANIFEST = f"{RUNS_DIR}/world_659927.csv"
MAPS = ["_n00", "", "_n810"]          # 0/0, 3/5, 8/10
REPLICATE = "_n00b"                   # second 0/0 run (run-to-run spread)
res = {t: em.evaluate(f"{RUNS_DIR}/world_659927{t}.yaml", MANIFEST) for t in MAPS + [REPLICATE]}
prec_main = [res[t]["prec"] * 100 for t in MAPS]
prec_rep = res[REPLICATE]["prec"] * 100
free_iou = [res[t]["free_iou"] * 100 for t in MAPS]
cov = [res[t]["surf_cov"] * 100 for t in MAPS]
shifts = [float(np.hypot(r["dx"], r["dy"])) for r in res.values()]
all_prec = prec_main + [prec_rep]
spread_all = max(all_prec) - min(all_prec)
spread_rep = abs(prec_rep - prec_main[0])

# measured drift of the 8/10 run (its log header records the active noise tier)
logs = [f for f in sorted(glob.glob(f"{RUNS_DIR}/odom_659927_*.csv"))
        if open(f).readline().startswith("# noise_v=0.08 noise_w=0.1")]
assert len(logs) == 1, logs
od = np.genfromtxt(logs[0], delimiter=",", skip_header=2)
dmax = float(np.hypot(od[:, 0] - od[:, 3], od[:, 1] - od[:, 4]).max())

tiers = ["0% / 0%\n(no noise)", "3% / 5%\n(realistic)", "8% / 10%\n(heavy)"]
drift = ["drift 0 m", "drift not logged", f"measured drift\nup to {dmax:.1f} m"]
print("precision", [f"{p:.1f}" for p in prec_main], f"replicate {prec_rep:.1f}")
print("surface coverage", [f"{c:.1f}" for c in cov], "free IoU", [f"{f:.1f}" for f in free_iou])
print(f"8/10 max drift {dmax:.2f} m ({os.path.basename(logs[0])}), alignment shifts {shifts}")

fig, ax = plt.subplots(figsize=(8.8, 5.8), dpi=170)
fig.patch.set_facecolor(PAGE)
ax.set_facecolor(SURFACE)
ax.grid(True, color=GRID, lw=0.7)
ax.tick_params(colors=MUTED, labelsize=9.5)
for s in ax.spines.values():
    s.set_color(GRID)

x = np.arange(3)
ax.plot(x, prec_main, "-o", color=BLUE, lw=2.2, ms=9, mec=SURFACE, mew=1.2, zorder=5,
        label="occupied precision")
ax.plot([0], [prec_rep], "o", ms=9, mfc="none", mec=BLUE, mew=1.8, zorder=5,
        label="0/0 replicate (run-to-run spread)")
ax.plot(x, free_iou, "-o", color=AQUA, lw=1.6, ms=7, mec=SURFACE, mew=1.0, zorder=4,
        label="free-space IoU")
ax.plot(x, cov, "--o", color=MUTED, lw=1.4, ms=7, mfc=PAGE, zorder=3,
        label="surface coverage (route-dependent)")
for i in range(3):
    ax.annotate(f"{prec_main[i]:.1f}%", (x[i], prec_main[i]), textcoords="offset points",
                xytext=(-12, -4) if i == 0 else (0, 11), ha="right" if i == 0 else "center",
                fontsize=10, color=INK, fontweight="bold")
    ax.annotate(drift[i], (x[i], 58.5), ha="center", fontsize=8.3, color=ORANGE if i == 2 else MUTED)
ax.annotate(f"{prec_rep:.1f}%", (0, prec_rep), textcoords="offset points",
            xytext=(12, 2), fontsize=9, color=BLUE)
ax.annotate(f"all four maps within {spread_all:.1f} pp, two runs without noise differ by {spread_rep:.1f} pp:\n"
            "no trend with the noise level",
            (1.02, 90.5), ha="center", fontsize=9.2, color=INK2, style="italic")
ax.set_xticks(x, tiers, fontsize=10)
ax.set_ylim(55, 100)
ax.set_xlim(-0.4, 2.3)
ax.set_xlabel("odometry noise (multiplicative σ: linear / angular)", color=INK2, fontsize=10.5)
ax.set_ylabel("map quality vs ground truth (%)", color=INK2, fontsize=10.5)
ax.set_title(f"Odometry-noise ablation: up to {dmax:.0f} m of raw drift, map precision barely moves",
             color=INK, fontsize=13, loc="left", pad=14)
ax.text(0.0, 1.015,
        f"world 659927, walking people, one run per tier · 8/10 tier confirmed by its odometry log · "
        f"alignment residual {min(shifts):.1f}-{max(shifts):.1f} m",
        transform=ax.transAxes, color=INK2, fontsize=8.8)
ax.legend(loc="lower left", bbox_to_anchor=(0.02, 0.10), fontsize=9, frameon=True,
          facecolor=SURFACE, edgecolor=GRID)
fig.savefig(RUNS_DIR + "/slam_noise_ablation.png", facecolor=PAGE, bbox_inches="tight")
print("saved slam_noise_ablation.png")
