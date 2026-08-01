#!/usr/bin/env python3
# FINAL Figure Phase 2 — batch #2 (full 25x20min, v3 watchdog validated).
import glob
import json
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"
GRID = "#e1e0d9"; BASE = "#b9b8ae"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; AQUA = "#1baf7a"; RED = "#e34948"

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
rows = json.load(open("/home/user/isaac/batch2_rows.json"))

# total human km in batch #2
d2_files = {}
for f in glob.glob(f"{RUNS}/run_202607230*.csv"):
    st = re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)
    if st >= "20260724_22":
        d2_files[re.search(r"run_(\d+)_", f).group(1)] = f
hkm = 0.0
for f in d2_files.values():
    d = np.genfromtxt(f, delimiter=",", skip_header=2)
    for c in (4, 6, 8):
        hkm += float(np.hypot(np.diff(d[:, c]), np.diff(d[:, c + 1])).sum())
hkm /= 1000.0
rm = sum(r["D"]["dist"] for r in rows)
hh = sum(r["D"]["dur"] for r in rows) / 3600

fig = plt.figure(figsize=(12.8, 11.8), dpi=170)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(2, 2, hspace=0.30, wspace=0.24,
                       left=0.07, right=0.965, top=0.875, bottom=0.07)
axA = fig.add_subplot(gs[0, 0]); axB = fig.add_subplot(gs[0, 1])
axC = fig.add_subplot(gs[1, 0]); axD = fig.add_subplot(gs[1, 1])
for a in (axA, axB, axC, axD):
    a.set_facecolor(SURFACE)
    a.grid(True, color=GRID, lw=0.7)
    a.tick_params(colors=MUTED, labelsize=9)
    for s in a.spines.values():
        s.set_color(GRID)

fig.suptitle("Phase 2 final — 25 paired worlds, full 20-min runs: cost of dynamics ≈ 0",
             color=INK, fontsize=15, x=0.07, ha="left", y=0.972)
fig.text(0.07, 0.940,
         f"v2.9.1 unmodified · static vs walking humans (1-3 per world, 0.7-1.2 m/s) · "
         f"dynamic exposure {hh:.2f} h, robot {rm:.0f} m, humans {hkm:.1f} km · "
         f"0 deadlocks · 0 near-contacts <0.35 m",
         color=INK2, fontsize=10.5)
fig.text(0.07, 0.918,
         "Overnight batch with the v3 spread-based watchdog: 25/25 runs completed, zero kills "
         "(v2 had falsely killed 16/25 healthy runs the day before).",
         color=INK2, fontsize=10.5, style="italic")

# ---------- A: parity ----------
for r in rows:
    axA.plot(r["S"]["dist"] / r["S"]["dur"] * 60, r["D"]["dist"] / r["D"]["dur"] * 60,
             "o", ms=8, mfc=BLUE, mec=SURFACE, mew=1.0, alpha=0.9, zorder=4)
lo, hi = 11.4, 16.2
axA.plot([lo, hi], [lo, hi], color=MUTED, lw=1.0, ls="--", zorder=2)
axA.text(hi - 0.15, hi - 0.45, "y = x", color=MUTED, fontsize=9, ha="right")
axA.set_xlim(lo, hi); axA.set_ylim(lo, hi); axA.set_aspect("equal")
axA.set_xlabel("static — speed made good (m/min)", color=INK2, fontsize=10)
axA.set_ylabel("dynamic — speed made good (m/min)", color=INK2, fontsize=10)
axA.set_title("A · Locomotion parity, seed by seed", color=INK, fontsize=12, loc="left")
sm_ = np.median([r["S"]["dist"] / r["S"]["dur"] * 60 for r in rows])
dm_ = np.median([r["D"]["dist"] / r["D"]["dur"] * 60 for r in rows])
axA.text(0.04, 0.96, f"median {sm_:.1f} → {dm_:.1f} m/min", transform=axA.transAxes,
         color=INK, fontsize=10, va="top",
         bbox=dict(boxstyle="round,pad=0.3", fc=SURFACE, ec=GRID))

# ---------- B: social clearance ----------
axB.axhspan(0.0, 0.35, color="#fdeaea", zorder=1)
axB.axhspan(0.35, 0.5, color="#fdf3ea", zorder=1)
axB.axhline(0.5, color=MUTED, lw=0.9, ls="--")
axB.axhline(1.2, color=MUTED, lw=0.9, ls=":")
for i, r in enumerate(rows):
    axB.plot(i + 1, r["D"]["min_clear"], "o", ms=8, mfc=BLUE, mec=SURFACE, mew=1.0, zorder=4)
axB.text(24.8, 1.185, "personal 1.2 m", color=MUTED, fontsize=8.5, ha="right", va="top")
axB.text(24.8, 0.51, "intimate 0.5 m", color=MUTED, fontsize=8.5, ha="right", va="bottom")
axB.text(24.8, 0.015, "near-contact <0.35 m: never entered", color=RED, fontsize=8.5,
         ha="right", va="bottom")
axB.set_xlim(0, 26); axB.set_ylim(0, 1.3)
axB.set_xlabel("world (seed 20260723001…025)", color=INK2, fontsize=10)
axB.set_ylabel("min robot-human clearance (m)", color=INK2, fontsize=10)
axB.set_title("B · Social clearance per run", color=INK, fontsize=12, loc="left")
axB.text(0.04, 0.96, "time <1.2 m: 4.1% · <0.5 m: 0.4% · braking floor ≈ 0.38-0.45 m",
         transform=axB.transAxes, color=INK, fontsize=10, va="top",
         bbox=dict(boxstyle="round,pad=0.3", fc=SURFACE, ec=GRID))

# ---------- C: crossings by door width ----------
bins = [(0.50, 0.60), (0.60, 0.66), (0.66, 0.72), (0.72, 0.80), (0.80, 0.95)]
labels, s_vals, d_vals, ns = [], [], [], []
for lo_, hi_ in bins:
    s_cr = d_cr = nd = 0
    for r in rows:
        for name, g in r["doors"]:
            if lo_ <= g < hi_:
                nd += 1
                s_cr += r["S"]["cross_per_door"][name]
                d_cr += r["D"]["cross_per_door"][name]
    labels.append(f"{lo_:.2f}-{hi_:.2f}\n({nd} doors)")
    s_vals.append(s_cr); d_vals.append(d_cr); ns.append(nd)
xx = np.arange(len(bins))
axC.bar(xx - 0.19, s_vals, width=0.36, color=BASE, zorder=3, label="static (36 total)")
axC.bar(xx + 0.19, d_vals, width=0.36, color=BLUE, zorder=3, label="dynamic (44 total)")
for i, (sv, dv) in enumerate(zip(s_vals, d_vals)):
    axC.text(i - 0.19, sv + 0.35, str(sv), ha="center", color=INK2, fontsize=9)
    axC.text(i + 0.19, dv + 0.35, str(dv), ha="center", color=INK, fontsize=9)
axC.set_xticks(xx, labels, fontsize=8.5)
axC.set_ylim(0, 26)
axC.set_ylabel("door crossings (25 worlds)", color=INK2, fontsize=10)
axC.set_xlabel("door width (m) — robot needs ≈0.66 m", color=INK2, fontsize=10)
axC.set_title("C · Door crossings by width: no collapse with humans", color=INK, fontsize=12, loc="left")
axC.legend(loc="upper left", fontsize=9, frameon=True, facecolor=SURFACE, edgecolor=GRID)
axC.text(0.03, 0.70, "7.9 → 10.4 crossings/h", transform=axC.transAxes, color=INK,
         fontsize=10, bbox=dict(boxstyle="round,pad=0.3", fc=SURFACE, ec=GRID))

# ---------- D: watchdog field test ----------
axD.bar([0], [16], width=0.5, color=ORANGE, zorder=3)
axD.bar([1], [0], width=0.5, color=BLUE, zorder=3)
axD.plot(1, 0.25, "v", ms=10, mfc=BLUE, mec=SURFACE, zorder=5)
axD.text(0, 16.5, "16 / 25", ha="center", color=INK, fontsize=13, fontweight="bold")
axD.text(1, 1.0, "0 / 25", ha="center", color=INK, fontsize=13, fontweight="bold")
axD.set_xticks([0, 1], ["v2 watchdog\n(two-point net)\ndyn batch 24/7", "v3 watchdog\n(trail spread, sim-gated)\ndyn batch 24-25/7"],
               fontsize=9)
axD.set_ylim(0, 19)
axD.set_ylabel("runs killed as \"terminal deadlock\"", color=INK2, fontsize=10)
axD.set_title("D · Watchdog field test: false kills 16 → 0", color=INK, fontsize=12, loc="left")
axD.text(0.5, 10.5, "all 16 v2 kills proven false by forensics:\n"
                    "robot walked 19-26 m inside the very\nwindow read as \"net <0.5 m\"\n\n"
                    "v3: bounding-box spread of the full trail\n+ samples only while sim advances\n"
                    "→ 25/25 full runs, zero kills",
         ha="center", color=INK2, fontsize=9.5, va="center",
         bbox=dict(boxstyle="round,pad=0.5", fc=SURFACE, ec=GRID))

fig.text(0.07, 0.013,
         "Deadlock rule (offline): excursion <0.5 m over 90 sim-s — 0 events in all 50 paired runs · dance episodes 5 vs 2 "
         "(all transient) · overnight RTF stable 0.49-0.55, no degradation — the morning desktop lag never reached the sim loop.",
         color=MUTED, fontsize=8.5)
fig.savefig("/home/user/isaac/phase2_final_dynamic_vs_static.png", facecolor=PAGE,
            bbox_inches="tight")
print(f"humans walked {hkm:.1f} km, robot {rm:.0f} m in {hh:.2f} h")
print("saved phase2_final_dynamic_vs_static.png")
