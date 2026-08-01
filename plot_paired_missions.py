#!/usr/bin/env python3
# THE STEP 2 FIGURE: paired mission matrices — amcl v8 (default) vs v9 (sparse-tuned)
# 25 worlds x 4 missions, verified on ground truth. The finding: the tuning does not change
# the real successes (11 -> 12/100) but eliminates the lies (fake 6 -> 0).
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.patches import Rectangle, Patch

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"
BASE = "#c3c2b7"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; RED = "#e34948"; AQUA = "#1baf7a"

A = {r["seed"]: r for r in json.load(open("/home/user/isaac/missions_results_v8.json"))}
B = {r["seed"]: r for r in json.load(open("/home/user/isaac/missions_results_v9.json"))}
PRED = {w["seed"]: w["goals"] for w in json.load(open("/home/user/isaac/missions_feasibility.json"))}
MISSIONS = ["adjacent-x", "adjacent-y", "diagonal", "home"]
CLS_ORDER = {"CAGE": 0, "PARTIAL": 1, "DELIRIUM": 2, "WEDGED": 3, "FAIL": 4, "UNVER": 5}
seeds = sorted(A, key=lambda s: (CLS_ORDER.get(A[s]["status"], 9), -(A[s]["coverage"] or 0)))

fig = plt.figure(figsize=(12.6, 12.0), dpi=150)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(2, 2, height_ratios=[10.2, 1.35], hspace=0.10, wspace=0.30,
                       left=0.115, right=0.965, top=0.838, bottom=0.045)

fig.suptitle("The localization tax, measured twice", color=INK, fontsize=17.5,
             x=0.115, ha="left", y=0.965)
fig.text(0.115, 0.932,
         "25 procedurally generated worlds · Nav2 tours of 4 goals on maps the robot built itself in 20 min · every arrival\n"
         "verified against simulator ground truth · left: default AMCL · right: sparse-map-tuned AMCL (paired, same worlds)",
         color=INK2, fontsize=10.5, va="top")
fig.text(0.115, 0.888,
         "offline topology predictor: 85/100 goals feasible on these maps — lived reality: 11 and 12 real arrivals",
         color=ORANGE, fontsize=10.5, style="italic")

def result_color(res, fake):
    if res == "SUCCEEDED":
        return RED if fake else BLUE
    if res == "TIMEOUT":
        return ORANGE
    return BASE   # ABORTED / REJECTED

def draw_panel(ax, data, title, note):
    ax.set_facecolor(SURFACE)
    ax.set_xlim(0, 4.6)
    ax.set_ylim(-0.4, len(seeds))
    ax.invert_yaxis()
    ax.set_title(title, color=INK, fontsize=12, loc="left", pad=10)
    ax.text(0.0, -0.02, note, transform=ax.transAxes, color=INK2, fontsize=8.6,
            va="top")
    for j, m in enumerate(MISSIONS):
        ax.text(j + 0.5, -0.55, m.replace("adjacent-", "adj-"), ha="center",
                color=INK2, fontsize=8.6, clip_on=False)
    for i, s in enumerate(seeds):
        r = data[s]
        for j, m in enumerate(MISSIONS):
            res = r["final"].get(m, "")
            td = r["true_dist"].get(m)
            fake = (res == "SUCCEEDED" and (td is None or float(td) > 1.5))
            fc = result_color(res, fake)
            cell = Rectangle((j + 0.06, i + 0.08), 0.88, 0.84, facecolor=fc,
                             edgecolor=SURFACE, lw=1.4,
                             joinstyle="round")
            ax.add_patch(cell)
            if m != "home" and PRED.get(s, {}).get(m) and not PRED[s][m]["map_ok"]:
                ax.add_patch(Rectangle((j + 0.06, i + 0.08), 0.88, 0.84, fill=False,
                                       edgecolor=INK, lw=1.1, ls=(0, (2, 2))))
            if res == "SUCCEEDED" and not fake:
                ax.text(j + 0.5, i + 0.54, "✓", ha="center", va="center",
                        color="white", fontsize=9, fontweight="bold")
            elif fake and res == "SUCCEEDED":
                ax.text(j + 0.5, i + 0.54, "!", ha="center", va="center",
                        color="white", fontsize=9, fontweight="bold")
        ax.text(4.18, i + 0.54, r["status"][:6], va="center", color=MUTED, fontsize=7.2)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)

axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])
draw_panel(axA, A, "AMCL v8 — default parameters",
           "11 real · 6 FALSE arrivals (reported SUCCEEDED, robot up to 7.6 m away)")
draw_panel(axB, B, "AMCL v9 — sparse-map tuning",
           "12 real · 0 false arrivals — the estimator stops lying; the map still wins")
for i, s in enumerate(seeds):
    axA.text(-0.12, i + 0.54, f"…{s[-3:]}", ha="right", va="center", color=INK2,
             fontsize=7.4)
    axA.text(-0.85, i + 0.54, f"{A[s]['coverage']:.0f}%", ha="right", va="center",
             color=MUTED, fontsize=7.0)
axA.text(-0.85, -0.55, "map\ncov.", ha="right", color=MUTED, fontsize=7.0, clip_on=False)

# ---- legend + funnel ----
axL = fig.add_subplot(gs[1, :])
axL.set_facecolor(PAGE)
axL.axis("off")
handles = [Patch(facecolor=BLUE, label="real success (ground-truth arrival ≤1.5 m)"),
           Patch(facecolor=RED, label="false success (goal 'reached', robot elsewhere)"),
           Patch(facecolor=ORANGE, label="timeout (4 min, kept trying)"),
           Patch(facecolor=BASE, label="planner refusal / abort"),
           Patch(facecolor=SURFACE, edgecolor=INK, ls=(0, (2, 2)),
                 label="offline predictor: infeasible on this map")]
axL.legend(handles=handles, loc="upper left", ncol=3, frameon=False, fontsize=8.8,
           handlelength=1.4, columnspacing=1.6, bbox_to_anchor=(0.0, 1.28))
axL.text(0.0, 0.14,
         "All four predicted cage-topology worlds (…001, …011, …018, …020) refused every cross-room goal in BOTH configurations — 8/8 "
         "reproductions of the offline prediction.\nTuning verdict: sparse-map AMCL converts silent lies into honest failures "
         "(DELIRIUM 4→0 worlds, WEDGED 4→8) but cannot buy back what the 20-minute map never captured.",
         color=INK2, fontsize=9.2, va="top")
fig.savefig("/home/user/isaac/phase3_localization_tax.png", facecolor=PAGE,
            bbox_inches="tight")
print("saved phase3_localization_tax.png")
