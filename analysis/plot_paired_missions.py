#!/usr/bin/env python3
# Experiments 4 and 5: paired mission matrices, AMCL v8 (baseline) vs v9 (sparse-map tuning).
# 25 worlds x 4 goals, every reported arrival checked against simulator ground truth.
import os
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")   # repo-relative data directory
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.patches import Rectangle, Patch

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"
BASE = "#c3c2b7"
BLUE = "#2a78d6"; ORANGE = "#eb6834"; RED = "#e34948"; AQUA = "#1baf7a"

A = {r["seed"]: r for r in json.load(open(RUNS_DIR + "/missions_results_v8.json"))}
B = {r["seed"]: r for r in json.load(open(RUNS_DIR + "/missions_results_v9.json"))}
PRED = {w["seed"]: w["goals"] for w in json.load(open(RUNS_DIR + "/missions_feasibility.json"))}
MISSIONS = ["adjacent-x", "adjacent-y", "diagonal", "home"]
CLS_ORDER = {"CAGE": 0, "PARTIAL": 1, "FALSE_ARRIVAL": 2, "WEDGED": 3, "FAIL": 4, "UNVER": 5}
seeds = sorted(A, key=lambda s: (CLS_ORDER.get(A[s]["status"], 9), -(A[s]["coverage"] or 0)))
LABEL = {"CAGE": "cage", "PARTIAL": "partial", "FALSE_ARRIVAL": "false", "WEDGED": "wedged",
         "FAIL": "fail", "UNVER": "excl."}

def tally(D):
    real = sum(r["real_succ"] for r in D.values()); fake = sum(r["fake_succ"] for r in D.values())
    home = sum(1 for r in D.values() if r["final"].get("home") == "SUCCEEDED"
               and r["true_dist"].get("home") is not None and r["true_dist"]["home"] <= 1.5)
    far = max([d for r in D.values() for m, d in r["true_dist"].items()
               if r["final"].get(m) == "SUCCEEDED" and d > 1.5] or [0])
    cls = {}
    for r in D.values(): cls[r["status"]] = cls.get(r["status"], 0) + 1
    return real, fake, real - home, far, cls

rA, fA, xA, farA, cA = tally(A); rB, fB, xB, farB, cB = tally(B)
feas = sum(g["map_ok"] for w in PRED.values() for m, g in w.items() if m != "home") + len(PRED)
cages = [s for s in seeds if A[s]["status"] == "CAGE"]
cage_refused = sum(1 for s in cages for D in (A, B)
                   if all(D[s]["final"].get(m) != "SUCCEEDED" for m in MISSIONS if m != "home"))
excluded = [s for s in seeds if B[s]["status"] == "UNVER"]

fig = plt.figure(figsize=(12.6, 12.0), dpi=150)
fig.patch.set_facecolor(PAGE)
gs = gridspec.GridSpec(2, 2, height_ratios=[10.2, 1.35], hspace=0.10, wspace=0.30,
                       left=0.115, right=0.965, top=0.838, bottom=0.045)

fig.suptitle("Experiments 4 and 5: missions on self-built maps", color=INK, fontsize=17.5,
             x=0.115, ha="left", y=0.965)
fig.text(0.115, 0.932,
         "25 generated worlds · Nav2 tours of 4 goals on maps the robot built itself · every reported arrival\n"
         "checked against simulator ground truth · left: baseline AMCL (v8) · right: sparse-map AMCL (v9), same worlds",
         color=INK2, fontsize=10.5, va="top")
fig.text(0.115, 0.888,
         f"offline reachability check: {feas}/100 goals feasible on these maps · real arrivals: {rA} (v8) and {rB} (v9)",
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
    ax.set_title(title, color=INK, fontsize=12, loc="left", pad=24)
    ax.text(0.0, -0.02, note, transform=ax.transAxes, color=INK2, fontsize=8.6,
            va="top")
    for j, m in enumerate(MISSIONS):
        ax.text(j + 0.5, -0.55, m.replace("adjacent-", "adj-"), ha="center",
                color=INK2, fontsize=8.6, clip_on=False)
    for i, s in enumerate(seeds):
        r = data[s]
        unver = r["status"] == "UNVER"
        for j, m in enumerate(MISSIONS):
            res = r["final"].get(m, "")
            td = r["true_dist"].get(m)
            fake = (not unver and res == "SUCCEEDED" and (td is None or float(td) > 1.5))
            fc = SURFACE if unver else result_color(res, fake)
            cell = Rectangle((j + 0.06, i + 0.08), 0.88, 0.84, facecolor=fc,
                             edgecolor=MUTED if unver else SURFACE, lw=1.0 if unver else 1.4,
                             hatch="////" if unver else None, joinstyle="round")
            ax.add_patch(cell)
            if m != "home" and PRED.get(s, {}).get(m) and not PRED[s][m]["map_ok"]:
                ax.add_patch(Rectangle((j + 0.06, i + 0.08), 0.88, 0.84, fill=False,
                                       edgecolor=INK, lw=1.1, ls=(0, (2, 2))))
            if unver:
                pass
            elif res == "SUCCEEDED" and not fake:
                ax.text(j + 0.5, i + 0.54, "✓", ha="center", va="center",
                        color="white", fontsize=9, fontweight="bold")
            elif fake and res == "SUCCEEDED":
                ax.text(j + 0.5, i + 0.54, "!", ha="center", va="center",
                        color="white", fontsize=9, fontweight="bold")
        ax.text(4.14, i + 0.54, LABEL.get(r["status"], r["status"].lower()), va="center", color=MUTED, fontsize=7.4)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)

axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])
draw_panel(axA, A, "AMCL v8 (baseline parameters)",
           f"{rA} real ({xA} cross-room) · {fA} false arrivals (robot up to {farA:.1f} m from the goal)")
draw_panel(axB, B, "AMCL v9 (sparse-map tuning)",
           f"{rB} real ({xB} cross-room) · {fB} false arrivals · world {', '.join(s[-3:] for s in excluded)} excluded")
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
                 label="offline check: infeasible on this map"),
           Patch(facecolor=SURFACE, edgecolor=MUTED, hatch="////",
                 label="excluded: no matching ground-truth log")]
axL.legend(handles=handles, loc="upper left", ncol=3, frameon=False, fontsize=8.8,
           handlelength=1.4, columnspacing=1.6, bbox_to_anchor=(0.0, 0.92))
axL.text(0.0, 0.30,
         f"The {len(cages)} worlds the offline check calls cages ({', '.join('…' + s[-3:] for s in cages)}) refused every cross-room goal "
         f"in both configurations ({cage_refused} of {2 * len(cages)} tours).\n"
         f"Row labels: world class. Among the checked tours, tuning removed the false-arrival worlds "
         f"({cA.get('FALSE_ARRIVAL', 0)} → {cB.get('FALSE_ARRIVAL', 0)}), "
         f"the wedged worlds went {cA.get('WEDGED', 0)} → {cB.get('WEDGED', 0)}, and real arrivals moved from {rA} to {rB}.",
         color=INK2, fontsize=9.2, va="top")
fig.savefig(RUNS_DIR + "/phase3_localization_tax.png", facecolor=PAGE,
            bbox_inches="tight")
print("saved phase3_localization_tax.png")
