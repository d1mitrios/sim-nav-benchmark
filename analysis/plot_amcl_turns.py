#!/usr/bin/env python3
# Experiments 6 and 8: AMCL position error over one tour, v9 and the revised configuration, same world,
# ground-truth map. Grey bands mark in-place turns (wheel axle moving less than 2 cm/s while the
# heading changes faster than 0.1 rad/s, from the simulator's true pose). v9 runs AMCL on base_link,
# 0.10 m ahead of the axle, so every in-place turn looks like a sideways step to its motion model.
# The table on the right counts, over all worlds of each arm, the AMCL updates whose error grew by
# more than 0.5 m and how many of them followed an in-place turn (axle moved < 5 cm and heading
# changed > 0.1 rad since the previous update).
# Usage: python3 plot_amcl_turns.py [seed]   (default 20260723003). Writes runs/amcl_turns.png.
import os
import sys
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_missions2 as am
from stats_missions import ARMS

SURFACE = "#fcfcfb"; PAGE = "#f9f9f7"
INK = "#0b0b0b"; INK2 = "#52514e"; MUTED = "#898781"; GRID = "#e1e0d9"; BASE = "#c3c2b7"
BLUE = "#2a78d6"; ORANGE = "#eb6834"
AXLE = 0.10          # wheel axle behind base_link (m)
JUMP = 0.5           # error increase between two updates counted as a jump (m)
SEED = sys.argv[1] if len(sys.argv) > 1 else "20260723003"


def load(seed, mfile, cfg, md5):
    rf = am.run_for(seed, am.ts_of(mfile))
    d = np.genfromtxt(rf, delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    th = np.unwrap(np.radians(d[:, 3]))
    ax_, ay_ = x - AXLE * np.cos(th), y - AXLE * np.sin(th)
    off = am.amcl_offset(cfg, md5 or "")
    rows = [r for r in csv.DictReader(open(mfile.replace("missions_", "amcl_"))) if r["sim_t"]]
    ts = np.array([float(r["sim_t"]) for r in rows])
    err = []
    for r in rows:
        _, tx, ty = am.truth_at(d, float(r["sim_t"]))
        yaw = math.radians(float(r["yaw_deg"]))
        err.append(math.hypot(float(r["x"]) + off * math.cos(yaw) - tx,
                              float(r["y"]) + off * math.sin(yaw) - ty))
    idx = np.clip(np.searchsorted(t, ts), 0, len(t) - 1)
    after_turn = np.zeros(len(ts), bool)
    for k in range(1, len(ts)):
        a, b = idx[k - 1], idx[k]
        after_turn[k] = (math.hypot(ax_[b] - ax_[a], ay_[b] - ay_[a]) < 0.05 and abs(th[b] - th[a]) > 0.1)
    # continuous in-place turn intervals
    dt = np.gradient(t)
    dt[dt <= 0] = 1e-3
    v = np.hypot(np.gradient(ax_), np.gradient(ay_)) / dt
    w = np.abs(np.gradient(th)) / dt
    k = 5
    v = np.convolve(v, np.ones(k) / k, "same")
    w = np.convolve(w, np.ones(k) / k, "same")
    turning = (v < 0.02) & (w > 0.1)
    rows_m = am.read_missions(mfile)
    t0, t1 = float(rows_m[0]["sim_start"]), float(rows_m[-1]["sim_end"])
    return dict(t=t, turning=turning, ts=ts, err=np.array(err), after_turn=after_turn, t0=t0, t1=t1)


def arm_files(arm):
    cfg, mk, d0, d1, md5 = ARMS[arm]
    return cfg, md5, am.pick_files(cfg, mk, d0, d1, set(am.BENCH_SEEDS), md5)


def tally(arm):
    cfg, md5, files = arm_files(arm)
    nu = jumps = jt = 0
    for seed, f in sorted(files.items()):
        try:
            r = load(seed, f, cfg, md5)
        except (OSError, ValueError, TypeError):
            continue
        j = np.diff(r["err"]) > JUMP
        nu += len(r["err"]) - 1
        jumps += int(j.sum())
        jt += int((j & r["after_turn"][1:]).sum())
    return len(files), nu, jumps, jt


fig = plt.figure(figsize=(13.2, 7.0), dpi=150)
fig.patch.set_facecolor(PAGE)
fig.text(0.05, 0.955, "Localization error and in-place turns", color=INK, fontsize=16.5, va="top")
fig.text(0.05, 0.895,
         f"World {SEED[-3:]}, ground-truth map · AMCL estimate against the simulator's true position · "
         "grey bands: the robot turning in place\nv9 tracks base_link, 0.10 m ahead of the wheel axle, "
         "the revised configuration tracks the axle (base_axle)",
         color=INK2, fontsize=10, va="top")
panels = [("v9_truth", "v9 configuration", ORANGE), ("rev_truth", "revised configuration", BLUE)]
data = {}
for arm, _, _ in panels:
    cfg, md5, files = arm_files(arm)
    if SEED in files:
        data[arm] = load(SEED, files[SEED], cfg, md5)
ymax = max([float(np.max(r["err"])) for r in data.values()] + [1.0]) * 1.08
tmax = max([r["t1"] - r["t0"] for r in data.values()] + [1.0])
for pi, (arm, title, col) in enumerate(panels):
    ax = fig.add_axes([0.05, 0.49 - pi * 0.38, 0.60, 0.27])
    ax.set_facecolor(SURFACE)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(colors=MUTED, labelsize=9, length=0)
    ax.grid(True, axis="y", color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    ax.set_xlim(0, tmax)
    ax.set_ylim(0, ymax)
    ax.set_ylabel("error (m)", color=INK2, fontsize=10)
    ax.set_title(title, color=INK, fontsize=11.5, loc="left", pad=6)
    r = data.get(arm)
    if r is None:
        continue
    t = r["t"] - r["t0"]
    m = r["turning"]
    edges = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
    for a, b in zip(edges[::2], edges[1::2]):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color=BASE, alpha=0.45, lw=0, zorder=0)
    ax.plot(r["ts"] - r["t0"], r["err"], color=col, lw=2.0, zorder=3, drawstyle="steps-post")
    ax.axhline(1.0, color=MUTED, lw=1.0, ls=(0, (3, 3)), zorder=2)
    ax.text(tmax * 0.995, 1.0, "1 m", color=MUTED, fontsize=8.6, ha="right", va="bottom")
    ax.text(0.995, 0.95, f"median {np.median(r['err']):.2f} m · max {np.max(r['err']):.1f} m",
            transform=ax.transAxes, color=INK2, fontsize=9, ha="right", va="top", zorder=5,
            bbox=dict(facecolor=SURFACE, edgecolor="none", pad=1.5))
    if pi == 1:
        ax.set_xlabel("simulated time since the tour started (s)", color=INK2, fontsize=10)

# table: error jumps over all worlds
tx = fig.add_axes([0.69, 0.11, 0.27, 0.66])
tx.axis("off")
tx.set_title(f"AMCL updates whose error grew\nby more than {JUMP} m, all worlds", color=INK, fontsize=11.5,
             loc="left", pad=6)
hdr = ["arm", "updates", "jumps", "after\na turn"]
cols = [0.0, 0.66, 0.84, 1.06]
yy = 0.92
for c, h in zip(cols, hdr):
    tx.text(c, yy, h, color=MUTED, fontsize=9, ha="left" if c == 0 else "right", va="bottom",
            transform=tx.transAxes)
for arm, label in [("v9_truth", "v9, truth map"), ("rev_truth", "revised, truth map"),
                   ("v9_slam", "v9, SLAM map"), ("rev_slam", "revised, SLAM map")]:
    nw, nu, jumps, jt = tally(arm)
    yy -= 0.09
    share = f"{jt} ({100 * jt / jumps:.0f}%)" if jumps else "0"
    for c, v in zip(cols, [label, f"{nu}", f"{jumps}", share]):
        tx.text(c if c == 0 else c, yy, v, color=INK if c == 0 else INK2, fontsize=9.5,
                ha="left" if c == 0 else "right", transform=tx.transAxes)
    tx.text(0.0, yy - 0.035, f"{nw} worlds", color=MUTED, fontsize=8, transform=tx.transAxes)
tx.text(0.0, yy - 0.16,
        "a turn: since the previous update the axle\nmoved less than 5 cm and the heading changed\n"
        "by more than 0.1 rad", color=MUTED, fontsize=8.6, transform=tx.transAxes, va="top")
out = os.path.join(os.path.dirname(HERE), "runs", "amcl_turns.png")
fig.savefig(out, facecolor=PAGE)
print("figure ->", out)
