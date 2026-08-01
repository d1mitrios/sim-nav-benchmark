#!/usr/bin/env python3
# === Final aggregator of the mission batch (Step 2) ===
# Per seed: last missions CSV (30/7) + prediction (missions_feasibility.json)
# + ground-truth join with the metrics CSV (where it has been staged) ->
#  - verified result per mission (SUCCEEDED with true arrival <1.5 m = real;
#    otherwise a FAKE success = localization delirium)
#  - world classification: OK / CAGE (correct refusals as predicted) /
#    DELIRIUM (amcl collapse) / WEDGED (GT frozen with active goals) / SKIP
#  - aggregate: prediction-vs-practice, success rates, times, retry saves
# Works with whatever data exists (partial execution = partial table).
import glob
import json
import math
import time
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
PRED = {w["seed"]: w for w in json.load(open("/home/user/isaac/missions_feasibility.json"))}
ROWS = {r["seed"]: r for r in json.load(open("/home/user/isaac/slam_batch_rows.json"))}
SEEDS = [f"202607230{i:02d}" for i in range(1, 26)]
ARRIVE_M = 1.5   # true-arrival threshold (join noise ~0.5-1.0 m from the linear RTF approximation)


def _ts(f):
    return f.rsplit("_", 2)[-2] + f.rsplit("_", 2)[-1].replace(".csv", "")


import sys
ERA = sys.argv[1] if len(sys.argv) > 1 else "A"
ERAS = {"A": ("20260730123000", "20260730215959"),   # v8 batch 30/7 (afternoon + 017 evening)
        "B": ("20260731000000", "20260731235959")}   # v9 sparse-amcl batch 31/7
E0, E1 = ERAS[ERA]


def pick_missions(seed):
    """Era file: A = first of 30/7 >=12:30 (smoke excluded, afternoon preferred,
    017 evening); B = 31/7 (v9)."""
    fs = sorted(glob.glob(f"{RUNS}/missions_{seed}_2026073*_*.csv"))
    fs = [f for f in fs if E0 <= _ts(f) <= E1]
    return fs[0] if fs else None


def pick_run(seed, mts):
    """Run CSV of the same world: the largest ts <= the missions file's ts."""
    fs = sorted(glob.glob(f"{RUNS}/run_{seed}_2026073*_*.csv"))
    fs = [f for f in fs if _ts(f) <= mts]
    return fs[-1] if fs else None


def load_metrics(seed, mts):
    f = pick_run(seed, mts)
    if not f:
        return None
    hdr = open(f).readline()
    ws = hdr.split("wall_start=")[1].split(" fmt")[0]
    t0 = time.mktime(time.strptime(ws, "%Y-%m-%d %H:%M:%S"))
    d = np.genfromtxt(f, delimiter=",", skip_header=2)
    if d.ndim != 2 or len(d) < 10:
        return None
    return t0, d


def pick_offset(t0, sim_end, e_last):
    for off in (0, -3600, -7200, -10800, -14400, 3600):
        wall = e_last - (t0 + off)
        if wall > 0 and 0.12 <= sim_end / wall <= 0.95:
            return off, sim_end / wall
    return None, None


def analyze_seed(seed):
    mf = pick_missions(seed)
    if not mf:
        return {"seed": seed, "status": "NO_DATA"}
    rows = [l.strip().split(",") for l in open(mf).readlines()[1:]]
    met = load_metrics(seed, _ts(mf))
    join = None
    if met:
        t0, d = met
        off, rtf = pick_offset(t0, d[-1, 0], float(rows[-1][7]))
        if off is not None:
            join = (t0, d, off, rtf)
    pred = PRED.get(seed, {}).get("goals", {})
    # final result per mission (last attempt) + true dist where possible
    out = {}
    truedist = {}
    frozen_pts = []
    for r in rows:
        nm, gx, gy, att, res = r[0], float(r[1]), float(r[2]), int(r[3]), r[4]
        e1 = float(r[7])
        out[nm] = res  # last attempt wins
        if join:
            t0, d, off, rtf = join
            s1 = (e1 - (t0 + off)) * rtf
            i = min(max(np.searchsorted(d[:, 0], s1), 0), len(d) - 1)
            x, y = d[i, 1], d[i, 2]
            truedist[nm] = math.hypot(x - gx, y - gy)
            frozen_pts.append((x, y, res))
    # verified successes
    n_succ = sum(1 for v in out.values() if v == "SUCCEEDED")
    if truedist:
        n_real = sum(1 for nm, v in out.items()
                     if v == "SUCCEEDED" and truedist.get(nm, 9e9) <= ARRIVE_M)
        n_fake = n_succ - n_real
    else:
        n_real, n_fake = 0, 0   # UNVERIFIED — nothing is credited without ground truth
    # prediction-vs-practice (cross-room only — home is always map-feasible)
    agree = tot = 0
    for nm, v in out.items():
        p = pred.get(nm)
        if not p or nm == "home":
            continue
        tot += 1
        lived = (v == "SUCCEEDED" and truedist.get(nm, 0) <= ARRIVE_M)
        if p["map_ok"] == lived or (not p["map_ok"] and v in ("ABORTED", "REJECTED")):
            agree += 1
    # world classification
    tl = [t for t in (truedist.get(n) for n in out) if t is not None]
    res_list = list(out.values())
    if n_fake > 0:
        cls = "DELIRIUM"
    elif join and len(frozen_pts) >= 4:
        xs = [p[0] for p in frozen_pts[-4:]]
        ys = [p[1] for p in frozen_pts[-4:]]
        spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        stuck_res = all(r in ("TIMEOUT", "ABORTED") for _, _, r in frozen_pts[-4:])
        if spread < 0.6 and stuck_res and "TIMEOUT" in res_list:
            cls = "WEDGED"
        elif pred and sum(p["map_ok"] for n, p in pred.items() if n != "home") <= 1 \
                and res_list.count("ABORTED") + res_list.count("REJECTED") >= len(res_list) - 1:
            cls = "CAGE"
        elif n_real >= 3:
            cls = "OK"
        elif n_real >= 1:
            cls = "PARTIAL"
        else:
            cls = "FAIL"
    elif not join:
        cls = "UNVER"
    else:
        cls = "?"
    surf = ROWS.get(seed, {}).get("surf")
    return {"seed": seed, "status": cls, "final": out, "true_dist": truedist,
            "real_succ": n_real, "fake_succ": n_fake,
            "pred_agree": f"{agree}/{tot}" if tot else "-",
            "coverage": round(surf * 100, 1) if surf else None}


def main():
    print(f"{'seed':13s} {'class':9s} {'succ(real)':>10s} {'fake':>4s} "
          f"{'pred✓':>6s} {'cov%':>5s}  missions")
    agg = {"real": 0, "fake": 0, "worlds": 0, "by_cls": {}}
    coverage_pairs = []
    for seed in SEEDS:
        r = analyze_seed(seed)
        if r["status"] == "NO_DATA":
            print(f"{seed:13s} {'—':9s}")
            continue
        agg["worlds"] += 1
        agg["real"] += r["real_succ"]
        agg["fake"] += r["fake_succ"]
        agg["by_cls"][r["status"]] = agg["by_cls"].get(r["status"], 0) + 1
        if r["coverage"] is not None:
            coverage_pairs.append((r["coverage"], r["real_succ"]))
        ms = " ".join(f"{n}:{v[:4]}{'*' if r['true_dist'].get(n, 9) > ARRIVE_M and v=='SUCCEEDED' else ''}"
                      for n, v in r["final"].items())
        print(f"{r['seed']:13s} {r['status']:9s} {r['real_succ']:>10d} {r['fake_succ']:>4d} "
              f"{r['pred_agree']:>6s} {str(r['coverage'] or ''):>5s}  {ms}")
    print(f"\nWorlds: {agg['worlds']}  |  real successes: {agg['real']}  fake: {agg['fake']}")
    print("Classification:", agg["by_cls"])
    if len(coverage_pairs) >= 5:
        c = np.array(coverage_pairs)
        r = np.corrcoef(c[:, 0], c[:, 1])[0, 1]
        print(f"coverage vs real-successes: r = {r:.2f} (n={len(c)})")


if __name__ == "__main__":
    main()
