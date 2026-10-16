#!/usr/bin/env python3
# Statistics of the October mission experiments (6, 7, 8 and 5b), the 2 x 2 design
# {v9, revised} x {ground-truth map, SLAM map} on the same 25 worlds, plus experiment 9.
# Usage: python3 stats_missions.py
# The world is the unit: the four goals of a tour are not independent (a failed goal leaves the
# robot somewhere else for the next one). For every arm the script reports real arrivals (true
# position within 0.6 m of the goal) overall, for the 75 cross-room goals and the 25 home goals,
# with a 95% interval from a bootstrap over worlds, and the share of feasible goals reached at
# one fixed planner radius (0.25 m, the revised configuration's). Paired comparisons use the
# per-world difference in real arrivals: an exact two-sided sign test, the Wilcoxon signed-rank
# test and a bootstrap interval of the mean difference. Only worlds present in both arms count.
# Writes runs/missions_stats.json.
import os
import sys
import json
import numpy as np
from scipy.stats import binomtest, wilcoxon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_missions2 as am

ARMS = {   # name: (config, map kind, first date, last date, params md5 or None)
    "v9_truth": ("nav2_params_v9", "truth", "20261012", "20261012", None),
    "v9_slam": ("nav2_params_v9", "slam", "20261014", None, "b7f0bb38"),
    "rev_truth": ("nav2_params", "truth", "20261013", "20261014", "9f696af1"),
    "rev_slam": ("nav2_params", "slam", "20261014", None, "9f696af1"),
    # experiment 9 (post hoc, stopped by choice after 3 complete worlds): revised + learning global costmap
    "exp9_slam": ("nav2_params_exp9", "slam", "20261015", None, "13931c6d"),
}
PAIRS = [("rev_truth", "v9_truth", "configuration, ground-truth maps"),
         ("rev_slam", "v9_slam", "configuration, SLAM maps"),
         ("rev_truth", "rev_slam", "map, revised configuration"),
         ("v9_truth", "v9_slam", "map, v9"),
         ("exp9_slam", "rev_slam", "experiment 9 vs revised, SLAM maps")]
FIXED_R = 0.25
B = 10000
RNG = np.random.default_rng(20261014)


def run_arm(config, mapkind, d0, d1, md5, seeds=None):
    files = am.pick_files(config, mapkind, d0, d1, set(seeds or am.BENCH_SEEDS), md5)
    if not files:
        return {}
    versions = sorted({am.read_missions(f)[0].get("params_md5", "") for f in files.values()})
    m = md5 or (versions[0] if len(versions) == 1 else "")
    radius, offset = am.planner_radius(config, m), am.amcl_offset(config, m)
    out = {}
    for seed, f in sorted(files.items()):
        r = am.analyze(seed, f, mapkind, radius, offset)
        if r["status"] == "UNVER":
            continue
        tour = [(n, None, None) for n in r["final"]]
        rows = am.read_missions(f)
        goals = {}
        for row in rows:
            goals.setdefault(row["mission"], (float(row["goal_x"]), float(row["goal_y"])))
        r["feasible_fixed"] = am.feasibility(seed, mapkind, FIXED_R,
                                             [(n, gx, gy) for n, (gx, gy) in goals.items()])
        r["real06"] = {n: (v == "SUCCEEDED" and r["true_dist"][n] <= am.STRICT_M)
                       for n, v in r["final"].items()}
        out[seed] = r
    return out


def boot_ci(per_world, stat=np.mean):
    a = np.asarray(per_world, float)
    if len(a) == 0:
        return (float("nan"), float("nan"))
    idx = RNG.integers(0, len(a), (B, len(a)))
    s = np.array([stat(a[i]) for i in idx])
    return float(np.percentile(s, 2.5)), float(np.percentile(s, 97.5))


def summarize(name, res):
    w = sorted(res)
    real = np.array([sum(res[s]["real06"].values()) for s in w])
    cross = np.array([sum(v for n, v in res[s]["real06"].items() if n != "home") for s in w])
    home = np.array([int(res[s]["real06"].get("home", False)) for s in w])
    feas = np.array([sum(res[s]["feasible_fixed"].values()) for s in w])
    feas_real = np.array([sum(res[s]["real06"][n] for n, ok in res[s]["feasible_fixed"].items() if ok)
                          for s in w])
    fake = sum(res[s]["fake_succ"] for s in w)
    n = len(w)
    lo, hi = boot_ci(real / 4.0)
    out = dict(worlds=n, real=int(real.sum()), goals=4 * n, rate=float(real.sum() / (4 * n)) if n else None,
               rate_ci=[lo, hi], cross=int(cross.sum()), cross_goals=3 * n, home=int(home.sum()),
               feasible_r025=int(feas.sum()), real_of_feasible=int(feas_real.sum()), false=int(fake),
               ok_worlds=int(sum(1 for s in w if res[s]["status"] == "OK")),
               per_world={s: int(v) for s, v in zip(w, real)})
    print(f"{name:10s} worlds {n:2d}  real {out['real']:3d}/{out['goals']:3d} "
          f"({100 * out['rate']:.0f}%, 95% CI {100 * lo:.0f}-{100 * hi:.0f}%)  cross-room "
          f"{out['cross']}/{out['cross_goals']}  home {out['home']}/{n}  of feasible(r 0.25) "
          f"{out['real_of_feasible']}/{out['feasible_r025']}  false {fake}  OK worlds {out['ok_worlds']}")
    return out


def compare(a_name, b_name, label, arms):
    a, b = arms[a_name], arms[b_name]
    common = sorted(set(a) & set(b))
    if not common:
        return None
    d = np.array([sum(a[s]["real06"].values()) - sum(b[s]["real06"].values()) for s in common])
    pos, neg = int((d > 0).sum()), int((d < 0).sum())
    p_sign = binomtest(pos, pos + neg, 0.5).pvalue if pos + neg else 1.0
    try:
        p_w = wilcoxon(d[d != 0]).pvalue if (d != 0).sum() >= 1 else 1.0
    except ValueError:
        p_w = float("nan")
    lo, hi = boot_ci(d)
    # goal-level McNemar (secondary, goals are not independent)
    bc = sum(1 for s in common for n in a[s]["real06"] if a[s]["real06"][n] and not b[s]["real06"].get(n, False))
    cb = sum(1 for s in common for n in a[s]["real06"] if not a[s]["real06"][n] and b[s]["real06"].get(n, False))
    p_mc = binomtest(bc, bc + cb, 0.5).pvalue if bc + cb else 1.0
    print(f"{a_name} vs {b_name} ({label}): {len(common)} worlds, mean diff {d.mean():+.2f} goals/world "
          f"(95% CI {lo:+.2f} to {hi:+.2f}), better in {pos}, worse in {neg}, sign test p = {p_sign:.2g}, "
          f"Wilcoxon p = {p_w:.2g}, goal-level McNemar {bc}:{cb} p = {p_mc:.2g}")
    return dict(worlds=len(common), mean_diff=float(d.mean()), ci=[lo, hi], better=pos, worse=neg,
                p_sign=float(p_sign), p_wilcoxon=float(p_w), mcnemar=[bc, cb], p_mcnemar=float(p_mc))


def main():
    arms, summary = {}, {}
    for name, (cfg, mk, d0, d1, md5) in ARMS.items():
        arms[name] = run_arm(cfg, mk, d0, d1, md5)
    print()
    for name in ARMS:
        if arms[name]:
            summary[name] = summarize(name, arms[name])
    print()
    comps = {}
    for a, b, label in PAIRS:
        c = compare(a, b, label, arms)
        if c:
            comps[f"{a}_vs_{b}"] = c
    json.dump({"arms": summary, "comparisons": comps}, open(os.path.join(am.RUNS, "missions_stats.json"), "w"),
              indent=1)
    print("\nresults -> runs/missions_stats.json")


if __name__ == "__main__":
    main()
