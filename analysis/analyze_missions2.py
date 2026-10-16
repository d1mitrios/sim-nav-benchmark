#!/usr/bin/env python3
# Mission judge for experiments 6 on (the protocol with simulated-time budgets).
# Usage: python3 analyze_missions2.py <config> <slam|truth> [first_date [last_date]]
#            [--seeds=a,b,..|--seeds=all] [--md5=HEX]
#   By default only the 25 benchmark worlds count (the development worlds of the pilots are left
#   out) and every parameter-file version of the configuration is listed. --md5 keeps one version.
#   config is the parameter file's name without .yaml (nav2_params_v9, or nav2_params for the
#   revised configuration),
#   dates are YYYYmmdd and limit the missions files by their start time.
# The mission runner of this protocol writes the simulated start and end time of every attempt
# (read from the metrics CSV that Isaac writes at the same moment), so the join with the ground
# truth is exact and needs no real-time-factor estimate. Per world it reports:
#  - real arrivals: SUCCEEDED and the robot's true position within STRICT_M = 0.6 m of the goal
#    (the goal checker's 0.35 m plus the 0.10 m axle offset plus margin). The join is exact, so
#    the 1.5 m of experiments 4-5, which absorbed the old 0.5-1.0 m join error, is kept only as a
#    second column for comparison with them.
#  - false arrivals: SUCCEEDED but farther than ARRIVE_M
#  - the world class of analyze_missions.py (CAGE, PARTIAL, OK, FALSE_ARRIVAL, WEDGED, FAIL)
#  - feasibility of every goal on the map that Nav2 was given, at the planner radius of the
#    configuration (BFS from the spawn, unknown cells free, as track_unknown_space: false)
#  - the localization error (AMCL pose vs truth at the same simulated time)
#  - the real-time factor of the tour and the longest stretch the robot stood still
#  - for that stretch, how often each stage of the velocity chain asked for motion
# Writes runs/missions_results_<config>_<map>.json.
import os
import sys
import csv
import glob
import json
import math
import time
import numpy as np
from collections import deque
from scipy.ndimage import binary_dilation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate_map as em

ROOT = os.path.dirname(HERE)
RUNS = os.environ.get("RUNS_DIR", os.path.join(ROOT, "runs"))   # override for tests
MAPS_SLAM = RUNS            # copies of the SLAM maps (world_<seed>.yaml), as in experiments 4-5
ARRIVE_M = 1.5
STRICT_M = 0.6
BENCH_SEEDS = [f"202607230{i:02d}" for i in range(1, 26)]
STILL_M = 0.10


def _md5_index():
    import hashlib
    idx = {}
    for f in sorted(glob.glob(os.path.join(ROOT, "ros", "*.yaml"))):
        idx[hashlib.md5(open(f, "rb").read()).hexdigest()[:8]] = f
    return idx


MD5_FILES = _md5_index()
# the md5 of nav2_params_v9.yaml as it ran in experiment 5b (its header comment has changed since)
MD5_FILES.setdefault("b7f0bb38", os.path.join(ROOT, "ros", "nav2_params_v9.yaml"))


def config_params(config, md5=""):
    """Parameters of the file that ran: looked up by its md5 when the rows carry one."""
    import yaml
    path = MD5_FILES.get(md5) or os.path.join(ROOT, "ros", config + ".yaml")
    if md5 and md5 not in MD5_FILES:
        print(f"  WARN: no parameter file with md5 {md5} in ros/, using {os.path.basename(path)}")
    return yaml.safe_load(open(path))


def planner_radius(config, md5=""):
    p = config_params(config, md5)
    return float(p["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"])


def amcl_offset(config, md5=""):
    """Distance from the AMCL base frame forward to the chassis centre, where the ground truth is
    logged: 0.10 m when AMCL tracks the wheel axle (base_axle), 0 for base_link."""
    frame = config_params(config, md5)["amcl"]["ros__parameters"]["base_frame_id"]
    return 0.10 if frame == "base_axle" else 0.0


def ts_of(path):
    b = os.path.basename(path).rsplit(".", 1)[0].split("_")
    return b[-2] + b[-1]


def read_missions(path):
    rows = list(csv.DictReader(open(path)))
    return rows


def pick_files(config, mapkind, d0, d1, seeds, md5=None):
    out, dup = {}, {}
    for f in sorted(glob.glob(f"{RUNS}/missions_*_*_*.csv")):
        seed = os.path.basename(f).split("_")[1]
        if seeds and seed not in seeds:
            continue
        day = ts_of(f)[:8]
        if (d0 and day < d0) or (d1 and day > d1):
            continue
        rows = read_missions(f)
        if not rows or "config" not in rows[0]:
            continue
        if rows[0]["config"] != config or rows[0]["map"] != mapkind:
            continue
        if md5 and rows[0].get("params_md5", "") != md5:
            continue
        if seed in out:
            dup.setdefault(seed, [os.path.basename(out[seed])]).append(os.path.basename(f))
        out[seed] = f   # the newest file of the cohort wins
    for seed, fs in dup.items():
        print(f"  note: {len(fs)} files for {seed}, using the newest ({fs[-1]})")
    return out


def run_for(seed, mts):
    fs = [f for f in sorted(glob.glob(f"{RUNS}/run_{seed}_*.csv")) if ts_of(f) <= mts]
    return fs[-1] if fs else None


def truth_at(d, s):
    i = min(max(np.searchsorted(d[:, 0], s), 0), len(d) - 1)
    return i, d[i, 1], d[i, 2]


def feasibility(seed, mapkind, radius, tour):
    path = f"{RUNS}/truth_{seed}.yaml" if mapkind == "truth" else f"{MAPS_SLAM}/world_{seed}.yaml"
    if not os.path.exists(path):
        return {}
    occ, _, res, origin = em.load_map(path)
    rc = int(round(radius / res))
    yy, xx = np.ogrid[-rc:rc + 1, -rc:rc + 1]
    blocked = binary_dilation(occ, structure=(yy * yy + xx * xx) <= rc * rc)
    h, w = blocked.shape
    cy, cx = int(round(-origin[1] / res)), int(round(-origin[0] / res))
    seen = np.zeros_like(blocked)
    if not blocked[cy, cx]:
        seen[cy, cx] = True
        q = deque([(cy, cx)])
        while q:
            y, x = q.popleft()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and not seen[ny, nx] and not blocked[ny, nx]:
                    seen[ny, nx] = True
                    q.append((ny, nx))
    out = {}
    for nm, gx, gy in tour:
        i, j = int(round((gy - origin[1]) / res)), int(round((gx - origin[0]) / res))
        out[nm] = bool(0 <= i < h and 0 <= j < w and seen[i, j])
    return out


def amcl_error(seed, mfile, d, offset=0.0):
    f = mfile.replace("missions_", "amcl_")
    if not os.path.exists(f):
        return None
    errs = []
    for r in csv.DictReader(open(f)):
        if not r["sim_t"]:
            continue
        _, x, y = truth_at(d, float(r["sim_t"]))
        yaw = math.radians(float(r["yaw_deg"]))
        ax, ay = float(r["x"]) + offset * math.cos(yaw), float(r["y"]) + offset * math.sin(yaw)
        errs.append(math.hypot(ax - x, ay - y))
    return errs or None


def cmd_share(mfile, s0, s1):
    """Share of samples asking for motion, per stage, between simulated times s0 and s1."""
    f = mfile.replace("missions_", "cmdvel_")
    if not os.path.exists(f):
        return None
    tot, mov = {}, {}
    for r in csv.DictReader(open(f)):
        if not r["sim_t"] or not (s0 <= float(r["sim_t"]) <= s1):
            continue
        st = r["stage"]
        tot[st] = tot.get(st, 0) + 1
        mov[st] = mov.get(st, 0) + (abs(float(r["vx"])) > 1e-3 or abs(float(r["wz"])) > 1e-3)
    return {k: round(mov[k] / tot[k], 2) for k in tot} or None


def analyze(seed, mfile, mapkind, radius, offset=0.0):
    rows = read_missions(mfile)
    rf = run_for(seed, ts_of(mfile))
    res = {"seed": seed, "file": os.path.basename(mfile), "params_md5": rows[0].get("params_md5", "")}
    if not rf or any(not r["sim_end"] for r in rows):
        res["status"] = "UNVER"
        return res
    d = np.genfromtxt(rf, delimiter=",", skip_header=2)
    s_first, s_last = float(rows[0]["sim_start"]), float(rows[-1]["sim_end"])
    if d[-1, 0] < s_last - 1.0:
        res["status"] = "UNVER"
        return res
    tour = []
    final, dist, ends = {}, {}, []
    for r in rows:
        nm = r["mission"]
        gx, gy = float(r["goal_x"]), float(r["goal_y"])
        if nm not in [t[0] for t in tour]:
            tour.append((nm, gx, gy))
        i, x, y = truth_at(d, float(r["sim_end"]))
        final[nm] = r["result"]
        dist[nm] = math.hypot(x - gx, y - gy)
        ends.append((x, y, r["result"]))
    feas = feasibility(seed, mapkind, radius, tour)
    succ = [nm for nm, v in final.items() if v == "SUCCEEDED"]
    real = [nm for nm in succ if dist[nm] <= ARRIVE_M]
    strict = [nm for nm in succ if dist[nm] <= STRICT_M]
    fake = len(succ) - len(real)
    results = [v for v in final.values()]
    if fake:
        cls = "FALSE_ARRIVAL"
    else:
        xs, ys = [p[0] for p in ends[-4:]], [p[1] for p in ends[-4:]]
        spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if len(ends) >= 4 and spread < 0.6 and all(p[2] in ("TIMEOUT", "ABORTED") for p in ends[-4:]) \
                and "TIMEOUT" in results:
            cls = "WEDGED"
        elif feas and sum(v for n, v in feas.items() if n != "home") <= 1 \
                and results.count("ABORTED") + results.count("REJECTED") >= len(results) - 1:
            cls = "CAGE"
        elif len(real) >= 3:
            cls = "OK"
        elif real:
            cls = "PARTIAL"
        else:
            cls = "FAIL"
    # tour statistics on the simulated clock
    seg = d[(d[:, 0] >= s_first) & (d[:, 0] <= s_last)]
    best = (0.0, s_first, s_first)
    i = 0
    while i < len(seg):
        j = i
        while j + 1 < len(seg) and math.hypot(seg[j + 1, 1] - seg[i, 1], seg[j + 1, 2] - seg[i, 2]) < STILL_M:
            j += 1
        if seg[j, 0] - seg[i, 0] > best[0]:
            best = (seg[j, 0] - seg[i, 0], seg[i, 0], seg[j, 0])
        i = j + 1
    wall = float(rows[-1]["epoch_end"]) - float(rows[0]["epoch_start"])
    errs = amcl_error(seed, mfile, d, offset)
    res.update({
        "status": cls, "final": final, "true_dist": {k: round(v, 2) for k, v in dist.items()},
        "real_succ": len(real), "strict_succ": len(strict), "fake_succ": fake,
        "feasible": feas, "rtf": round((s_last - s_first) / wall, 2) if wall > 0 else None,
        "path_m": round(float(np.sum(np.hypot(np.diff(seg[:, 1]), np.diff(seg[:, 2])))), 1),
        "longest_still_s": round(best[0], 1),
        "still_cmd_share": cmd_share(mfile, best[1], best[2]) if best[0] > 30 else None,
        "amcl_err_med": round(float(np.median(errs)), 2) if errs else None,
        "amcl_err_max": round(float(np.max(errs)), 2) if errs else None,
    })
    return res


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    seeds, md5 = set(BENCH_SEEDS), None
    for a in sys.argv[1:]:
        if a.startswith("--seeds="):
            v = a.split("=", 1)[1]
            seeds = None if v == "all" else set(v.split(","))
        elif a.startswith("--md5="):
            md5 = a.split("=", 1)[1]
    if len(args) < 2:
        raise SystemExit("usage: analyze_missions2.py <config> <slam|truth> [first_date [last_date]] [--seeds=a,b]")
    config, mapkind = args[0], args[1]
    d0 = args[2] if len(args) > 2 else None
    d1 = args[3] if len(args) > 3 else None
    files = pick_files(config, mapkind, d0, d1, seeds, md5)
    versions = sorted({read_missions(f)[0].get("params_md5", "") for f in files.values()})
    run_md5 = md5 or (versions[0] if len(versions) == 1 else "")
    if len(versions) > 1:
        print(f"  WARN: several parameter versions in this cohort: {versions} (use --md5=)")
    radius = planner_radius(config, run_md5)
    offset = amcl_offset(config, run_md5)
    print(f"config {config} (md5 {','.join(versions) or '-'}), {mapkind} maps, "
          f"planner radius {radius:.2f} m, {len(files)} worlds")
    print(f"{'seed':13s} {'class':13s} {'<1.5':>4s} {'<0.6':>4s} {'fake':>4s} {'feas':>4s} "
          f"{'rtf':>4s} {'still':>6s} {'amclMed':>7s} {'amclMax':>7s}  missions")
    out, agg = [], {"real": 0, "strict": 0, "fake": 0, "feas": 0, "by_cls": {}}
    for seed, f in sorted(files.items()):
        r = analyze(seed, f, mapkind, radius, offset)
        out.append(r)
        agg["by_cls"][r["status"]] = agg["by_cls"].get(r["status"], 0) + 1
        if r["status"] == "UNVER":
            print(f"{seed:13s} UNVER")
            continue
        agg["real"] += r["real_succ"]
        agg["strict"] += r["strict_succ"]
        agg["fake"] += r["fake_succ"]
        agg["feas"] += sum(r["feasible"].values())
        ms = " ".join(f"{n}:{v[:4]}{'*' if v == 'SUCCEEDED' and r['true_dist'][n] > ARRIVE_M else ''}"
                      for n, v in r["final"].items())
        print(f"{seed:13s} {r['status']:13s} {r['real_succ']:4d} {r['strict_succ']:4d} {r['fake_succ']:4d} "
              f"{sum(r['feasible'].values()):4d} {r['rtf'] or 0:4.2f} {r['longest_still_s']:6.0f} "
              f"{str(r['amcl_err_med']):>7s} {str(r['amcl_err_max']):>7s}  {ms}")
    n = sum(1 for r in out if r["status"] != "UNVER")
    print(f"\nworlds {len(out)} (verified {n}) | real arrivals (within {STRICT_M} m) {agg['strict']}/{4 * n} "
          f"| within {ARRIVE_M} m {agg['real']} | false (beyond {ARRIVE_M} m) {agg['fake']} "
          f"| feasible goals {agg['feas']}/{4 * n}")
    print("classes:", agg["by_cls"])
    name = f"missions_results_{config}_{mapkind}.json"
    if out and seeds == set(BENCH_SEEDS) and len(versions) == 1:
        json.dump(out, open(os.path.join(RUNS, name), "w"), indent=1)
        print(f"results -> runs/{name}")


if __name__ == "__main__":
    main()
