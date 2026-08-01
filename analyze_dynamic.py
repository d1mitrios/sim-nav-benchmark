#!/usr/bin/env python3
# Phase 2 — THE COST-OF-DYNAMICS COMPARISON: static vs dynamic, paired per seed.
# v2.9.1 UNMODIFIED in both conditions. Same geometry (md5-verified), only the
# people changed: stationary (static batch 23/7) -> walking (dynamic batch 24/7).
# Uniform deadlock rule: net displacement < 0.5 m over a 90 sim-s window.
# Terminal: final 90 sim-s with excursion < 0.5 m from the final position.
import glob
import re
import json
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
STATIC_HI = "20260723_19"   # static v2.9.1: 23/7 10:36-17:57 (before the v1 baseline)
DYN_LO = "20260724_15"      # dynamic: 24/7 15:27+ (the 12:39 fragment is excluded)
W90 = 900                   # 90 s @ ~10 Hz

BATCH_LOG_DYN = {  # runs/batch_log.csv, today's 25 lines (ground truth from device)
    "20260723001": ("full", 1201), "20260723002": ("terminal_deadlock", 423),
    "20260723003": ("terminal_deadlock", 199), "20260723004": ("full", 1201),
    "20260723005": ("terminal_deadlock", 1141), "20260723006": ("terminal_deadlock", 1112),
    "20260723007": ("terminal_deadlock", 964), "20260723008": ("terminal_deadlock", 1115),
    "20260723009": ("terminal_deadlock", 680), "20260723010": ("full", 1201),
    "20260723011": ("terminal_deadlock", 596), "20260723012": ("terminal_deadlock", 1040),
    "20260723013": ("full", 1201), "20260723014": ("full", 1202),
    "20260723015": ("terminal_deadlock", 188), "20260723016": ("terminal_deadlock", 711),
    "20260723017": ("full", 1202), "20260723018": ("full", 1202),
    "20260723019": ("terminal_deadlock", 238), "20260723020": ("terminal_deadlock", 264),
    "20260723021": ("full", 1203), "20260723022": ("terminal_deadlock", 809),
    "20260723023": ("terminal_deadlock", 1163), "20260723024": ("full", 1201),
    "20260723025": ("terminal_deadlock", 829),
}


def load_world(seed):
    doors, persons = [], []
    for line in open(f"{RUNS}/world_{seed}.csv"):
        p = line.strip().split(",")
        if p[0] == "door":
            doors.append(dict(name=p[1], x=float(p[2]), y=float(p[3]), g=float(p[4]), axis=p[5]))
        elif p[0] == "person":
            persons.append((float(p[2]), float(p[3])))
    pxw = next(d["x"] for d in doors if d["axis"] == "x")
    pyw = next(d["y"] for d in doors if d["axis"] == "y")
    return doors, pxw, pyw, persons


def newest_per_seed(files):
    best = {}
    for f in files:
        seed = re.search(r"run_(\d+)_", f).group(1)
        stamp = re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)
        if seed not in best or stamp > best[seed][1]:
            best[seed] = (f, stamp)
    return {s: f for s, (f, _) in best.items()}


def door_crossings(x, y, doors, pxw, pyw):
    """Sign-changes per wall, attributed to the nearest door of the span (same as analyze_batch)."""
    per_door = {d_["name"]: 0 for d_ in doors}
    for wall_axis, wall_pos in (("x", pxw), ("y", pyw)):
        coord = x if wall_axis == "x" else y
        other = y if wall_axis == "x" else x
        side = coord > wall_pos
        ci = np.where(side[:-1] != side[1:])[0]
        wall_doors = [d_ for d_ in doors if d_["axis"] == wall_axis]
        for i2 in ci:
            oc = (other[i2] + other[i2 + 1]) / 2
            dd = min(wall_doors, key=lambda q: abs(oc - (q["y"] if wall_axis == "x" else q["x"])))
            per_door[dd["name"]] += 1
    return per_door


def analyze(csv, doors, pxw, pyw, persons_static):
    d = np.genfromtxt(csv, delimiter=",", skip_header=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    seg = np.hypot(np.diff(x), np.diff(y))
    r = dict(dur=float(t[-1]), dist=float(seg.sum()))
    r["cross_per_door"] = door_crossings(x, y, doors, pxw, pyw)
    r["cross"] = sum(r["cross_per_door"].values())

    # deadlock: first t with net<0.5 m over 90 sim-s
    dl_t = None
    i = 0
    while i + W90 < len(t):
        if np.hypot(x[i + W90] - x[i], y[i + W90] - y[i]) < 0.5:
            dl_t = float(t[i])
            break
        i += 25
    r["dl_t"] = dl_t

    # terminal: excursion of the final 90s window from the FINAL position
    k0 = np.searchsorted(t, t[-1] - 90.0)
    exc = np.hypot(x[k0:] - x[-1], y[k0:] - y[-1])
    r["terminal"] = bool(len(exc) and exc.max() < 0.5)

    # onset of the final stillness (first sample of the contiguous "near the end" interval)
    m = len(t) - 1
    while m > 0 and np.hypot(x[m - 1] - x[-1], y[m - 1] - y[-1]) < 0.5:
        m -= 1
    r["onset_t"] = float(t[m]) if r["terminal"] else None
    r["end_pos"] = (float(x[-1]), float(y[-1]))
    r["end_door_dist"] = min(np.hypot(x[-1] - d_["x"], y[-1] - d_["y"]) for d_ in doors)

    # --- humans ---
    if d.shape[1] >= 10:  # fmt v2: p0..p2 in the CSV
        P = [(d[:, 4], d[:, 5]), (d[:, 6], d[:, 7]), (d[:, 8], d[:, 9])]
        r["walkers"] = sum(1 for px, py in P
                           if float(np.hypot(px - px[0], py - py[0]).max()) > 1.0)
        per = np.array([np.hypot(x - px, y - py) for px, py in P])
        mind = per.min(axis=0)
        r["min_clear"] = float(mind.min())
        r["pers_frac"] = float((mind < 1.2).mean())
        r["intim_frac"] = float((mind < 0.5).mean())
        r["near_contact"] = int(((mind[1:] < 0.35) & (mind[:-1] >= 0.35)).sum())
        if r["terminal"]:
            r["h_at_onset"] = float(mind[m])
            r["h_min_final"] = float(mind[m:].min())
            r["h_mean_final"] = float(mind[m:].mean())
    else:  # static runs: humans stationary at the manifest positions
        per = np.array([np.hypot(x - px, y - py) for px, py in persons_static])
        mind = per.min(axis=0)
        r["walkers"] = 0
        r["min_clear"] = float(mind.min())
        r["pers_frac"] = float((mind < 1.2).mean())
        r["intim_frac"] = float((mind < 0.5).mean())
    return r


all_files = glob.glob(f"{RUNS}/run_202607230*.csv")


def stamp(f):
    return re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)


st_files = newest_per_seed([f for f in all_files if stamp(f) < STATIC_HI])
dy_files = newest_per_seed([f for f in all_files if stamp(f) >= DYN_LO])
print(f"static files: {len(st_files)}, dynamic files: {len(dy_files)}")

rows = []
hdr = (f"{'seed':>11} | {'ST dur':>6} {'dist':>6} {'cr':>3} {'TD':>3} | "
       f"{'DY dur':>6} {'dist':>6} {'cr':>3} {'TD':>3} {'dl@':>5} {'wlk':>3} "
       f"{'minC':>5} {'<1.2':>5} {'<0.5':>5} {'h@on':>5} {'log':>5}")
print(hdr)
for seed in sorted(dy_files):
    doors, pxw, pyw, persons = load_world(seed)
    S = analyze(st_files[seed], doors, pxw, pyw, persons)
    D = analyze(dy_files[seed], doors, pxw, pyw, persons)
    log_reason, log_wall = BATCH_LOG_DYN[seed]
    agree = (log_reason == "terminal_deadlock") == D["terminal"]
    rows.append(dict(seed=seed, S=S, D=D, log=log_reason, log_wall=log_wall, agree=agree,
                     doors=[(d_["name"], d_["g"]) for d_ in doors]))
    print(f"{seed[-3:]:>11} | {S['dur']:6.0f} {S['dist']:6.1f} {S['cross']:3d} "
          f"{'X' if S['terminal'] else '-':>3} | "
          f"{D['dur']:6.0f} {D['dist']:6.1f} {D['cross']:3d} "
          f"{'X' if D['terminal'] else '-':>3} "
          f"{('%.0f' % D['dl_t']) if D['dl_t'] is not None else '—':>5} {D['walkers']:3d} "
          f"{D['min_clear']:5.2f} {D['pers_frac']*100:4.1f}% {D['intim_frac']*100:4.1f}% "
          f"{('%.2f' % D.get('h_at_onset', -1)) if D['terminal'] else '—':>5} "
          f"{'OK' if agree else '!!':>5}")

S_ = [r["S"] for r in rows]
D_ = [r["D"] for r in rows]
n = len(rows)
td_s = sum(1 for s in S_ if s["terminal"])
td_d = sum(1 for x_ in D_ if x_["terminal"])
print(f"\n=== AGGREGATES ({n} pairs, same seeds/geometry) ===")
print(f"terminal deadlocks: static {td_s}/{n}  ->  dynamic {td_d}/{n}")
print(f"offline rule agreement with batch_log: {sum(1 for r in rows if r['agree'])}/{n}")
dl_ts = [x_["dl_t"] for x_ in D_ if x_["dl_t"] is not None]
if dl_ts:
    print(f"dynamic: time to 1st deadlock — median {np.median(dl_ts):.0f}s, "
          f"min {min(dl_ts):.0f}s, max {max(dl_ts):.0f}s")
sd, dd = sum(s["dur"] for s in S_), sum(x_["dur"] for x_ in D_)
sm, dm = sum(s["dist"] for s in S_), sum(x_["dist"] for x_ in D_)
sc, dc = sum(s["cross"] for s in S_), sum(x_["cross"] for x_ in D_)
print(f"sim time:  static {sd/3600:.2f} h,  dynamic {dd/3600:.2f} h")
print(f"distance:  static {sm:.0f} m ({sm/sd*60:.1f} m/min),  dynamic {dm:.0f} m ({dm/dd*60:.1f} m/min)")
print(f"crossings: static {sc} ({sc/(sd/3600):.1f}/h),  dynamic {dc} ({dc/(dd/3600):.1f}/h)")

full = [x_ for x_ in D_ if not x_["terminal"]]
if full:
    fm, fd = sum(x_["dist"] for x_ in full), sum(x_["dur"] for x_ in full)
    fc = sum(x_["cross"] for x_ in full)
    print(f"dynamic full runs ONLY ({len(full)}): {fm/fd*60:.1f} m/min, {fc/(fd/3600):.1f} crossings/h")

term = [x_ for x_ in D_ if x_["terminal"]]
if term:
    near = [x_ for x_ in term if x_["h_min_final"] < 1.5]
    at_door = [x_ for x_ in term if x_["end_door_dist"] < 1.2]
    print(f"\n=== DIAGNOSIS OF THE {len(term)} DYNAMIC DEADLOCKS ===")
    print(f"with a human <1.5 m at some point during the final still interval: {len(near)}/{len(term)}")
    print(f"human distance at the moment of onset: median "
          f"{np.median([x_['h_at_onset'] for x_ in term]):.2f} m")
    print(f"min human distance in the final interval: median "
          f"{np.median([x_['h_min_final'] for x_ in term]):.2f} m")
    print(f"stuck at a door (<1.2 m from a door center): {len(at_door)}/{len(term)}")
    for x_, r in [(x_, r) for r in rows for x_ in [r["D"]] if x_["terminal"]]:
        print(f"  seed {r['seed'][-3:]}: onset {x_['onset_t']:.0f}s, h@onset {x_['h_at_onset']:.2f} m, "
              f"h_min_final {x_['h_min_final']:.2f} m, h_mean {x_['h_mean_final']:.2f} m, "
              f"door_dist {x_['end_door_dist']:.2f} m, walkers {x_['walkers']}")

mc = [x_["min_clear"] for x_ in D_]
print(f"\n=== SOCIAL (dynamic, 25 runs) ===")
print(f"min clearance: median {np.median(mc):.2f} m, min {min(mc):.2f} m")
print(f"time <1.2 m: mean {np.mean([x_['pers_frac'] for x_ in D_])*100:.1f}%, "
      f"<0.5 m: {np.mean([x_['intim_frac'] for x_ in D_])*100:.1f}%")
print(f"near contacts (<0.35 m): total {sum(x_['near_contact'] for x_ in D_)}")
print(f"walkers per world: {sorted(x_['walkers'] for x_ in D_)}")

json.dump([{**{"seed": r["seed"], "log": r["log"], "log_wall": r["log_wall"],
               "doors": r["doors"]},
            "S": r["S"], "D": r["D"]} for r in rows],
          open("/home/user/isaac/dynamic_rows.json", "w"))
print("\nsaved dynamic_rows.json")
