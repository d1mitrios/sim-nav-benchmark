#!/usr/bin/env python3
# dyn3 (cross-room walker): door-conflict analysis + comparison of 3 conditions (static/dyn2/dyn3)
import glob
import re
import json
import numpy as np

RUNS = "/mnt/user-data/uploads/isaac_project/runs"
W90 = 900
ZONE = 1.5     # radius of the "door zone" around the xdoor center
MIN_EP = 0.5   # minimum episode duration (s)

# xdoor ground truth (from the v6 manifests on the device, seed order 001-025)
XDOOR = {
    "20260723001": (0, -3.49, -2.84, 0.55), "20260723002": (0, 4.35, 1.63, 0.89),
    "20260723003": (1, -4.57, -5.20, 0.95), "20260723004": (0, -3.15, 4.59, 0.83),
    "20260723005": (0, 0.82, -2.77, 0.91), "20260723006": (1, 6.52, 3.56, 0.88),
    "20260723007": (0, 2.40, -4.07, 0.94), "20260723008": (1, -5.57, -4.46, 0.84),
    "20260723009": (0, 2.05, -4.82, 0.77), "20260723010": (2, -5.71, -3.94, 0.84),
    "20260723011": (2, 5.96, 2.61, 0.93), "20260723012": (0, 1.51, -3.63, 0.81),
    "20260723013": (0, -7.18, -3.77, 0.73), "20260723014": (2, -3.51, -8.47, 0.56),
    "20260723015": (0, 4.46, 1.74, 0.94), "20260723016": (1, -4.44, 1.95, 0.93),
    "20260723017": (2, -4.06, -6.45, 0.88), "20260723018": (1, 4.93, 2.91, 0.81),
    "20260723019": (0, 3.49, 0.32, 0.74), "20260723020": (2, -2.44, -0.09, 0.82),
    "20260723021": (0, -7.24, 3.96, 0.91), "20260723022": (1, 2.36, -4.16, 0.95),
    "20260723023": (0, -0.21, 2.42, 0.77), "20260723024": (1, 1.06, -3.31, 0.67),
    "20260723025": (1, 4.48, 8.36, 0.82),
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


def newest(files):
    best = {}
    for f in files:
        s = re.search(r"run_(\d+)_", f).group(1)
        st = re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)
        if s not in best or st > best[s][1]:
            best[s] = (f, st)
    return {s: f for s, (f, _) in best.items()}


def base_metrics(d, doors, pxw, pyw, persons):
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    r = dict(dur=float(t[-1]), dist=float(np.hypot(np.diff(x), np.diff(y)).sum()))
    cross = {q["name"]: 0 for q in doors}
    for wall_axis, wall_pos in (("x", pxw), ("y", pyw)):
        coord, other = (x, y) if wall_axis == "x" else (y, x)
        side = coord > wall_pos
        for i2 in np.where(side[:-1] != side[1:])[0]:
            oc = (other[i2] + other[i2 + 1]) / 2
            wd = [q for q in doors if q["axis"] == wall_axis]
            cross[min(wd, key=lambda q: abs(oc - (q["y"] if wall_axis == "x" else q["x"])))["name"]] += 1
    r["cross_per_door"], r["cross"] = cross, sum(cross.values())
    k0 = np.searchsorted(t, t[-1] - 90.0)
    exc = np.hypot(x[k0:] - x[-1], y[k0:] - y[-1])
    r["terminal"] = bool(len(exc) and exc.max() < 0.5)
    still = 0
    i = 0
    while i + W90 < len(t):
        if np.hypot(x[i:i + W90] - x[i], y[i:i + W90] - y[i]).max() < 0.5:
            still += 1
            i += W90
        else:
            i += 100
    r["still"] = still
    seg = np.hypot(np.diff(x), np.diff(y))
    Wd, dance = 600, 0
    i = 0
    while i + Wd < len(t):
        if np.hypot(x[i + Wd] - x[i], y[i + Wd] - y[i]) < 0.35 and seg[i:i + Wd].sum() > 2.5:
            dance += 1
            i += Wd
        else:
            i += 100
    r["dance"] = dance
    if d.shape[1] >= 10:
        P = [(d[:, 4], d[:, 5]), (d[:, 6], d[:, 7]), (d[:, 8], d[:, 9])]
        mind = np.min([np.hypot(x - px, y - py) for px, py in P], axis=0)
    else:
        mind = np.min([np.hypot(x - hx, y - hy) for hx, hy in persons], axis=0)
    r["min_clear"] = float(mind.min())
    r["pers_frac"] = float((mind < 1.2).mean())
    r["intim_frac"] = float((mind < 0.5).mean())
    r["near_contact"] = int(((mind[1:] < 0.35) & (mind[:-1] >= 0.35)).sum())
    return r


def conflicts(d, seed, doors, pxw, pyw):
    """Episodes of simultaneous robot + xroom walker presence in the xdoor zone."""
    pi, dx, dy, g = XDOOR[seed]
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    hx, hy = d[:, 4 + 2 * pi], d[:, 5 + 2 * pi]
    rd = np.hypot(x - dx, y - dy)
    hd = np.hypot(hx - dx, hy - dy)
    both = (rd < ZONE) & (hd < ZONE)
    hum = np.hypot(x - hx, y - hy)
    v = np.concatenate([[0], np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-6)])
    # which manifest door is the xdoor (for crossings through it)
    xd = min(doors, key=lambda q: abs(q["x"] - dx) + abs(q["y"] - dy))
    # crossing times through the xdoor
    wall_axis = xd["axis"]
    coord, other = (x, y) if wall_axis == "x" else (y, x)
    wall_pos = pxw if wall_axis == "x" else pyw
    side = coord > wall_pos
    ct = []
    for i2 in np.where(side[:-1] != side[1:])[0]:
        oc = (other[i2] + other[i2 + 1]) / 2
        wd = [q for q in doors if q["axis"] == wall_axis]
        if min(wd, key=lambda q: abs(oc - (q["y"] if wall_axis == "x" else q["x"])))["name"] == xd["name"]:
            ct.append(t[i2])
    ct = np.array(ct)
    # episodes
    eps = []
    i = 0
    n = len(t)
    while i < n:
        if not both[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and both[j + 1]:
            j += 1
        if t[j] - t[i] >= MIN_EP:
            m = slice(i, j + 1)
            crossed = bool(len(ct) and np.any((ct >= t[i]) & (ct <= t[j] + 8.0)))
            waited = bool(np.any(np.convolve((v[m] < 0.10).astype(int),
                                             np.ones(30), "same") >= 30))  # ~3s
            eps.append(dict(t0=float(t[i]), dur=float(t[j] - t[i]),
                            min_h=float(hum[m].min()), crossed=crossed, waited=waited))
        i = j + 1
    return dict(n_ep=len(eps), ep_time=sum(e["dur"] for e in eps),
                ep_cross=sum(e["crossed"] for e in eps),
                ep_wait=sum(e["waited"] for e in eps),
                ep_minh=min((e["min_h"] for e in eps), default=np.nan),
                x_cross=len(ct), eps=eps, xdoor_name=xd["name"], xdoor_g=g)


def stamp(f):
    return re.search(r"_(\d{8}_\d{6})\.csv", f).group(1)


all_files = glob.glob(f"{RUNS}/run_202607230*.csv")
st_f = newest([f for f in all_files if stamp(f) < "20260723_19"])
d2_f = newest([f for f in all_files if "20260724_22" <= stamp(f) < "20260726"])
d3_f = newest([f for f in all_files if stamp(f) >= "20260726_14"])
print(f"static {len(st_f)} | dyn2 {len(d2_f)} | dyn3 {len(d3_f)}")

rows = []
print(f"{'seed':>5} {'dur':>5} {'m/min':>6} {'cr':>3} {'TD':>3} {'minC':>5} {'<1.2':>5} {'nc':>3} | "
      f"{'xdoor':>5} {'W':>5} {'eps':>4} {'t_ep':>6} {'wait':>5} {'shared':>6} {'minH':>5} {'xCr':>4}")
for seed in sorted(d3_f):
    doors, pxw, pyw, persons = load_world(seed)
    d3 = np.genfromtxt(d3_f[seed], delimiter=",", skip_header=2)
    d2 = np.genfromtxt(d2_f[seed], delimiter=",", skip_header=2)
    dS = np.genfromtxt(st_f[seed], delimiter=",", skip_header=2)
    B3 = base_metrics(d3, doors, pxw, pyw, persons)
    B2 = base_metrics(d2, doors, pxw, pyw, persons)
    BS = base_metrics(dS, doors, pxw, pyw, persons)
    C3 = conflicts(d3, seed, doors, pxw, pyw)
    # baseline: how many "episodes" in the same zone in dyn2 (same-room walkers)
    C2 = conflicts(d2, seed, doors, pxw, pyw)
    rows.append(dict(seed=seed, S=BS, D2=B2, D3=B3, C3=C3, C2n=C2["n_ep"],
                     C2x=C2["x_cross"]))
    print(f"{seed[-3:]:>5} {B3['dur']:5.0f} {B3['dist']/B3['dur']*60:6.1f} {B3['cross']:3d} "
          f"{'X' if B3['terminal'] else '-':>3} {B3['min_clear']:5.2f} {B3['pers_frac']*100:4.1f}% "
          f"{B3['near_contact']:3d} | {C3['xdoor_name'][5:]:>5} {C3['xdoor_g']:5.2f} "
          f"{C3['n_ep']:4d} {C3['ep_time']:6.1f} {C3['ep_wait']:5d} {C3['ep_cross']:6d} "
          f"{C3['ep_minh'] if C3['n_ep'] else float('nan'):5.2f} {C3['x_cross']:4d}")

S_ = [r["S"] for r in rows]
D2_ = [r["D2"] for r in rows]
D3_ = [r["D3"] for r in rows]
C3_ = [r["C3"] for r in rows]


def agg(A):
    dd = sum(a["dur"] for a in A)
    return dict(h=dd / 3600,
                mpm=np.median([a["dist"] / a["dur"] * 60 for a in A]),
                cr=sum(a["cross"] for a in A), crh=sum(a["cross"] for a in A) / (dd / 3600),
                td=sum(a["terminal"] for a in A), still=sum(a["still"] for a in A),
                dance=sum(a["dance"] for a in A),
                mc=min(a["min_clear"] for a in A),
                pf=np.mean([a["pers_frac"] for a in A]) * 100,
                itf=np.mean([a["intim_frac"] for a in A]) * 100,
                nc=sum(a["near_contact"] for a in A))


print("\n=== 3 CONDITIONS (25 worlds, v2.9.1 unmodified) ===")
print(f"{'':>10} {'sim h':>6} {'m/min':>6} {'cross':>6} {'/h':>5} {'TD':>3} {'stil':>4} {'dnc':>4} "
      f"{'minC':>5} {'<1.2':>5} {'<0.5':>5} {'nc':>3}")
for nm, A in (("static", S_), ("dyn2", D2_), ("dyn3 xroom", D3_)):
    g = agg(A)
    print(f"{nm:>10} {g['h']:6.2f} {g['mpm']:6.1f} {g['cr']:6d} {g['crh']:5.1f} {g['td']:3d} "
          f"{g['still']:4d} {g['dance']:4d} {g['mc']:5.2f} {g['pf']:4.1f}% {g['itf']:4.1f}% {g['nc']:3d}")

n_ep = sum(c["n_ep"] for c in C3_)
t_ep = sum(c["ep_time"] for c in C3_)
h3 = sum(a["dur"] for a in D3_) / 3600
print(f"\n=== DOOR CONFLICTS (dyn3) ===")
print(f"simultaneous-zone episodes (<{ZONE}m from xdoor): {n_ep} ({n_ep/h3:.1f}/h), "
      f"total time {t_ep:.0f}s")
print(f"baseline dyn2 in the same zone: {sum(r['C2n'] for r in rows)} episodes")
print(f"  with the robot waiting (v<0.1 for ≥3s): {sum(c['ep_wait'] for c in C3_)}")
print(f"  with a door crossing during/right after: {sum(c['ep_cross'] for c in C3_)}")
minhs = [c["ep_minh"] for c in C3_ if c["n_ep"]]
print(f"  min human distance in the episodes: min {min(minhs):.2f} m, "
      f"median {np.median(minhs):.2f} m")
print(f"crossings THROUGH the xdoor: dyn3 {sum(c['x_cross'] for c in C3_)} vs dyn2 "
      f"{sum(r['C2x'] for r in rows)} (same doors)")
td_door = [r for r in rows if r["D3"]["terminal"]]
print(f"terminal deadlocks dyn3: {len(td_door)}")

json.dump([{"seed": r["seed"], "S": r["S"], "D2": r["D2"], "D3": r["D3"],
            "C3": {k: v for k, v in r["C3"].items() if k != "eps"},
            "eps": r["C3"]["eps"]} for r in rows],
          open("/home/user/isaac/xroom_rows.json", "w"))
print("\nsaved xroom_rows.json")
