#!/usr/bin/env python3
# Where does AMCL lose track on self-built maps? (experiments 7 and 5b)
# Usage: python3 amcl_map_coverage.py
# For every AMCL update of the SLAM-map arms, the error against the simulator's true position and
# the share of map cells within 3 m of the true position that the mapping run saw (free or
# occupied, as opposed to unknown). Compares updates with an error above 1 m against the others.
# Writes runs/missions_amcl_coverage.json.
import os
import sys
import csv
import math
import json
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_missions2 as am
import evaluate_map as em
from stats_missions import ARMS

RADIUS_M = 3.0
BAD_M = 1.0


def main():
    out = {}
    for arm in ("rev_slam", "v9_slam"):
        cfg, mk, d0, d1, md5 = ARMS[arm]
        files = am.pick_files(cfg, mk, d0, d1, set(am.BENCH_SEEDS), md5)
        off = am.amcl_offset(cfg, md5 or "")
        share_bad, share_good = [], []
        for seed, f in sorted(files.items()):
            af = f.replace("missions_", "amcl_")
            rf = am.run_for(seed, am.ts_of(f))
            if not (os.path.exists(af) and rf):
                continue
            d = np.genfromtxt(rf, delimiter=",", skip_header=2)
            occ, free, res, origin = em.load_map(f"{am.RUNS}/world_{seed}.yaml")
            known = occ | free
            h, w = known.shape
            rc = int(round(RADIUS_M / res))
            yy, xx = np.ogrid[-rc:rc + 1, -rc:rc + 1]
            mask = (yy * yy + xx * xx) <= rc * rc
            for r in csv.DictReader(open(af)):
                if not r["sim_t"]:
                    continue
                _, tx, ty = am.truth_at(d, float(r["sim_t"]))
                yaw = math.radians(float(r["yaw_deg"]))
                err = math.hypot(float(r["x"]) + off * math.cos(yaw) - tx,
                                 float(r["y"]) + off * math.sin(yaw) - ty)
                i, j = int(round((ty - origin[1]) / res)), int(round((tx - origin[0]) / res))
                i0, i1, j0, j1 = i - rc, i + rc + 1, j - rc, j + rc + 1
                if i0 < 0 or j0 < 0 or i1 > h or j1 > w:
                    continue
                share = float(known[i0:i1, j0:j1][mask].mean())
                (share_bad if err > BAD_M else share_good).append(share)
        sb, sg = np.array(share_bad), np.array(share_good)
        res_arm = dict(updates=int(len(sb) + len(sg)), bad=int(len(sb)),
                       median_share_bad=float(np.median(sb)) if len(sb) else None,
                       median_share_good=float(np.median(sg)) if len(sg) else None,
                       bad_below_half=float((sb < 0.5).mean()) if len(sb) else None,
                       good_below_half=float((sg < 0.5).mean()) if len(sg) else None)
        out[arm] = res_arm
        print(f"{arm}: {res_arm['updates']} updates, {res_arm['bad']} with error > {BAD_M} m | share of mapped "
              f"cells within {RADIUS_M} m, median: bad {res_arm['median_share_bad']:.2f}, good "
              f"{res_arm['median_share_good']:.2f} | share below 0.5: bad {100 * res_arm['bad_below_half']:.0f}%, "
              f"good {100 * res_arm['good_below_half']:.0f}%")
    json.dump(out, open(os.path.join(am.RUNS, "missions_amcl_coverage.json"), "w"), indent=1)
    print("results -> runs/missions_amcl_coverage.json")


if __name__ == "__main__":
    main()
