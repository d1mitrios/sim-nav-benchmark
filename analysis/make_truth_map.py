#!/usr/bin/env python3
# Ground-truth occupancy map of one world, for the perfect-map control (experiments 6 and 8).
# Usage: python3 make_truth_map.py <seed> [out_dir]   (default out_dir: runs/)
# Renders the manifest's static geometry (partitions with their door gaps, furniture boxes,
# cylinders and the boundary walls) with the rasteriser of the map evaluation
# (evaluate_map.render_truth), 0.05 m per cell, in the world frame. The world frame is the map
# frame because the robot spawns at the origin facing +x. People are left out: they move, so
# they belong in the live costmap, not in the static map. Every cell inside the arena is either
# free or occupied, the cells outside the boundary walls are unknown.
# Writes truth_<seed>.pgm and truth_<seed>.yaml in the map_server format of the SLAM maps.
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate_map as em

RUNS_DIR = os.path.join(os.path.dirname(HERE), "runs")
RES = 0.05
ORIGIN = (-10.5, -10.5)
N = int(round(21.0 / RES))          # 420 cells, the arena with its boundary walls fits inside
INNER = 9.75                        # inner face of the boundary walls


def render(seed, man_dir=RUNS_DIR):
    occ, _ = em.render_truth(f"{man_dir}/world_{seed}.csv", RES, (N, N), ORIGIN)
    c = ORIGIN[0] + (np.arange(N) + 0.5) * RES          # cell centres, same along x and y
    inside = (np.abs(c)[:, None] < INNER) & (np.abs(c)[None, :] < INNER)
    img = np.full((N, N), 205, np.uint8)                 # unknown
    img[inside] = 254                                    # free
    img[occ] = 0                                         # occupied
    return img


def write(seed, out_dir=RUNS_DIR, man_dir=RUNS_DIR):
    img = render(seed, man_dir)
    base = f"{out_dir}/truth_{seed}"
    with open(base + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (N, N))
        f.write(np.flipud(img).tobytes())                # PGM rows run from the top
    with open(base + ".yaml", "w") as f:
        f.write(f"image: truth_{seed}.pgm\nmode: trinary\nresolution: {RES:.3f}\n"
                f"origin: [{ORIGIN[0]:.3f}, {ORIGIN[1]:.3f}, 0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return base + ".yaml"


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("usage: make_truth_map.py <seed> [out_dir]")
    out = sys.argv[2] if len(sys.argv) > 2 else RUNS_DIR
    print(write(sys.argv[1], out))
