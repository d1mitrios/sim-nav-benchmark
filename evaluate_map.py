#!/usr/bin/env python3
# Phase 3: SLAM map evaluation vs manifest ground truth.
# Usage: python3 evaluate_map.py <map.yaml> <world_manifest.csv> [out.png]
# The map (map_saver_cli PGM+YAML) is compared against the manifest geometry,
# reporting: wall coverage, occupied precision, free-IoU, per-obstacle detection.
# Frames: odom starts at the spawn (0,0, yaw 0) => map frame ~ world frame; a small
# translation search (±0.4 m) handles the remaining misalignment (no rotation).
import sys
import math
import re
import numpy as np


def load_map(yaml_path):
    meta = {}
    for line in open(yaml_path):
        m = re.match(r"(\w+):\s*(.+)", line.strip())
        if m:
            meta[m.group(1)] = m.group(2)
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    pgm = yaml_path.rsplit(".", 1)[0] + ".pgm"
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    img = np.flipud(img)   # PGM: first row = top; grid: row 0 = origin (bottom)
    occ = img < 100
    free = img > 200
    return occ, free, res, origin


def render_truth(manifest, res, shape, origin):
    """Occupied mask of the true geometry on the same grid."""
    H, W = shape
    gt = np.zeros(shape, bool)

    def fill_rect(cx, cy, sx, sy, yaw_deg):
        th = math.radians(yaw_deg)
        c, s = math.cos(th), math.sin(th)
        n = max(3, int(max(sx, sy) / res) * 2)
        for ux in np.linspace(-0.5, 0.5, n):
            for uy in np.linspace(-0.5, 0.5, max(3, int(min(sx, sy) / res) * 2)):
                x = cx + (ux * sx) * c - (uy * sy) * s
                y = cy + (ux * sx) * s + (uy * sy) * c
                i = int((y - origin[1]) / res)
                j = int((x - origin[0]) / res)
                if 0 <= i < H and 0 <= j < W:
                    gt[i, j] = True

    obstacles = []
    for line in open(manifest):
        p = line.strip().split(",")
        if p[0] == "box":
            fill_rect(float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6]))
            if not p[1].startswith("partition"):
                obstacles.append((p[1], float(p[2]), float(p[3]), max(float(p[4]), float(p[5])) / 2))
        elif p[0] == "cyl":
            cx, cy, r = float(p[2]), float(p[3]), float(p[4])
            n = max(4, int(2 * r / res) * 2)
            for ux in np.linspace(-r, r, n):
                for uy in np.linspace(-r, r, n):
                    if ux * ux + uy * uy <= r * r:
                        i = int((cy + uy - origin[1]) / res)
                        j = int((cx + ux - origin[0]) / res)
                        if 0 <= i < H and 0 <= j < W:
                            gt[i, j] = True
            obstacles.append((p[1], cx, cy, r))
    # boundary: centers at ±10 (as in build_arena/worldgen), inner face at 9.75
    for bx, by, sx, sy in [(0, 10.0, 20.5, 0.5), (0, -10.0, 20.5, 0.5),
                           (10.0, 0, 0.5, 20.5), (-10.0, 0, 0.5, 20.5)]:
        fill_rect(bx, by, sx, sy, 0)
    return gt, obstacles


def dilate(mask, k):
    out = mask.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            out |= np.roll(np.roll(mask, dy, 0), dx, 1)
    return out


def main():
    map_yaml, manifest = sys.argv[1], sys.argv[2]
    occ, free, res, origin = load_map(map_yaml)
    gt, obstacles = render_truth(manifest, res, occ.shape, origin)

    # small translation search for residual misalignment
    best = (-1.0, 0, 0)
    k = int(0.4 / res)
    gtd = dilate(gt, 1)
    for dy in range(-k, k + 1, 2):
        for dx in range(-k, k + 1, 2):
            sc = float((np.roll(np.roll(occ, dy, 0), dx, 1) & gtd).sum())
            if sc > best[0]:
                best = (sc, dy, dx)
    _, dy, dx = best
    occ_a = np.roll(np.roll(occ, dy, 0), dx, 1)
    free_a = np.roll(np.roll(free, dy, 0), dx, 1)
    print(f"[eval] alignment shift: dx={dx*res:.2f} m dy={dy*res:.2f} m")

    gt_d = dilate(gt, 2)          # tolerance 2 cells (0.10 m)
    occ_d = dilate(occ_a, 2)
    # v2: the lidar sees SURFACES, not interiors — coverage vs the EDGES of the geometry
    gt_edge = gt & ~(np.roll(gt, 1, 0) & np.roll(gt, -1, 0) &
                     np.roll(gt, 1, 1) & np.roll(gt, -1, 1))
    prec = float((occ_a & gt_d).sum()) / max(occ_a.sum(), 1)
    surf_cov = float((gt_edge & occ_d).sum()) / max(gt_edge.sum(), 1)
    free_gt = ~dilate(gt, 3)
    free_iou = float((free_a & free_gt).sum()) / max((free_a | free_gt).sum(), 1)
    unknown = 1.0 - (occ.sum() + free.sum()) / occ.size
    print(f"[eval] occupied precision (map->truth): {prec*100:.1f}%")
    print(f"[eval] SURFACE coverage (truth edges->map): {surf_cov*100:.1f}%")
    print(f"[eval] free-space IoU: {free_iou*100:.1f}%")
    print(f"[eval] unknown fraction: {unknown*100:.1f}%")
    # per-quadrant surface coverage (quad worlds: pxw/pyw from the manifest doors)
    pxw = pyw = None
    for line in open(manifest):
        p = line.strip().split(",")
        if p[0] == "door" and p[5] == "x":
            pxw = float(p[2])
        elif p[0] == "door" and p[5] == "y":
            pyw = float(p[3])
    if pxw is not None and pyw is not None:
        H, W = gt.shape
        jj, ii = np.meshgrid(np.arange(W), np.arange(H))
        wx = jj * res + origin[0]
        wy = ii * res + origin[1]
        for qname, qmask in (("SW", (wx < pxw) & (wy < pyw)), ("SE", (wx >= pxw) & (wy < pyw)),
                             ("NW", (wx < pxw) & (wy >= pyw)), ("NE", (wx >= pxw) & (wy >= pyw))):
            e = gt_edge & qmask
            if e.sum():
                print(f"    quadrant {qname}: surface coverage "
                      f"{float((e & occ_d).sum())/e.sum()*100:5.1f}%")
    det = 0
    for name, cx, cy, r in obstacles:
        i = int((cy - origin[1]) / res)
        j = int((cx - origin[0]) / res)
        kk = max(2, int((r + 0.1) / res))
        H, W = occ.shape
        win = occ_a[max(0, i - kk):min(H, i + kk), max(0, j - kk):min(W, j + kk)]
        hit = win.sum() > 3
        det += hit
        print(f"    {name:>12}: {'OK' if hit else 'MISSED'}")
    print(f"[eval] obstacles detected: {det}/{len(obstacles)}")

    if len(sys.argv) > 3:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
        ax.set_facecolor("#fcfcfb")
        base = np.full(occ.shape, 0.94)
        base[free_a] = 1.0
        base[occ_a] = 0.15
        ax.imshow(base, cmap="gray", origin="lower", vmin=0, vmax=1)
        yy, xx = np.where(gt)
        ax.scatter(xx, yy, s=0.15, c="#e34948", alpha=0.35, label="ground truth")
        ax.set_title("SLAM map (grey) vs manifest ground truth (red)", fontsize=11)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_xticks([]); ax.set_yticks([])
        fig.savefig(sys.argv[3], bbox_inches="tight", facecolor="#f9f9f7")
        print(f"[eval] figure -> {sys.argv[3]}")


if __name__ == "__main__":
    main()
