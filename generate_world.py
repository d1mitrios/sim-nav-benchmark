# === Random world generator (seeded) — Benchmark Factory — v6 (quad + CROSS-ROOM walker) ===
# v6: ONE walker per world passes a DOOR (door conflict): loop spawn->D1->D2->far->D2->D1,
#     where D1/D2 are points 0.9 m on either side of the widest door of its room. Column 7
#     of the path rows = pause seconds per waypoint (0 at the door points — nobody stands in the opening).
#     The xdoor,* row in the manifest = the conflict door (ground truth for the conflict metrics).
# v5: path,person_i,x,y,seq,speed rows = seeded same-room routes (the other 2).
# GEOMETRY DOES NOT CHANGE for the same seed: all v6 draws happen AFTER placement.
# v3: MODE = "rooms" -> a partition wall cuts the arena into 2 rooms; its 1-2 DOORS
# (0.65-0.95 m) are the ONLY passages -> a full patrol REQUIRES crossing.
# Metric: crossings/hour per door width (door,* rows in the manifest = ground truth).
# MODE = "open" -> the v2 behavior (free-standing gates-furniture).
# Reads the seed from C:/isaac_project/world_seed.txt (if missing: random, printed).
# Keeps: boundary walls, lights, GroundPlane, robot, graphs, the 3 people (move only).
# Regenerates: DENSE clutter (boxes/cylinders, MIN_OBS_GAP 0.7) + N_GATES deliberately narrow
# corridor passages (orange walls, clear width 0.65-0.95 m = labeled ground
# truth for the metrics: success-vs-width). Each gate's "throat" is reserved so no
# other obstacle blocks it. Rules: clear spawn zone at (0,0), people in open space.
# Manifest -> C:/isaac_project/runs/world_<seed>.csv (gate,* rows = the narrows + widths).
# SAVES the stage (git = safety net). Robot: 0.42 m + 2×0.12 inflation ≈ 0.66 minimum.
# Use: File → Open in the Script Editor → Run (with Stop pressed, before Play).
import os
import random
import math
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf

SEED_FILE = "C:/isaac_project/world_seed.txt"
RUNS_DIR = "C:/isaac_project/runs"
ARENA = "/Arena"
SPAWN_CLEAR_R = 2.0        # clear radius around (0,0) for the robot
MIN_OBS_GAP = 0.7          # v2: tighter clutter (was 1.0)
PERSON_CLEAR = 1.2         # clear space around a person
N_BOXES = (8, 12)          # v2: denser (was 6-10)
N_CYLS = (4, 6)            # v2: denser (was 3-5)
N_GATES = (3, 4)           # v2: deliberately narrow corridor passages (only in MODE="open")
GATE_W = (0.65, 0.95)      # v2: clear gate width (0.66 = the robot's physical minimum)
GATE_LEN = (1.5, 2.5)      # v2: gate corridor length
GATE_T = 0.4               # v2: gate wall thickness
MODE = "quad"              # v4: "quad" (4 rooms) | "rooms" (2 rooms) | "open" (gates-furniture)
N_DOORS = (2, 3)           # v3.1: more doors = more boundary samples per world
PART_T = 0.4               # v3: partition thickness
DOOR_KEEPOUT = 1.2         # v3: clear radius around each door (free approach)
# v3.1 HARD MODE: one anchor door guaranteed passable (connectivity for the patrol),
# the rest are sampled AROUND the robot's physical limit
# (0.42 body + 2×0.12 inflation ≈ 0.66) -> that's where the success-vs-width curve lives.
DOOR_W_ANCHOR = (0.72, 0.95)
DOOR_W_HARD = (0.50, 0.78)

stage = omni.usd.get_context().get_stage()

# ---- seed ----
if os.path.exists(SEED_FILE):
    seed = int(open(SEED_FILE).read().strip())
else:
    seed = random.randint(1, 999999)
rng = random.Random(seed)
print(f"[worldgen] seed = {seed}")

# ---- clear old obstacles (we keep walls/lights/PEOPLE — the person_N prims are
# MOVED further down, not recreated: they carry character refs + lidar_proxy) ----
keep = ("boundary_", "SunLight", "DomeLight", "person_")
removed = 0
arena = stage.GetPrimAtPath(ARENA)
for child in list(arena.GetChildren()):
    name = child.GetName()
    if not any(name.startswith(k) for k in keep):
        stage.RemovePrim(child.GetPath())
        removed += 1
print(f"[worldgen] removed {removed} old prims")

# ---- helpers (same pattern as build_arena) ----
def box(name, pos, size, yaw_deg=0.0, color=(0.6, 0.6, 0.6)):
    c = UsdGeom.Cube.Define(stage, ARENA + "/" + name)
    c.GetSizeAttr().Set(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if yaw_deg:
        xf.AddRotateZOp().Set(yaw_deg)
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

def cyl(name, pos, radius, height, color=(0.6, 0.6, 0.6)):
    c = UsdGeom.Cylinder.Define(stage, ARENA + "/" + name)
    c.GetRadiusAttr().Set(radius)
    c.GetHeightAttr().Set(height)
    c.GetAxisAttr().Set("Z")
    c.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2), Gf.Vec3f(radius, radius, height / 2)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

placed = []  # (x, y, effective_radius) — for placement (includes keepouts/persons)
placed_obs = []  # v5: ONLY physical obstacles (for route visibility checks)
partitions_fe = []  # v4: list of (axis, pos) for far_enough (1 in rooms, 2 in quad)

def far_enough(x, y, r):
    if math.hypot(x, y) < SPAWN_CLEAR_R + r:
        return False
    # v3/v4: nothing flush against the partitions (clear corridors along them)
    for pax, ppos in partitions_fe:
        d = abs((x if pax == "x" else y) - ppos)
        if d < r + PART_T / 2 + 0.5:
            return False
    for px, py, pr in placed:
        if math.hypot(x - px, y - py) < r + pr + MIN_OBS_GAP:
            return False
    return True

def sample(r, tries=200):
    for _ in range(tries):
        x = rng.uniform(-8.5, 8.5)
        y = rng.uniform(-8.5, 8.5)
        if far_enough(x, y, r):
            return x, y
    return None

manifest = [f"# world seed {seed}", "type,name,x,y,param1,param2,yaw"]

# ---- v3: partition with doors (MODE="rooms") — the ONLY passages ----
if MODE == "rooms":
    axis = rng.choice(("x", "y"))                      # "x": vertical wall x=pos, "y": horizontal y=pos
    pos = rng.choice((-1.0, 1.0)) * rng.uniform(2.2, 5.0)
    nd = rng.randint(*N_DOORS)
    centers = []
    for _ in range(300):
        if len(centers) >= nd:
            break
        c = rng.uniform(-7.5, 7.5)
        if all(abs(c - c2) > 3.0 for c2 in centers):
            centers.append(c)
    # v3.1: one random door from the anchor range, the rest from the hard range
    widths = [rng.uniform(*DOOR_W_HARD) for _ in centers]
    widths[rng.randrange(len(centers))] = rng.uniform(*DOOR_W_ANCHOR)
    doors = sorted(zip(centers, widths))
    edges = [-10.0]
    for c, g in doors:
        edges += [c - g / 2, c + g / 2]
    edges += [10.0]
    for k in range(0, len(edges), 2):
        lo, hi = edges[k], edges[k + 1]
        if hi - lo <= 0.05:
            continue
        mid = (lo + hi) / 2.0
        if axis == "x":
            box(f"partition_{k // 2}", (pos, mid, 1.0), (PART_T, hi - lo, 2.0), 0.0, (0.55, 0.35, 0.65))
            manifest.append(f"box,partition_{k // 2},{pos:.2f},{mid:.2f},{PART_T:.2f},{hi - lo:.2f},0")
        else:
            box(f"partition_{k // 2}", (mid, pos, 1.0), (hi - lo, PART_T, 2.0), 0.0, (0.55, 0.35, 0.65))
            manifest.append(f"box,partition_{k // 2},{mid:.2f},{pos:.2f},{hi - lo:.2f},{PART_T:.2f},0")
    for i, (c, g) in enumerate(doors):
        dx, dy = (pos, c) if axis == "x" else (c, pos)
        placed.append((dx, dy, DOOR_KEEPOUT))          # reserved clear approach
        manifest.append(f"door,door_{i},{dx:.2f},{dy:.2f},{g:.2f},{axis},0")
    partitions_fe.append((axis, pos))
    print(f"[worldgen] partition {axis}={pos:.2f}, doors: " +
          ", ".join(f"W={g:.2f}@{c:.1f}" for c, g in doors))

# ---- v4: CROSS (MODE="quad") — 4 rooms, 1 door per half-wall ----
if MODE == "quad":
    pxw = rng.choice((-1.0, 1.0)) * rng.uniform(2.2, 5.0)   # vertical wall x=pxw
    pyw = rng.choice((-1.0, 1.0)) * rng.uniform(2.2, 5.0)   # horizontal wall y=pyw
    spans = [
        ("S", "x", pxw, -10.0, pyw),   # vertical-south: connects SW-SE
        ("N", "x", pxw, pyw, 10.0),    # vertical-north: NW-NE
        ("W", "y", pyw, -10.0, pxw),   # horizontal-west: SW-NW
        ("E", "y", pyw, pxw, 10.0),    # horizontal-east: SE-NE
    ]
    # 2 anchors on ADJACENT edges (they share a room) -> 3 rooms ALWAYS accessible
    anchors = set(rng.choice([("S", "E"), ("S", "W"), ("N", "E"), ("N", "W")]))
    door_rows = []
    for nm, wax, wpos, lo, hi in spans:
        margin = 1.3
        c_lo, c_hi = lo + margin, hi - margin
        if c_hi - c_lo < 0.8:
            continue
        c = rng.uniform(c_lo, c_hi)
        g = rng.uniform(*DOOR_W_ANCHOR) if nm in anchors else rng.uniform(*DOOR_W_HARD)
        door_rows.append((nm, wax, wpos, lo, hi, c, g))
    seg_i = 0
    for nm, wax, wpos, lo, hi, c, g in door_rows:
        for s_lo, s_hi in ((lo, c - g / 2), (c + g / 2, hi)):
            if s_hi - s_lo <= 0.05:
                continue
            mid = (s_lo + s_hi) / 2.0
            L = s_hi - s_lo
            if wax == "x":
                box(f"partition_{seg_i}", (wpos, mid, 1.0), (PART_T, L, 2.0), 0.0, (0.55, 0.35, 0.65))
                manifest.append(f"box,partition_{seg_i},{wpos:.2f},{mid:.2f},{PART_T:.2f},{L:.2f},0")
            else:
                box(f"partition_{seg_i}", (mid, wpos, 1.0), (L, PART_T, 2.0), 0.0, (0.55, 0.35, 0.65))
                manifest.append(f"box,partition_{seg_i},{mid:.2f},{wpos:.2f},{L:.2f},{PART_T:.2f},0")
            seg_i += 1
        dx, dy = (wpos, c) if wax == "x" else (c, wpos)
        placed.append((dx, dy, DOOR_KEEPOUT))
        tag = "A" if nm in anchors else "H"
        manifest.append(f"door,door_{nm}_{tag},{dx:.2f},{dy:.2f},{g:.2f},{wax},0")
    partitions_fe.append(("x", pxw))
    partitions_fe.append(("y", pyw))
    print(f"[worldgen] quad cross x={pxw:.2f} y={pyw:.2f}; doors: " +
          ", ".join(f"{nm}{'*' if nm in anchors else ''}=W{g:.2f}" for nm, _, _, _, _, c, g in door_rows))

# ---- v2: gates (deliberately narrow passages) — FIRST, so they claim space ----
# Two parallel walls (L × GATE_T) with a clear opening g between them. The throat
# is registered in placed as a pseudo-obstacle (r=g/2) so a corridor stays clear.
gates_made = 0
for i in range(rng.randint(*N_GATES) if MODE == "open" else 0):
    g = rng.uniform(*GATE_W)
    L = rng.uniform(*GATE_LEN)
    yaw = rng.uniform(0, 180)
    er_wall = 0.5 * math.hypot(L, GATE_T)
    er_gate = g / 2 + GATE_T + er_wall          # conservative envelope of the whole gate
    p = sample(er_gate)
    if not p:
        continue
    cx, cy = p
    th = math.radians(yaw)
    nx, ny = -math.sin(th), math.cos(th)        # perpendicular to the corridor direction
    off = g / 2 + GATE_T / 2
    for s, tag in ((+1.0, "a"), (-1.0, "b")):
        wx, wy = cx + s * nx * off, cy + s * ny * off
        box(f"gate_{i}_{tag}", (wx, wy, 1.0), (L, GATE_T, 2.0), yaw, (0.95, 0.45, 0.1))
        placed.append((wx, wy, er_wall))
        placed_obs.append((wx, wy, er_wall))
        manifest.append(f"box,gate_{i}_{tag},{wx:.2f},{wy:.2f},{L:.2f},{GATE_T:.2f},{yaw:.1f}")
    placed.append((cx, cy, g / 2))              # throat reservation
    manifest.append(f"gate,gate_{i},{cx:.2f},{cy:.2f},{g:.2f},{L:.2f},{yaw:.1f}")
    gates_made += 1
print(f"[worldgen] gates: {gates_made} (orange, W = ground truth in the manifest)")

# ---- obstacles ----
nb = rng.randint(*N_BOXES)
nc = rng.randint(*N_CYLS)
for i in range(nb):
    sx = rng.uniform(0.8, 3.0)
    sy = rng.uniform(0.5, 2.0)
    yaw = rng.uniform(0, 180)
    er = 0.5 * math.hypot(sx, sy)
    p = sample(er)
    if not p:
        continue
    x, y = p
    color = (rng.uniform(0.2, 0.9), rng.uniform(0.2, 0.9), rng.uniform(0.2, 0.9))
    box(f"rnd_box_{i}", (x, y, 1.0), (sx, sy, 2.0), yaw, color)
    placed.append((x, y, er))
    placed_obs.append((x, y, er))
    manifest.append(f"box,rnd_box_{i},{x:.2f},{y:.2f},{sx:.2f},{sy:.2f},{yaw:.1f}")

for i in range(nc):
    r = rng.uniform(0.4, 1.4)
    p = sample(r)
    if not p:
        continue
    x, y = p
    color = (rng.uniform(0.2, 0.9), rng.uniform(0.2, 0.9), rng.uniform(0.2, 0.9))
    cyl(f"rnd_cyl_{i}", (x, y, 1.0), r, 2.0, color)
    placed.append((x, y, r))
    placed_obs.append((x, y, r))
    manifest.append(f"cyl,rnd_cyl_{i},{x:.2f},{y:.2f},{r:.2f},2.0,0")

persons_placed = []  # v5
# ---- people: move the existing person_N prims to new random positions ----
for i in range(3):
    base = stage.GetPrimAtPath(f"{ARENA}/person_{i}")
    if not base:
        continue
    p = sample(PERSON_CLEAR) or sample(0.9)   # v2: fallback in a dense world
    if not p:
        print(f"[worldgen] WARN: no free spot for person_{i}, stays where it was")
        continue
    x, y = p
    yaw = rng.uniform(-180, 180)
    xf = UsdGeom.Xformable(base)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(yaw)
    placed.append((x, y, PERSON_CLEAR))
    manifest.append(f"person,person_{i},{x:.2f},{y:.2f},0,0,{yaw:.1f}")
    persons_placed.append((i, x, y))

# ---- v5: people walking routes (seeded, same room, visibility check) ----
PEOPLE_PATHS = True
N_WPTS = (3, 5)
SPEED_RANGE = (0.7, 1.2)

def same_room(x1, y1, x2, y2):
    for pax, ppos in partitions_fe:
        if ((x1 if pax == "x" else y1) > ppos) != ((x2 if pax == "x" else y2) > ppos):
            return False
    return True

def segment_clear(x1, y1, x2, y2):
    L = math.hypot(x2 - x1, y2 - y1)
    for k in range(max(2, int(L / 0.3)) + 1):
        xx = x1 + (x2 - x1) * k / max(2, int(L / 0.3))
        yy = y1 + (y2 - y1) * k / max(2, int(L / 0.3))
        for pax, ppos in partitions_fe:
            if abs((xx if pax == "x" else yy) - ppos) < 0.45:
                return False
        for ox, oy, orr in placed_obs:
            if math.hypot(xx - ox, yy - oy) < orr + 0.45:
                return False
    return True

# ---- v6: ONE cross-room walker (door conflict) — the WIDEST feasible door is chosen ----
xroom_pi = None
if PEOPLE_PATHS and MODE == "quad" and persons_placed:
    cands = []   # (g, pi, spawn, D1, D2, door_name, door_xy)
    for pi, sx0, sy0 in persons_placed:
        for nm, wax, wpos, lo, hi, c, g in door_rows:
            if wax == "x":                       # vertical wall: same half (N/S) as the person?
                if (sy0 > pyw) != (c > pyw):
                    continue
                s = -0.9 if sx0 < wpos else 0.9  # D1 on the person's side
                d1, d2, dxy = (wpos + s, c), (wpos - s, c), (wpos, c)
            else:                                 # horizontal wall: same half (E/W)?
                if (sx0 > pxw) != (c > pxw):
                    continue
                s = -0.9 if sy0 < wpos else 0.9
                d1, d2, dxy = (c, wpos + s), (c, wpos - s), (c, wpos)
            cands.append((g, pi, (sx0, sy0), d1, d2, nm, dxy))
    for g, pi, (sx0, sy0), d1, d2, nm, dxy in sorted(cands, key=lambda q: -q[0]):
        if not segment_clear(sx0, sy0, d1[0], d1[1]):
            continue
        far = None
        for _try in range(40):
            ang = rng.uniform(0, 2 * math.pi)
            dd = rng.uniform(2.0, 6.0)
            fx, fy = d2[0] + dd * math.cos(ang), d2[1] + dd * math.sin(ang)
            if not (-8.8 < fx < 8.8 and -8.8 < fy < 8.8):
                continue
            if same_room(d2[0], d2[1], fx, fy) and segment_clear(d2[0], d2[1], fx, fy):
                far = (fx, fy)
                break
        speed = rng.uniform(*SPEED_RANGE)
        if far:   # loop: spawn -> D1 -> D2 -> far -> D2 -> D1 -> (spawn)
            wpts = [(sx0, sy0, 3.0), (d1[0], d1[1], 0.0), (d2[0], d2[1], 0.0),
                    (far[0], far[1], 3.0), (d2[0], d2[1], 0.0), (d1[0], d1[1], 0.0)]
        else:      # poke: pops in and out through the door
            wpts = [(sx0, sy0, 3.0), (d1[0], d1[1], 0.0), (d2[0], d2[1], 0.6), (d1[0], d1[1], 0.0)]
        for wk, (wx, wy, pz) in enumerate(wpts):
            manifest.append(f"path,person_{pi},{wx:.2f},{wy:.2f},{wk},{speed:.2f},{pz}")
        manifest.append(f"xdoor,person_{pi},{dxy[0]:.2f},{dxy[1]:.2f},{g:.2f},{nm},0")
        xroom_pi = pi
        print(f"[worldgen] XROOM walker person_{pi} via door_{nm} (W={g:.2f}) "
              f"{'loop' if far else 'poke'} @ {speed:.2f} m/s")
        break
    if xroom_pi is None:
        print("[worldgen] XROOM: no feasible door path - all walkers same-room")

if PEOPLE_PATHS:
    for pi, sx0, sy0 in persons_placed:
        if pi == xroom_pi:
            continue
        wpts = [(sx0, sy0)]
        speed = rng.uniform(*SPEED_RANGE)
        for _w in range(rng.randint(*N_WPTS) - 1):
            for _try in range(200):
                ang = rng.uniform(0, 2 * math.pi)
                dd = rng.uniform(2.5, 7.0)
                nx_ = wpts[-1][0] + dd * math.cos(ang)
                ny_ = wpts[-1][1] + dd * math.sin(ang)
                if not (-8.8 < nx_ < 8.8 and -8.8 < ny_ < 8.8):
                    continue
                if not same_room(sx0, sy0, nx_, ny_):
                    continue
                if segment_clear(wpts[-1][0], wpts[-1][1], nx_, ny_):
                    wpts.append((nx_, ny_))
                    break
        while len(wpts) >= 2 and not segment_clear(wpts[-1][0], wpts[-1][1], wpts[0][0], wpts[0][1]):
            wpts.pop()   # the loop must close cleanly (last -> first)
        for wk, (wx, wy) in enumerate(wpts):
            manifest.append(f"path,person_{pi},{wx:.2f},{wy:.2f},{wk},{speed:.2f},3.0")
        print(f"[worldgen] path person_{pi}: {len(wpts)} waypoints @ {speed:.2f} m/s (same-room)")

# ---- manifest + save ----
os.makedirs(RUNS_DIR, exist_ok=True)
open(f"{RUNS_DIR}/current_seed.txt", "w").write(str(seed))   # v2: for the metrics_logger
mpath = f"{RUNS_DIR}/world_{seed}.csv"
open(mpath, "w").write("\n".join(manifest) + "\n")
print(f"[worldgen] {len(placed) - gates_made} objects placed ({gates_made} gates), manifest -> {mpath}")
print("[worldgen] saved:", stage.GetRootLayer().Save())
print("[worldgen] READY - press Play (or run the scan publisher first on a fresh session)")
