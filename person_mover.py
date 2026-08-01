# === Person mover — Phase 2 (script-driven walking) — v2: per-waypoint pause ===
# Moves person_N along the seeded waypoint paths from the current world manifest
# (rows: path,person_i,x,y,seq,speed[,pause_s]). Constant walking speed, per-waypoint
# pause (col 7; missing -> PAUSE_S, v6 worldgen writes it explicitly: 0 at door points
# so nobody stands inside a doorway). Yaw faces the walking direction. The lidar_proxy
# is a child of person_N, so the lidar sees the moving person; YOLO sees the mesh.
# Reloads paths on every PLAY (batch runner changes worlds). Run-once guard.
# NOTE: English prints only (Windows console mangles Greek).
import math
import builtins
import omni.usd
import omni.physx
import omni.timeline
from pxr import UsdGeom, Gf

RUNS = "C:/isaac_project/runs"
PAUSE_S = 3.0

if getattr(builtins, "_person_mover", None):
    print("[people] already running - guard skip")
else:
    class _PersonMover:
        def __init__(self):
            self.tl = omni.timeline.get_timeline_interface()
            self.people = []
            self.phys_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
            self.tl_sub = self.tl.get_timeline_event_stream().create_subscription_to_pop(self._on_ev)
            self._load()

        def _load(self):
            self.people = []
            try:
                seed = open(f"{RUNS}/current_seed.txt").read().strip()
                rows = [l.strip().split(",") for l in open(f"{RUNS}/world_{seed}.csv")
                        if l.startswith("path,")]
            except Exception as e:
                print("[people] no paths available:", e)
                return
            if not rows:
                print("[people] manifest has no path rows - people stay static")
                return
            stage = omni.usd.get_context().get_stage()
            byp = {}
            for p in rows:
                pz = float(p[6]) if len(p) > 6 and p[6] else PAUSE_S   # v2: per-wpt pause
                byp.setdefault(p[1], []).append((float(p[2]), float(p[3]), int(p[4]),
                                                 float(p[5]), pz))
            for name, wp in byp.items():
                wp.sort(key=lambda w: w[2])
                if len(wp) < 2:
                    continue
                prim = stage.GetPrimAtPath(f"/Arena/{name}")
                if not prim:
                    continue
                ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
                if len(ops) < 2:
                    continue
                self.people.append(dict(name=name, ops=ops, speed=wp[0][3],
                                        wp=[(w[0], w[1], w[4]) for w in wp],
                                        idx=1, pos=[wp[0][0], wp[0][1]], pause=0.0))
            print(f"[people] mover active: {len(self.people)} walkers "
                  f"({', '.join(p['name'] + '@' + ('%.2f' % p['speed']) for p in self.people)})")

        def _on_ev(self, e):
            if e.type == int(omni.timeline.TimelineEventType.PLAY):
                self._load()   # new world (batch) -> reload paths

        def _on_step(self, dt):
            for p in self.people:
                if p["pause"] > 0.0:
                    p["pause"] -= dt
                    continue
                tx, ty, tpause = p["wp"][p["idx"]]
                dx, dy = tx - p["pos"][0], ty - p["pos"][1]
                d = math.hypot(dx, dy)
                if d < 0.08:
                    p["pause"] = tpause              # v2: pause of the waypoint we just reached
                    p["idx"] = (p["idx"] + 1) % len(p["wp"])
                    continue
                step = min(d, p["speed"] * dt)
                p["pos"][0] += dx / d * step
                p["pos"][1] += dy / d * step
                p["ops"][0].Set(Gf.Vec3d(p["pos"][0], p["pos"][1], 0.0))
                p["ops"][1].Set(math.degrees(math.atan2(dy, dx)))

    builtins._person_mover = _PersonMover()
    print("[people] person mover loaded")
