# === Metrics logger — Benchmark Factory, Phase 1 ===
# Logs pose (~10 Hz) + sim time to one CSV per run in C:/isaac_project/runs/.
# ONE CSV per run: new file on every PLAY, closed on every STOP.
# Name: run_<seed>_<YYYYmmdd_HHMMSS>.csv (seed from runs/current_seed.txt — generate_world writes it).
# Same pattern as scan_raycast_publisher2 (physics step subscription) + timeline events.
# Run-once guard via builtins._metrics_logger -> a double Run is HARMLESS (prints and skips).
# Loaded automatically by bootstrap.py at every launch; manually: Script Editor -> File→Open -> Run.
# v3 (29/7): REBIND stage+prims on every PLAY (and lazily inside the step). The double bootstrap
# reopened the stage -> the cached handles died -> 2-line CSV (headers only) and
# "Accessed schema on invalid prim" on every physics step. Format/columns UNCHANGED (fmt=v2).
import math
import os
import time
import builtins
import omni.usd
import omni.physx
import omni.timeline
from pxr import UsdGeom

POSE_PATH = "/my_custom_robot/Geometry/chassis/lidar_link"  # same prim as the scan publisher (definitely updated)
RUNS_DIR = "C:/isaac_project/runs"
SEED_FILE = RUNS_DIR + "/current_seed.txt"
LOG_EVERY = 6   # 60 Hz physics -> ~10 Hz samples
PERSONS = ["/Arena/person_0", "/Arena/person_1", "/Arena/person_2"]  # v2: log the people too

if getattr(builtins, "_metrics_logger", None):
    print("[metrics] already active — the guard skips the double Run")
else:
    class _MetricsLogger:
        def __init__(self):
            self.tl = omni.timeline.get_timeline_interface()
            self.f = None
            self.path = ""
            self.n = 0
            self.rows = 0
            self.sim_t = 0.0
            self.pxf = []
            self._bind()
            self.phys_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
            self.tl_sub = self.tl.get_timeline_event_stream().create_subscription_to_pop(self._on_tl_event)
            if self.tl.is_playing():
                self._open_run()

        def _bind(self):
            # v3: fresh handles — called at load, on every PLAY, and lazily in the step
            self.stage = omni.usd.get_context().get_stage()
            self.xf = UsdGeom.Xformable(self.stage.GetPrimAtPath(POSE_PATH))
            self.pxf = [UsdGeom.Xformable(self.stage.GetPrimAtPath(p)) for p in PERSONS]

        def _seed(self):
            try:
                return open(SEED_FILE).read().strip()
            except Exception:
                return "unknown"

        def _open_run(self):
            os.makedirs(RUNS_DIR, exist_ok=True)
            self.path = f"{RUNS_DIR}/run_{self._seed()}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            self.f = open(self.path, "w", buffering=1)   # line-buffered: readable mid-run
            self.f.write(f"# seed={self._seed()} wall_start={time.strftime('%Y-%m-%d %H:%M:%S')} fmt=v2\n")
            self.f.write("sim_t,x,y,yaw_deg,p0x,p0y,p1x,p1y,p2x,p2y\n")
            self._bind()   # v3: new world/stage -> new handles
            self.sim_t = 0.0
            self.rows = 0
            print(f"[metrics] new run -> {self.path}")

        def _close_run(self):
            if self.f:
                self.f.close()
                self.f = None
                print(f"[metrics] run over: {self.rows} samples -> {self.path}")

        def _on_tl_event(self, e):
            if e.type == int(omni.timeline.TimelineEventType.PLAY):
                if self.f is None:
                    self._open_run()
            elif e.type == int(omni.timeline.TimelineEventType.STOP):
                self._close_run()

        def _on_step(self, dt):
            self.sim_t += dt
            self.n += 1
            if self.f is None or self.n % LOG_EVERY:
                return
            try:
                m = self.xf.ComputeLocalToWorldTransform(0)
            except Exception:
                try:
                    self._bind()   # v3: self-heal after a stage reopen
                    m = self.xf.ComputeLocalToWorldTransform(0)
                except Exception:
                    return         # prim not available yet - skip the sample
            t = m.ExtractTranslation()
            r = m.ExtractRotationMatrix()
            yaw = math.degrees(math.atan2(r[0][1], r[0][0]))  # local +X -> world (same convention as scan)
            row = f"{self.sim_t:.3f},{float(t[0]):.3f},{float(t[1]):.3f},{yaw:.1f}"
            for px in self.pxf:   # v2: people positions
                try:
                    pt = px.ComputeLocalToWorldTransform(0).ExtractTranslation()
                    row += f",{float(pt[0]):.3f},{float(pt[1]):.3f}"
                except Exception:
                    row += ",0,0"
            self.f.write(row + "\n")
            self.rows += 1

    builtins._metrics_logger = _MetricsLogger()
    print("[metrics] v3 logger active — new CSV + rebind on every Play, closed on every Stop")
