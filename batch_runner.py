# === Batch runner — Benchmark Factory, Phase 1 ===
# Automates the full experiment loop: STOP -> new seed -> generate_world -> PLAY,
# for N_RUNS runs x RUN_MINUTES each. The metrics_logger (already loaded by bootstrap)
# opens/closes one CSV per run automatically on the PLAY/STOP events, and
# generate_world writes the manifest + current_seed.txt per world. Zero clicks.
#
# Seeds are REPRODUCIBLE: SEED_BASE+1 .. SEED_BASE+N_RUNS (written to world_seed.txt
# before each run). The same list re-runs later with yolo_navigator v1 for the
# baseline comparison. After the batch, world_seed.txt is removed (manual runs
# go back to random seeds).
#
# Start:  Script Editor -> File->Open -> THIS file -> Run (once). Play/Stop handled here.
# Abort:  create the file C:/isaac_project/runs/batch_stop.txt (any content).
# Guard:  builtins._batch_runner - a second Run while active just prints and exits.
# NOTE:   English prints only (the Windows console mangles Greek as '???').
import asyncio
import builtins
import os
import time

N_RUNS = 25                    # mission batch
START_RUN = 1                  # PHASE B: full paired rerun with amcl v9 (sparse-map tuning)
RUN_MINUTES = 20.0             # (ignored in NAV_MISSIONS mode)
SEED_BASE = 20260723000        # run i uses seed SEED_BASE+i
STOP_FILE = "C:/isaac_project/runs/batch_stop.txt"
SEED_FILE = "C:/isaac_project/world_seed.txt"
GEN = "C:/isaac_project/generate_world.py"
SUMMARY = "C:/isaac_project/runs/batch_log.csv"
RUNS_DIR = "C:/isaac_project/runs"
# v3: auto-end a run on TERMINAL deadlock — SPREAD of the trail (diagonal of the
# bounding box of ALL window positions) < STUCK_NET_M over STUCK_WINDOW_S wall-s.
# Why it changed: v2 compared only the two ENDPOINTS of the trail (two-point net) and fired
# falsely when the robot came back near an old position while moving — 16/25 pseudo-kills in the
# dynamic batch, 24/7 (all healthy: path 19-26 m, spread 4.8-12.6 m in the final window).
# Spread catches only REAL immobility (the true v1 deadlocks have spread ~0).
# Extra guard: sample the trail only while the sim advances — in the 2080 Ti freezes the
# wall clock runs while the sim does not, and that must not count as a deadlock.
STUCK_WINDOW_S = 180.0
STUCK_NET_M = 0.5
POSE_PATH = "/my_custom_robot/Geometry/chassis/lidar_link"
# v4: mapping batch — at the end of each run a signal is written for the WSL slam_batch.sh
# (it saves the map BEFORE the world changes). MAPPING_SAVE=False for pure nav batches
# (otherwise each run waits up to 30s for the signal to be consumed).
MAPPING_SAVE = False
SAVE_FLAG = "C:/isaac_project/runs/save_map_now.txt"
# v5: NAV mission batch (Step 2) — the world stays alive until the WSL nav_batch.sh
# writes missions_done_<seed>.txt (end of the mission_runner v3 tour), otherwise
# safety cap NAV_MAX_MINUTES (in case the WSL side dies). NO stuck-watchdog here:
# under Nav2 immobility is a GIVEN (correct refusals of impassable doors, human-blocks,
# waiting between goals), not a sim deadlock. The cap (45) > worst tour
# (4 goals x 2 attempts x ~275s + gaps ~ 39 min) so the world never changes
# under a live tour (wrong-universe lesson, 29/7).
NAV_MISSIONS = True
NAV_MAX_MINUTES = 45.0
# v5.1: the WSL trigger via mtime of current_seed.txt does NOT work (9p attribute cache
# on unchanged content) — we write an explicit nav_go.txt signal (=seed) before each PLAY,
# like the proven save_map_now.txt handshake (25/25 in the mapping batch).
NAV_GO = RUNS_DIR + "/nav_go.txt"

if getattr(builtins, "_batch_runner", None):
    print("[batch] already running - guard skip (create runs/batch_stop.txt to abort)")
else:
    async def _batch():
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from pxr import UsdGeom
        app = omni.kit.app.get_app()
        tl = omni.timeline.get_timeline_interface()
        try:
            os.remove(STOP_FILE)   # v2: clear old stop file at startup
        except OSError:
            pass
        if NAV_MISSIONS:
            for fn in os.listdir(RUNS_DIR):   # v5: stale done-flags from old batches
                if fn.startswith("missions_done_"):
                    try:
                        os.remove(RUNS_DIR + "/" + fn)
                    except OSError:
                        pass
            try:
                os.remove(NAV_GO)             # v5.1: stale go-signal from an old batch
            except OSError:
                pass
        if not os.path.exists(SUMMARY):
            open(SUMMARY, "a").write("seed,end_reason,wall_seconds\n")
        print(f"[batch] START: runs {START_RUN}..{N_RUNS}, "
              f"seeds {SEED_BASE + START_RUN}..{SEED_BASE + N_RUNS}")
        if NAV_MISSIONS:
            print(f"[batch] NAV mode: run ends on missions_done_<seed>.txt "
                  f"(cap {NAV_MAX_MINUTES:.0f} min), stuck-watchdog OFF")
        else:
            print(f"[batch] {RUN_MINUTES:.0f} min max per run, deadlock rule: "
                  f"spread<{STUCK_NET_M}m / {STUCK_WINDOW_S:.0f}s (sim-gated)")
        done = 0
        for i in range(START_RUN, N_RUNS + 1):
            if os.path.exists(STOP_FILE):
                print("[batch] stop file found - aborting before run", i)
                break
            seed = SEED_BASE + i
            tl.stop()
            for _ in range(30):
                await app.next_update_async()
            open(SEED_FILE, "w").write(str(seed))
            try:
                exec(open(GEN).read(), {})
            except Exception as e:
                print(f"[batch] worldgen FAILED for seed {seed}: {e} - skipping run")
                continue
            for _ in range(10):
                await app.next_update_async()
            flag = f"{RUNS_DIR}/missions_done_{seed}.txt"
            if NAV_MISSIONS:
                try:
                    os.remove(flag)   # v5: never a stale flag before PLAY
                except OSError:
                    pass
                open(NAV_GO, "w").write(str(seed))   # v5.1: explicit go-signal to WSL
            tl.play()
            t0 = time.time()
            if NAV_MISSIONS:
                print(f"[batch] run {i}/{N_RUNS} seed={seed} PLAYING "
                      f"(nav tour, cap {NAV_MAX_MINUTES:.0f} min)")
                end_reason = "nav_cap"
                while time.time() - t0 < NAV_MAX_MINUTES * 60.0:
                    if os.path.exists(STOP_FILE):
                        print("[batch] stop file found - ending current run early")
                        end_reason = "stop_file"
                        break
                    if os.path.exists(flag):
                        end_reason = "missions_done"
                        try:
                            os.remove(flag)   # ACK to WSL
                        except OSError:
                            pass
                        print(f"[batch] missions done for seed {seed} "
                              f"({time.time() - t0:.0f}s)")
                        break
                    for _ in range(60):
                        await app.next_update_async()
                if end_reason == "nav_cap":
                    print(f"[batch] WARN: nav cap hit for seed {seed} - "
                          f"is nav_batch.sh running on the WSL side?")
            else:
                print(f"[batch] run {i}/{N_RUNS} seed={seed} PLAYING "
                      f"(max {RUN_MINUTES:.0f} min)")
                stage = omni.usd.get_context().get_stage()
                xf = UsdGeom.Xformable(stage.GetPrimAtPath(POSE_PATH))
                trail = []
                last_sim = -1.0
                end_reason = "full"
                while time.time() - t0 < RUN_MINUTES * 60.0:
                    if os.path.exists(STOP_FILE):
                        print("[batch] stop file found - ending current run early")
                        end_reason = "stop_file"
                        break
                    for _ in range(60):
                        await app.next_update_async()
                    m = xf.ComputeLocalToWorldTransform(0)
                    tr = m.ExtractTranslation()
                    now = time.time()
                    sim_now = tl.get_current_time()
                    if sim_now != last_sim:   # v3: watchdog counts only while the sim advances
                        last_sim = sim_now
                        trail.append((now, float(tr[0]), float(tr[1])))
                    trail = [p for p in trail if now - p[0] <= STUCK_WINDOW_S + 10.0]
                    if trail and now - t0 > STUCK_WINDOW_S and now - trail[0][0] >= STUCK_WINDOW_S - 15.0:
                        xs = [p[1] for p in trail]
                        ys = [p[2] for p in trail]
                        spread = ((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2) ** 0.5
                        if spread < STUCK_NET_M:
                            end_reason = "terminal_deadlock"
                            print(f"[batch] TERMINAL DEADLOCK after {now - t0:.0f}s "
                                  f"(spread {spread:.2f}m/{STUCK_WINDOW_S:.0f}s) - skipping rest of run")
                            break
            if MAPPING_SAVE:
                # v4: keep the world alive until WSL saves the map (max 30s)
                open(SAVE_FLAG, "w").write(str(seed))
                t_save = time.time()
                while time.time() - t_save < 30.0 and os.path.exists(SAVE_FLAG):
                    for _ in range(30):
                        await app.next_update_async()
                if os.path.exists(SAVE_FLAG):
                    print(f"[batch] WARN: map-save signal not consumed (seed {seed})")
                    try:
                        os.remove(SAVE_FLAG)
                    except OSError:
                        pass
                else:
                    print(f"[batch] map saved for seed {seed}")
            open(SUMMARY, "a").write(f"{seed},{end_reason},{time.time() - t0:.0f}\n")
            done += 1
        tl.stop()
        for _ in range(20):
            await app.next_update_async()
        try:
            os.remove(SEED_FILE)   # manual worldgen runs go back to random seeds
        except OSError:
            pass
        builtins._batch_runner = None
        print(f"[batch] DONE - {done} runs completed. CSVs + manifests in C:/isaac_project/runs/")

    builtins._batch_runner = asyncio.ensure_future(_batch())
    print("[batch] scheduled - taking control of the timeline now")
