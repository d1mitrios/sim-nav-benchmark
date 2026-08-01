# === One-shot bring-up, executed at Isaac launch via: isaac-sim.bat --exec bootstrap.py ===
# Waits for the app + ros2 bridge, opens robot.usda, removes the old raw-camera graph,
# starts the lidar publisher + JPEG camera publisher, presses Play. Zero UI clicks.
# v2 (29/7): RE-RUN SAFE. A second Run reopened the stage AGAIN -> all the publishers'
# cached prim handles died and the run-once guards blocked the rebinding
# (2-line CSV, zero /scan, error wall). Now: the stage opens ONLY if robot.usda is
# not already open, and the v3+ publishers rebind handles on every PLAY anyway.
import asyncio
import builtins

async def _boot():
    import omni.kit.app
    app = omni.kit.app.get_app()

    # 1) let the app/extensions settle
    for _ in range(30):
        await app.next_update_async()

    # 2) wait until the ros2 bridge's rclpy is importable (autoload)
    for _ in range(300):
        try:
            import rclpy  # noqa
            break
        except ImportError:
            await app.next_update_async()
    print("[boot] rclpy available")

    # 3) open the stage — v2: ONLY if robot.usda is not already open
    import omni.usd
    ctx = omni.usd.get_context()
    cur = ctx.get_stage()
    ident = ""
    try:
        if cur and cur.GetRootLayer():
            ident = cur.GetRootLayer().identifier.replace("\\", "/").lower()
    except Exception:
        pass
    if ident.endswith("robot.usda"):
        print("[boot] robot.usda already open - skipping reopen (re-run safe)")
    else:
        ctx.open_stage("C:/isaac_project/robot.usda")
        for _ in range(60):
            await app.next_update_async()
    stage = ctx.get_stage()
    print("[boot] stage:", stage.GetRootLayer().identifier)

    # 4) drop the old raw-image camera graph (raw 900KB frames killed the WSL link)
    if stage.GetPrimAtPath("/Graph/ROS_Camera"):
        stage.RemovePrim("/Graph/ROS_Camera")
        stage.GetRootLayer().Save()
        print("[boot] removed /Graph/ROS_Camera (raw camera path) + saved")

    # 5) publishers (lidar raycast + jpeg camera) + metrics logger, kept alive via builtins
    #    (all now carry their own run-once guards + rebind-on-PLAY -> the exec is harmless)
    ns = {}
    exec(open("C:/isaac_project/scan_raycast_publisher2.py").read(), ns)
    exec(open("C:/isaac_project/camera_compressed_publisher.py").read(), ns)
    exec(open("C:/isaac_project/metrics_logger.py").read(), ns)   # has its own guard
    exec(open("C:/isaac_project/person_mover.py").read(), ns)     # Phase 2: moving people
    exec(open("C:/isaac_project/odom_publisher.py").read(), ns)   # Phase 3: /odom + TF for SLAM
    if getattr(builtins, "_isaac_bringup_keepalive", None) is None:
        builtins._isaac_bringup_keepalive = ns   # v2: do NOT throw away the old ns on a re-run

    # 6) play
    for _ in range(10):
        await app.next_update_async()
    import omni.timeline
    omni.timeline.get_timeline_interface().play()
    print("[boot] PLAY pressed - all systems go")

asyncio.ensure_future(_boot())
