# === One-shot bring-up, executed at Isaac launch via: isaac-sim.bat --exec bootstrap.py ===
# Waits for the app + ros2 bridge, opens robot.usda, removes the raw-camera graph if the
# stage has one, starts the lidar, JPEG camera, metrics, walker and odometry scripts and
# presses Play. Zero UI clicks. Safe to run twice: the stage is opened only if robot.usda is
# not already open, and every publisher rebinds its prim handles on each Play.
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

    # 3) open the stage, only if robot.usda is not already open
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
        ctx.open_stage("C:/isaac_project/isaac/robot.usda")
        for _ in range(60):
            await app.next_update_async()
    stage = ctx.get_stage()
    print("[boot] stage:", stage.GetRootLayer().identifier)

    # 4) remove the raw-image camera graph if present (raw 900 KB frames overloaded the WSL link)
    if stage.GetPrimAtPath("/Graph/ROS_Camera"):
        stage.RemovePrim("/Graph/ROS_Camera")
        stage.GetRootLayer().Save()
        print("[boot] removed /Graph/ROS_Camera (raw camera path) + saved")

    # 5) publishers (lidar raycast + jpeg camera), metrics logger, walkers and odometry, kept
    #    alive via builtins. Each has its own run-once guard and rebinds on Play.
    ns = {}
    exec(open("C:/isaac_project/isaac/scan_raycast_publisher2.py").read(), ns)
    exec(open("C:/isaac_project/isaac/camera_compressed_publisher.py").read(), ns)
    exec(open("C:/isaac_project/isaac/metrics_logger.py").read(), ns)   # has its own guard
    exec(open("C:/isaac_project/isaac/person_mover.py").read(), ns)     # walkers (experiment 2 onward)
    exec(open("C:/isaac_project/isaac/odom_publisher.py").read(), ns)   # /odom + TF for SLAM (experiment 3 onward)
    if getattr(builtins, "_isaac_bringup_keepalive", None) is None:
        builtins._isaac_bringup_keepalive = ns   # keep the existing namespace on a re-run

    # 6) play
    for _ in range(10):
        await app.next_update_async()
    import omni.timeline
    omni.timeline.get_timeline_interface().play()
    print("[boot] PLAY pressed - all systems go")

asyncio.ensure_future(_boot())
