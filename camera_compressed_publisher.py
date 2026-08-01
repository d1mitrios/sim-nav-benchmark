# === Isaac 6.0.x: robot camera -> ROS 2 /camera_jpeg (sensor_msgs/CompressedImage) ===
# Renders the robot camera via a replicator render product and publishes small JPEGs
# (~30-60 KB) instead of raw 900 KB frames, so the WSL2 NAT link never fragments-storms.
# A relay on the WSL side decompresses back to raw /camera for yolo_navigator.
# v2 (29/7): real run-once guard (builtins) — the double bootstrap created a
# SECOND publisher/render product. The annotator works with a path string (not a cached
# prim handle), so no stage rebind is needed.
import io
import builtins
import omni.kit.app
import omni.replicator.core as rep
import rclpy
from sensor_msgs.msg import CompressedImage

CAM_PATH      = "/my_custom_robot/Geometry/chassis/camera_link/robot_camera"
RESOLUTION    = (640, 480)
PUBLISH_EVERY = 3   # every 3rd rendered frame (~5 Hz at ~15 fps)
JPEG_QUALITY  = 80

if getattr(builtins, "_cam_jpeg_pub", None):
    print("[camera] already running - guard skip")
else:
    try:
        from PIL import Image as PILImage
        _ENC = "pil"
    except ImportError:
        import cv2
        _ENC = "cv2"

    if not rclpy.ok():
        rclpy.init()
    _cam_node = rclpy.create_node("isaac_cam_jpeg_pub")
    _cam_pub  = _cam_node.create_publisher(CompressedImage, "/camera_jpeg", 10)

    _rp    = rep.create.render_product(CAM_PATH, RESOLUTION)
    _annot = rep.AnnotatorRegistry.get_annotator("rgb")
    _annot.attach([_rp])

    _cam_cnt = {"n": 0}

    def _cam_tick(e):
        _cam_cnt["n"] += 1
        if _cam_cnt["n"] % PUBLISH_EVERY != 0:
            return
        data = _annot.get_data()
        if data is None or getattr(data, "size", 0) == 0:
            return
        rgb = data[:, :, :3]
        buf = io.BytesIO()
        if _ENC == "pil":
            PILImage.fromarray(rgb).save(buf, "JPEG", quality=JPEG_QUALITY)
            payload = buf.getvalue()
        else:
            import numpy as _np
            ok, enc = cv2.imencode(".jpg", rgb[:, :, ::-1])  # cv2 wants BGR
            if not ok:
                return
            payload = enc.tobytes()
        msg = CompressedImage()
        msg.header.stamp = _cam_node.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = payload
        _cam_pub.publish(msg)

    _cam_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(_cam_tick)
    builtins._cam_jpeg_pub = {"node": _cam_node, "pub": _cam_pub, "sub": _cam_sub,
                              "annot": _annot, "rp": _rp}
    print("[camera] v2 JPEG publisher running (/camera_jpeg, every %d frames, q%d, enc=%s)"
          % (PUBLISH_EVERY, JPEG_QUALITY, _ENC))
