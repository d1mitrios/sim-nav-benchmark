# Proper (non-deprecated) camera throttle for Isaac 6.0:
# - resets frameSkipCount to 0 (deprecated mechanism)
# - sets omni:sensor:tickRate = 5.0 on the camera prim (publish ~5 Hz)
# Run once, saves the stage.
import omni.usd
from pxr import Usd, Sdf

stage = omni.usd.get_context().get_stage()

# 1) reset deprecated frameSkipCount back to 0
for prim in Usd.PrimRange(stage.GetPrimAtPath("/Graph/ROS_Camera")):
    for attr in prim.GetAttributes():
        if "frameskip" in attr.GetName().lower().replace("_", ""):
            attr.Set(0)
            print("reset", prim.GetPath(), attr.GetName(), "-> 0")

# 2) tickRate on the sensor (camera) prim
cam = stage.GetPrimAtPath("/my_custom_robot/Geometry/chassis/camera_link/robot_camera")
a = cam.GetAttribute("omni:sensor:tickRate")
if not a:
    a = cam.CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float)
a.Set(5.0)
print("camera omni:sensor:tickRate =", cam.GetAttribute("omni:sensor:tickRate").Get())
print("saved:", stage.GetRootLayer().Save())
