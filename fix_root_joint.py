import omni.usd, omni.timeline
from pxr import UsdPhysics
st = omni.usd.get_context().get_stage()
rj = st.GetPrimAtPath("/my_custom_robot/Physics/root_joint")
rj.SetActive(False)
print("root_joint IsActive now (want False):", rj.IsActive())
for j in ["left_wheel_joint", "right_wheel_joint"]:
    d = UsdPhysics.DriveAPI.Get(st.GetPrimAtPath("/my_custom_robot/Physics/" + j), "angular")
    d.CreateTargetVelocityAttr(0.0)
    print(j, "targetVelocity reset to 0")
print("saved:", st.GetRootLayer().Save())
tl = omni.timeline.get_timeline_interface()
tl.stop()
tl.play()
print("timeline restarted - root free")
