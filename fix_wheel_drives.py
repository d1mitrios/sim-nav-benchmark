import omni.usd, omni.timeline
from pxr import UsdPhysics
st = omni.usd.get_context().get_stage()
for j in ["left_wheel_joint", "right_wheel_joint"]:
    p = st.GetPrimAtPath("/my_custom_robot/Physics/" + j)
    d = UsdPhysics.DriveAPI.Get(p, "angular")
    if not d:
        print(j, ": NO DriveAPI -> applying")
        d = UsdPhysics.DriveAPI.Apply(p, "angular")
    print(j, "BEFORE stiff:", d.GetStiffnessAttr().Get(), "| damp:", d.GetDampingAttr().Get(), "| maxF:", d.GetMaxForceAttr().Get())
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(100000.0)
    d.CreateMaxForceAttr(1000000.0)
    print(j, "AFTER  stiff:", d.GetStiffnessAttr().Get(), "| damp:", d.GetDampingAttr().Get(), "| maxF:", d.GetMaxForceAttr().Get())
print("saved:", st.GetRootLayer().Save())
tl = omni.timeline.get_timeline_interface()
tl.stop()
tl.play()
print("timeline restarted")
