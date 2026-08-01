# Throttles the /camera publish rate (target ~4-5 Hz) to stop the UDP fragment
# storm over the WSL2 NAT that was collapsing both /camera and /scan.
# Finds a frameSkip-style input on the ROS_Camera graph and sets it to 2
# (= publish every 3rd rendered frame). Saves the stage. Run once, anytime.
import omni.usd
from pxr import Usd

stage = omni.usd.get_context().get_stage()
found = False
for prim in Usd.PrimRange(stage.GetPrimAtPath("/Graph/ROS_Camera")):
    for attr in prim.GetAttributes():
        n = attr.GetName()
        if "frameskip" in n.lower().replace("_", ""):
            print("FOUND:", prim.GetPath(), n, "=", attr.Get())
            attr.Set(2)
            print(" -> set to 2 (publish every 3rd frame)")
            found = True

if not found:
    print("No frameSkip attribute found. Camera-helper inputs for manual inspection:")
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/Graph/ROS_Camera")):
        ins = [a.GetName() for a in prim.GetAttributes() if a.GetName().startswith("inputs:")]
        if ins:
            print(" ", prim.GetPath(), "->", ins)

print("saved:", stage.GetRootLayer().Save())
