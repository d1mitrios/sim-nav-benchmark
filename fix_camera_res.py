import omni.usd
import omni.timeline
from pxr import Usd
stage = omni.usd.get_context().get_stage()
n = 0
for prim in Usd.PrimRange(stage.GetPrimAtPath("/Graph")):
    w = prim.GetAttribute("inputs:width")
    h = prim.GetAttribute("inputs:height")
    if w and h:
        print("node:", prim.GetPath(), "was", w.Get(), "x", h.Get())
        w.Set(640)
        h.Set(480)
        n += 1
print("nodes updated:", n)
print("saved:", stage.GetRootLayer().Save())
tl = omni.timeline.get_timeline_interface()
tl.stop()
tl.play()
print("timeline restarted at 640x480")
