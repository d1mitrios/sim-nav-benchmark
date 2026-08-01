# Reduces RTX render load for stability on the 2080 Ti (below 6.0 tested min-spec).
# Run once per session, anytime after opening the stage. No save needed (session settings).
import carb.settings
import omni.kit.viewport.utility as vu

vp = vu.get_active_viewport()
print("viewport was:", vp.resolution)
vp.resolution = (640, 480)
print("viewport now:", vp.resolution)

s = carb.settings.get_settings()
s.set("/rtx/post/dlss/execMode", 0)  # 0 = performance
print("DLSS -> performance mode")
print("done - lighter render load")
