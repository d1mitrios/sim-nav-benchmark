import omni.usd, omni.timeline
from pxr import UsdPhysics, UsdShade
st = omni.usd.get_context().get_stage()

def make_mat(path, sf, df):
    m = UsdShade.Material.Define(st, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(sf)
    api.CreateDynamicFrictionAttr(df)
    api.CreateRestitutionAttr(0.0)
    return m

slip = make_mat("/PhysicsMaterials/CasterSlip", 0.0, 0.0)
grip = make_mat("/PhysicsMaterials/WheelGrip", 1.2, 1.0)

for path, m in [("/my_custom_robot/Geometry/chassis/caster_wheel", slip),
                ("/my_custom_robot/Geometry/chassis/left_wheel", grip),
                ("/my_custom_robot/Geometry/chassis/right_wheel", grip)]:
    p = st.GetPrimAtPath(path)
    UsdShade.MaterialBindingAPI.Apply(p).Bind(m, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")
    print("bound", str(m.GetPath()), "->", path)

print("saved:", st.GetRootLayer().Save())
tl = omni.timeline.get_timeline_interface()
tl.stop()
tl.play()
print("timeline restarted - friction fixed")
