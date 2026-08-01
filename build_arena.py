# Recreates dynamic_world.sdf in Isaac at /Arena (root level, outside the
# /my_custom_robot self-filter). All prims get colliders. Also adds lights
# (needed for the camera/YOLO) and removes /TestCube (it sits inside the maze).
# Run ONCE on a fresh robot.usda stage, BEFORE pressing Play. Saves at the end.
import math
import omni.usd
from pxr import UsdGeom, UsdPhysics, UsdLux, Gf

stage = omni.usd.get_context().get_stage()

def box(name, pos, size, yaw_rad=0.0, color=(0.6, 0.6, 0.6)):
    c = UsdGeom.Cube.Define(stage, "/Arena/" + name)
    c.GetSizeAttr().Set(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if yaw_rad:
        xf.AddRotateZOp().Set(yaw_rad * 180.0 / math.pi)
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

def cyl(name, pos, radius, height, color=(0.6, 0.6, 0.6)):
    c = UsdGeom.Cylinder.Define(stage, "/Arena/" + name)
    c.GetRadiusAttr().Set(radius)
    c.GetHeightAttr().Set(height)
    c.GetAxisAttr().Set("Z")
    c.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2.0),
                        Gf.Vec3f(radius, radius, height / 2.0)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

UsdGeom.Xform.Define(stage, "/Arena")

# ---- boundary walls (20x20 arena) ----
box("boundary_north", (0, 10, 1), (20, 0.5, 2))
box("boundary_south", (0, -10, 1), (20, 0.5, 2))
box("boundary_east", (10, 0, 1), (0.5, 20, 2))
box("boundary_west", (-10, 0, 1), (0.5, 20, 2))

# ---- obstacles ----
box("obs_box_1", (4, 4, 1), (2, 2, 2), 0.0, (0.8, 0.2, 0.2))
box("obs_box_2", (4, -4, 1), (3, 1, 2), 0.785, (0.2, 0.8, 0.2))
box("obs_box_3", (-5, 5, 1), (1, 4, 2), 0.0, (0.2, 0.2, 0.8))
cyl("obs_cyl_1", (0, 6, 1), 1.5, 2, (0.8, 0.8, 0.2))
box("obs_box_4", (8, 6, 1), (2, 1, 2), 0.0, (0.7, 0.3, 0.3))
box("obs_box_5", (-8, -2, 1), (1, 4, 2), 0.5, (0.3, 0.7, 0.3))
cyl("obs_cyl_3", (2, -8, 1), 1.2, 2, (0.3, 0.3, 0.7))
box("obs_box_6", (2, 2, 1), (1, 1, 2), 0.3, (0.8, 0.5, 0.2))
box("obs_box_7", (-2, -2, 1), (1.5, 1.5, 2), -0.3, (0.2, 0.8, 0.5))
box("obs_box_8", (7, -4, 1), (3, 0.5, 2), 0.8, (0.5, 0.2, 0.8))
cyl("obs_cyl_4", (-7, 8, 1), 0.8, 2, (0.9, 0.9, 0.1))
cyl("obs_cyl_5", (-3, 8, 1), 0.5, 2, (0.1, 0.9, 0.9))
box("obs_box_9", (0, -3, 1), (2, 0.5, 2), 1.57, (0.5, 0.5, 0.5))
box("obs_box_10", (6, 0, 1), (1, 1, 2), 0.0, (1.0, 0.4, 0.7))

# ---- maze / narrow passages ----
box("maze_wall_1", (3, 0.6, 1), (2, 0.4, 2), 0.0, (1, 1, 0))
box("maze_wall_2", (3, -0.6, 1), (2, 0.4, 2), 0.0, (1, 1, 0))
box("maze_wall_3", (6, -1.0, 1), (1.5, 3.0, 2), 0.0, (1, 0.5, 0))
box("maze_wall_4", (6, 2.2, 1), (1.5, 0.4, 2), 0.0, (1, 0, 1))
box("maze_wall_5", (6, 1.1, 1), (1.5, 0.4, 2), 0.0, (1, 0, 1))

# ---- lights (sun-like distant + soft dome, for the camera/YOLO) ----
sun = UsdLux.DistantLight.Define(stage, "/Arena/SunLight")
sun.CreateIntensityAttr(3000.0)
sun.CreateAngleAttr(1.0)
sxf = UsdGeom.Xformable(sun.GetPrim())
sxf.ClearXformOpOrder()
sxf.AddRotateXYZOp().Set(Gf.Vec3f(-30.0, 10.0, 0.0))
dome = UsdLux.DomeLight.Define(stage, "/Arena/DomeLight")
dome.CreateIntensityAttr(400.0)

# ---- remove the old test cube (sits inside the maze corridor) ----
if stage.GetPrimAtPath("/TestCube"):
    stage.RemovePrim("/TestCube")
    print("removed /TestCube")

n = len([p for p in stage.GetPrimAtPath("/Arena").GetChildren()])
print("Arena prims created:", n, "(expect 25: 4 walls + 14 obstacles + 5 maze + 2 lights)")
print("saved:", stage.GetRootLayer().Save())
print("Next: run scan publisher ONCE, then press Play.")
