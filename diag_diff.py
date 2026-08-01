import omni.graph.core as og
import omni.usd
from pxr import UsdPhysics

for ap in ["/Graph/ROS_DiffDrive/twist.outputs:linearVelocity",
           "/Graph/ROS_DiffDrive/twist.outputs:angularVelocity",
           "/Graph/ROS_DiffDrive/diff.outputs:velocityCommand",
           "/Graph/ROS_DiffDrive/art.inputs:robotPath",
           "/Graph/ROS_DiffDrive/art.inputs:jointNames"]:
    try:
        print(ap.split("/")[-1], "=", og.Controller.get(og.Controller.attribute(ap)))
    except Exception as e:
        print(ap, "ERROR:", e)

st = omni.usd.get_context().get_stage()
roots = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("ArticulationRootAPI prims:", roots)
for p in ["/my_custom_robot", "/my_custom_robot/Physics/root_joint"]:
    prim = st.GetPrimAtPath(p)
    print(p, "| valid:", bool(prim), "| artroot:", prim.HasAPI(UsdPhysics.ArticulationRootAPI) if prim else "-")
