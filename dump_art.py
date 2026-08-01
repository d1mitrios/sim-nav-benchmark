import omni.graph.core as og
n = og.get_node_by_path("/Graph/ROS_DiffDrive/art")
print("NODE:", n.get_prim_path(), "| type:", n.get_type_name(), "| valid:", n.is_valid())
for a in sorted(n.get_attributes(), key=lambda x: x.get_name()):
    nm = a.get_name()
    if nm.startswith("inputs:"):
        try:
            print(nm, "=", og.Controller.get(a))
        except Exception:
            print(nm, "= <unreadable>")
