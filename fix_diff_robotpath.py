import omni.graph.core as og
import omni.usd, omni.timeline
og.Controller.set(og.Controller.attribute("/Graph/ROS_DiffDrive/art.inputs:robotPath"), "/my_custom_robot/Geometry/chassis")
print("robotPath now:", og.Controller.get(og.Controller.attribute("/Graph/ROS_DiffDrive/art.inputs:robotPath")))
print("saved:", omni.usd.get_context().get_stage().GetRootLayer().Save())
tl = omni.timeline.get_timeline_interface()
tl.stop()
tl.play()
print("timeline restarted")
