import omni.usd
import omni.timeline
import omni.graph.core as og

stage = omni.usd.get_context().get_stage()
GRAPH = "/Graph/ROS_DiffDrive"
keys = og.Controller.Keys

(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("lin", "omni.graph.nodes.BreakVector3"),
            ("ang", "omni.graph.nodes.BreakVector3"),
            ("diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("art", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "twist.inputs:execIn"),
            ("ctx.outputs:context", "twist.inputs:context"),
            ("twist.outputs:execOut", "diff.inputs:execIn"),
            ("twist.outputs:linearVelocity", "lin.inputs:tuple"),
            ("twist.outputs:angularVelocity", "ang.inputs:tuple"),
            ("lin.outputs:x", "diff.inputs:linearVelocity"),
            ("ang.outputs:z", "diff.inputs:angularVelocity"),
            ("diff.outputs:velocityCommand", "art.inputs:velocityCommand"),
            ("tick.outputs:tick", "art.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("twist.inputs:topicName", "cmd_vel"),
            ("diff.inputs:wheelDistance", 0.35),
            ("diff.inputs:wheelRadius", 0.05),
            ("art.inputs:robotPath", "/my_custom_robot"),
            ("art.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
        ],
    },
)
print("diff-drive graph built at", GRAPH)
print("saved:", stage.GetRootLayer().Save())

tl = omni.timeline.get_timeline_interface()
tl.stop()
tl.play()
print("timeline restarted - diff drive live")
