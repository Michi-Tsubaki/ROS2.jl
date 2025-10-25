#!/usr/bin/env julia
# This example demonstrates minimal RViz integration with TF and visualization markers.
# In other terminal(bash), run: `ros2 run rviz2 rviz2`

using ROS2

init()
node = ROSNode("rviz_minimal")

static_broadcaster = StaticTransformBroadcaster(node)
broadcaster = TransformBroadcaster(node)
listener = TransformListener(node)
pose_pub = Publisher(node, "robot_pose", "geometry_msgs.msg.PoseStamped")
marker_pub = Publisher(node, "visualization_marker", "visualization_msgs.msg.Marker")

static_transform =
    create_transform_stamped("map", "odom", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
static_transform.header.stamp = to_msg_time(now())
send_static_transform(static_broadcaster, static_transform)

robot_x = Ref(0.0)
robot_y = Ref(0.0)
robot_theta = Ref(0.0)

timer = ROSTimer(
    node,
    0.1,
    () -> begin
        robot_theta[] += 0.1
        robot_x[] = 2.0 * cos(robot_theta[])
        robot_y[] = 2.0 * sin(robot_theta[])

        qz = sin(robot_theta[] / 2.0)
        qw = cos(robot_theta[] / 2.0)

        odom_transform = create_transform_stamped(
            "odom",
            "base_link",
            (robot_x[], robot_y[], 0.0),
            (0.0, 0.0, qz, qw),
        )
        odom_transform.header.stamp = to_msg_time(now())
        send_transform(broadcaster, odom_transform)

        pose_msg = create_msg("geometry_msgs.msg.PoseStamped")
        pose_msg.header.stamp = to_msg_time(now())
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = robot_x[]
        pose_msg.pose.position.y = robot_y[]
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        publish(pose_pub, pose_msg)

        marker_msg = create_msg("visualization_msgs.msg.Marker")
        marker_msg.header.stamp = to_msg_time(now())
        marker_msg.header.frame_id = "base_link"
        marker_msg.type = 2
        marker_msg.action = 0
        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = 0.0
        marker_msg.pose.position.z = 0.5
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.5
        marker_msg.scale.y = 0.5
        marker_msg.scale.z = 1.0
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.id = 0
        publish(marker_pub, marker_msg)

        try
            transform = lookup_transform(listener, "map", "base_link")
            println(
                "Robot at: x=$(round(transform.transform.translation.x, digits=2)), y=$(round(transform.transform.translation.y, digits=2))",
            )
        catch
        end
    end,
)

try
    while true
        spin_once(node)
        sleep(0.05)
    end
finally
    timer_cancel(timer)
    shutdown()
end
