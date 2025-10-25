#!/usr/bin/env julia
# This example demonstrates periodic publishing of PoseStamped messages with timestamps.

using ROS2

init()

node = ROSNode("time_example")

pose_pub = Publisher(node, "pose_with_time", "geometry_msgs.msg.PoseStamped")

println("Starting periodic publishing at 10 Hz for 1 minute...")

counter = Ref(0)
start_time = time()

timer = ROSTimer(
    node,
    0.1,  # 10 Hz
    () -> begin
        counter[] += 1
        current_time = now()
        msg_time = to_msg_time(current_time)

        pose_msg = create_msg("geometry_msgs.msg.PoseStamped")
        pose_msg.header.stamp = msg_time
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = Float64(counter[])
        pose_msg.pose.position.y = sin(counter[] * 0.1)
        pose_msg.pose.position.z = 0.0
        publish(pose_pub, pose_msg)

        println(
            "Timer callback #$(counter[]) executed - x: $(counter[]), y: $(sin(counter[] * 0.1))",
        )
    end,
)

try
    while (time() - start_time) < 60.0
        spin_once(node)
        sleep(0.1)
    end
    println("Finished publishing messages after 1 minute")
catch e
    println("Error: $e")
finally
    timer_cancel(timer)
    shutdown()
end
