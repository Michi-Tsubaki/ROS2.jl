#!/usr/bin/env julia
# This example demonstrates periodic publishing of multiple message types: Int32 and Pose.
using ROS2

function main()
    node = ROSNode("multi_type_publisher")

    int_pub = Publisher(node, "counter", "std_msgs.msg.Int32")
    pose_pub = Publisher(node, "robot_pose", "geometry_msgs.msg.Pose")

    counter = 1

    try
        while true
            int_msg = create_msg("std_msgs.msg.Int32")
            int_msg.data = counter
            publish(int_pub, int_msg)

            pose_msg = create_msg("geometry_msgs.msg.Pose")
            pose_msg.position.x = sin(counter * 0.1)
            pose_msg.position.y = cos(counter * 0.1)
            pose_msg.position.z = 0.0
            pose_msg.orientation.w = 1.0
            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            publish(pose_pub, pose_msg)

            println(
                "Published: Counter=$(counter), Pose=($(pose_msg.position.x), $(pose_msg.position.y))",
            )
            counter += 1

            spin_once(node)
            sleep(1.0)
        end
    catch e
        if e isa InterruptException
            println("\nShutting down gracefully...")
        else
            rethrow(e)
        end
    finally
        shutdown()
    end
end

main()
