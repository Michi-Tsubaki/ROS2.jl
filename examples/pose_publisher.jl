#!/usr/bin/env julia

# pose_publisher.jl
using ROS2

function main()
    # Creating node
    node = ROSNode("multi_type_publisher")
    
    # Multiple publisher
    int_pub = Publisher(node, "counter", "std_msgs.msg.Int32")
    pose_pub = Publisher(node, "robot_pose", "geometry_msgs.msg.Pose")
    
    counter = 1
    
    # Starting publish loop
    try
        while true
            # Int32 msg
            int_msg = create_msg("std_msgs.msg.Int32")
            int_msg.data = counter
            publish(int_pub, int_msg)
            
            # Pose msg
            pose_msg = create_msg("geometry_msgs.msg.Pose")
            pose_msg.position.x = sin(counter * 0.1)
            pose_msg.position.y = cos(counter * 0.1)
            pose_msg.position.z = 0.0
            pose_msg.orientation.w = 1.0
            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            publish(pose_pub, pose_msg)
            
            println("Published: Counter=$(counter), Pose=($(pose_msg.position.x), $(pose_msg.position.y))")
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