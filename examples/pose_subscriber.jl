#!/usr/bin/env julia

# pose_subscriber.jl
using ROJ

function int_callback(msg)
    println("Received counter: $(msg.data)")
end

function pose_callback(msg)
    println("Received pose: position=($(msg.position.x), $(msg.position.y), $(msg.position.z))")
end

function main()
    # Creating node
    node = ROSNode("multi_type_subscriber")
    
    # Multiple Subscribers
    int_sub = Subscriber(node, "counter", "std_msgs.msg.Int32", int_callback)
    pose_sub = Subscriber(node, "robot_pose", "geometry_msgs.msg.Pose", pose_callback)
    
    # Starting subscribe loop
    try
        while true
            spin_once(node)
            sleep(0.01)
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