#!/usr/bin/env julia
# simple_publisher.jl
# description: sample publishing code

using ROS2

function main()
    # Creating node
    node = ROSNode("simple_publisher")
    
    # Creating publisher
    pub = Publisher(node, "hello_topic", "std_msgs.msg.String")
    
    counter = 1
    
    # Starting publish loop
    try
        while is_ok()
            msg = create_msg("std_msgs.msg.String")
            msg.data = "Hello $(counter)"
            publish(pub, msg)
            
            println("Published: Hello $(counter)")
            
            # ROS event processing
            spin_once(node, timeout_sec=0.1)
            
            counter += 1
            sleep(1.0)
        end
    catch e
        if e isa InterruptException
            println("\nShutting down gracefully...")
        else
            println("Unexpected error: ", e)
            rethrow(e)
        end
    finally
        println("Initiating shutdown...")
        shutdown()
        println("Shutdown complete")
    end
end

main()