#!/usr/bin/env julia
# simple_subscriber.jl
# description : sample subscription code
using RobotJuliaClient

function callback(msg)
    println("Subscribed: ", msg.data)
end

function main()
    # Creating node
    node = ROSNode("simple_subscriber")
    
    # Creating subscriber
    sub = Subscriber(node, "hello_topic", "std_msgs.msg.String", callback)
    
    # Starting subscribe loop
    try
        while is_ok()
            # ROS event processing
            spin_once(node, timeout_sec=0.1)
            # wait
            sleep(0.1)
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