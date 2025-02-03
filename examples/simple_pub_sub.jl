#!/usr/bin/env julia

# simple_pub_sub.jl
using ROS2

function main()
    # Creating node
    node = ROSNode("test_node")
    
    # Creating publisher
    pub = Publisher(node, "test_topic", "std_msgs.msg.String")
    
    # Creating message
    msg = create_msg("std_msgs.msg.String")
    msg.data = "Hello from Julia!"
    
    println("Publishing message...")
    publish(pub, msg)
    
    # Wait
    for i in 1:10
        spin_once(node)
        sleep(0.1)
    end
    
    shutdown()
    println("Done!")
end

main()