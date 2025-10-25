#!/usr/bin/env julia
# This example demonstrates a simple publisher that sends a String message.
using ROS2

function main()
    node = ROSNode("test_node")
    pub = Publisher(node, "test_topic", "std_msgs.msg.String")
    msg = create_msg("std_msgs.msg.String")
    msg.data = "Hello from Julia!"
    println("Publishing message...")
    publish(pub, msg)
    for i = 1:10
        spin_once(node)
        sleep(0.1)
    end
    shutdown()
    println("Done!")
end

main()
