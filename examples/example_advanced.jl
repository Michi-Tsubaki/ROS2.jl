#!/usr/bin/env julia
using ROS2

init()

set_domain_id(1)
println("Domain ID set to: $(get_domain_id())")

node = ROSNode("advanced_simple")

println("\n=== Testing Component Management ===")
println("    To test components, run in another terminal:")
println("   ros2 run rclcpp_components component_container")
println("   #or")
println("   ros2 run rclcpp_components component_container_isolated")

try
    component_manager = ComponentManager("ComponentManager")

    components = list_components(component_manager)
    if components !== nothing
        println(
            "  Component manager connected, loaded components: $(length(components.unique_ids))",
        )
        demo_packages = [
            ("demo_nodes_cpp", "demo_nodes_cpp::Talker"),
            ("demo_nodes_cpp", "demo_nodes_cpp::Listener"),
            ("composition", "composition::Talker"),
            ("composition", "composition::Listener"),
        ]

        loaded_id = nothing
        for (pkg, plugin) in demo_packages
            println("Trying to load: $pkg :: $plugin")
            response = load_component(
                component_manager,
                pkg,
                plugin,
                "julia_loaded_$(replace(plugin, "::" => "_"))",
            )
            if response !== nothing && response.success
                println("Successfully loaded $plugin with ID: $(response.unique_id)")
                loaded_id = response.unique_id
                break
            else
                println("Failed to load $plugin")
            end
        end

        if loaded_id !== nothing
            println("Running with loaded component for 5 seconds...")
            sleep(5.0)

            unload_response = unload_component(component_manager, loaded_id)
            if unload_response !== nothing && unload_response.success
                println("Successfully unloaded component")
            else
                println("Failed to unload component")
            end
        end
    else
        println("No component manager found - continuing with other features")
    end

catch e
    println("Component functionality not available: $e")
end

println("\n=== Regular Advanced Features ===")

executor = MultiThreadedExecutor(2)
add_node(executor, node)

custom_qos = QoSProfile("best_effort", "volatile", "keep_last", 1)
pub = Publisher(node, "advanced_topic", "std_msgs.msg.String")

static_broadcaster = StaticTransformBroadcaster(node)
transform =
    create_transform_stamped("map", "base_link", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
transform.header.stamp = to_msg_time(now())
send_static_transform(static_broadcaster, transform)

counter = Ref(0)
finished = Ref(false)

timer = ROSTimer(
    node,
    0.5,
    () -> begin
        counter[] += 1

        msg = create_msg_dynamic("std_msgs.msg.String")
        msg.data = "Advanced message #$(counter[])"
        publish(pub, msg)

        twist_msg = create_msg_dynamic("geometry_msgs.msg.Twist")
        twist_msg.linear.x = Float64(counter[])

        pose_msg = create_msg_dynamic("geometry_msgs.msg.PoseStamped")
        pose_msg.header.stamp = to_msg_time(now())
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = Float64(counter[])

        available_types = get_available_message_types()
        println("Message #$(counter[]), Available types: $(length(available_types))")

        if counter[] >= 5
            println("Finishing example...")
            finished[] = true
            timer_cancel(timer)
        end
    end,
)

try
    println("Running advanced example with executor...")
    println("Features demonstrated:")
    println("- Multi-threaded executor")
    println("- Custom QoS profiles")
    println("- Dynamic message creation")
    println("- TF2 static transforms")
    println("- Domain ID configuration")
    println("- Component management (if container is running)")

    start_time = time()
    while !finished[] && (time() - start_time) < 10.0
        spin_once(node)
        sleep(0.1)
    end
    println("Advanced example completed successfully!")
catch e
    println("Error: $e")
finally
    try
        timer_cancel(timer)
    catch
    end
    shutdown_executor(executor)
    shutdown()
end
