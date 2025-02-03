#!/usr/bin/env julia
# example_service_client.jl

using ROS2

function main()
    println("Creating node...")
    node = ROSNode("adder_client")
    
    println("Creating service client...")
    client = ServiceClient(node, "add_two_ints", "example_interfaces.srv.AddTwoInts")
    
    # Wait
    println("Waiting for service...")
    if !wait_for_service(client, timeout_sec=5.0)
        println("Service not available!")
        return
    end
    
    println("Service found!")
    
    try
        while is_ok()
            # Request
            request = create_request(client)
            request.a = rand(1:100)
            request.b = rand(1:100)
            
            println("\nSending request: a=$(request.a), b=$(request.b)")
            
            # Call
            response = call(client, request)
            println("Got response: sum=$(response.sum)")
            
            sleep(2.0)
        end
    catch e
        if e isa InterruptException
            println("\nShutting down gracefully...")
        else
            println("Unexpected error: ", e)
            rethrow(e)
        end
    finally
        println("Shutting down...")
        shutdown()
    end
end

main()