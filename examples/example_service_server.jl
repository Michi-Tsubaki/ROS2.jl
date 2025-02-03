#!/usr/bin/env julia
# example_service_server.jl
using RobotOperatingClient
using PyCall

function add_service_callback(request, response)
    println("Received request: a=$(request.a), b=$(request.b)")
    
    response.sum = request.a + request.b
    
    println("Sending response: sum=$(response.sum)")
    return response
end

function main()
    println("Creating node...")
    node = ROSNode("adder_server")
    
    println("Creating service server...")
    server = ServiceServer(node, "add_two_ints", "example_interfaces.srv.AddTwoInts", add_service_callback)
    
    println("Service ready...")
    try
        while is_ok()
            spin_once(node)
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
        println("Shutting down...")
        shutdown()
    end
end

main()