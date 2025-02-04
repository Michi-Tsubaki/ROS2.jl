module Core

using PyCall

# Global variables to store Python modules
const rclpy = PyNULL()
const rclpy_node = PyNULL()

function __init__()
    if contains(lowercase(get(ENV, "GITHUB_WORKFLOW", "")), "automerge")
        return
    end

    try
        if !haskey(ENV, "AMENT_PREFIX_PATH")
            @warn "ROS2 environment not sourced"
            return
        end

        # Import rclpy and node
        py_sys = pyimport("sys")
        ros_python_path = "/opt/ros/humble/lib/python3.10/site-packages"
        
        if ros_python_path not in py_sys."path"
            pushfirst!(py_sys."path", ros_python_path)
        end
        
        copy!(rclpy, pyimport("rclpy"))
        copy!(rclpy_node, pyimport("rclpy.node"))
        
    catch e
        @warn "ROS2 initialization deferred: $e"
    end
end

# Basic Node wrapper
mutable struct ROSNode
    pynode::PyObject
    
    function ROSNode(node_name::String; context=nothing)
        try
            if !rclpy.ok()
                # println("Initializing rclpy...") # for debug
                rclpy.init(context=context)
            end
            # println("Creating Node object...") # for debug
            pynode = rclpy_node.Node(node_name, context=context)
            # println("Node created successfully") # for debug
            return new(pynode)
        catch e
            println("Error creating ROS node: ", e)
            println("Exception type: ", typeof(e))
            println("Stacktrace: ", stacktrace())
            rethrow(e)
        end
    end
end

# Initialization and shutdown
function init(; args=nothing)
    if !rclpy.ok()
        rclpy.init(args=args)
    end
end

function shutdown()
    if rclpy.ok()
        rclpy.shutdown()
    end
end

# Spinning functions
function spin_once(node::ROSNode; timeout_sec=nothing)
    #println("Spinning once...")
    try
        if timeout_sec === nothing
            timeout_sec = 0.1  # デフォルトのタイムアウトを設定
        end
        rclpy.spin_once(node.pynode, timeout_sec=timeout_sec)
        #println("Spin complete")
        return true
    catch e
        println("Error during spin: ", e)
        return false
    end
end

function spin(node::ROSNode)
    rclpy.spin(node.pynode)
end

# ROS status check
function is_ok()
    return rclpy.ok()
end

export ROSNode, init, shutdown, spin, spin_once, is_ok

end # module