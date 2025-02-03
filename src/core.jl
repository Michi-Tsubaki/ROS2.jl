module Core

using PyCall

# Global variables to store Python modules
const rclpy = PyNULL()
const rclpy_node = PyNULL()

function __init__()
    # println("Initializing Core module...") # for debug
    try
        # Check ROS2 environment
        if !haskey(ENV, "AMENT_PREFIX_PATH")
            error("ROS2 environment not sourced. Please run 'source /opt/ros/\$ROS_DISTRO/setup.bash' first")
        end
        # println("AMENT_PREFIX_PATH: ", ENV["AMENT_PREFIX_PATH"]) # for debug
        
        # Get Python executable path
        python_path = PyCall.python
        # println("Using Python: ", python_path) # for debug
        
        # Add ROS2 Python path
        ros_python_path = "/opt/ros/jazzy/lib/python3.12/site-packages"
        # println("Adding to Python path: ", ros_python_path) for debug
        py"""
        import sys
        ros_path = $ros_python_path
        if ros_path not in sys.path:
            sys.path.insert(0, ros_path)
        """
        
        # Import rclpy
        copy!(rclpy, pyimport("rclpy"))
        copy!(rclpy_node, pyimport("rclpy.node"))
    catch e
        println("Error during RobotJuliaClient initialization: ", e)
        println("Exception type: ", typeof(e))
        println("Stacktrace: ", stacktrace())
        rethrow(e)
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