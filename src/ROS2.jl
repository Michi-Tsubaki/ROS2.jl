module ROS2
using PyCall

const rclpy = PyNULL()
const rclpy_node = PyNULL()

function __init__()
    try
        # Check ROS2 environment
        if !haskey(ENV, "AMENT_PREFIX_PATH")
            error("ROS2 environment not sourced. Please run 'source /opt/ros/\$ROS_DISTRO/setup.bash' first")
        end
        
        # Get Python executable path
        python_path = PyCall.python
        
        # Add ROS2 Python path
        ros_python_path = "/opt/ros/jazzy/lib/python3.12/site-packages"
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
        println("Error during ROS2 initialization: ", e)
        println("Exception type: ", typeof(e))
        println("Stacktrace: ", stacktrace())
        rethrow(e)
    end
end

# Include submodules
include("core.jl")
include("pubsub.jl")
include("timer.jl")
include("service.jl")
include("parameter.jl")
include("action.jl")
include("logging.jl")

# Re-export from submodules
using .Core
using .PubSub
using .Timer
using .Service
using .Parameter
using .Action
using .Logging

# Export all symbols from submodules with the updated function names
export ROSNode, init, shutdown, spin, spin_once, is_ok,  # from Core
       Publisher, Subscriber, create_msg, publish,  # from PubSub
       ROSTimer, timer_cancel, timer_reset, timer_is_ready, timer_time_since_last_call,  # from Timer
       ServiceServer, ServiceClient, create_request, call, call_async,  # from Service
       wait_for_service, service_is_ready,
       declare_parameter, declare_parameters, get_parameter, set_parameter,  # from Parameter
       has_parameter, get_parameter_types,
       add_on_set_parameters_callback, remove_on_set_parameters_callback,
       ActionServer, ActionClient, GoalHandle, send_goal, send_goal_sync,  # from Action
       cancel_goal, create_goal, accept_goal, reject_goal,
       publish_feedback, succeed, abort,
       get_logger, debug, info, warn, log_error, fatal, set_level,  # from Logging
       DEBUG, INFO, WARN, ERROR, FATAL

end # module
