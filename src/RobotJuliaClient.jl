module RobotJuliaClient

using PyCall

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
