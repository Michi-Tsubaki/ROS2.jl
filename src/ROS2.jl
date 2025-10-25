module ROS2
using PyCall
const rclpy = PyNULL()
const rclpy_node = PyNULL()
const py_sys = PyNULL()

include("core.jl")
include("time.jl")
include("pubsub.jl")
include("timer.jl")
include("service.jl")
include("parameter.jl")
include("action.jl")
include("logging.jl")
include("tf2.jl")
include("messages.jl")

function __init__()
    if contains(lowercase(get(ENV, "GITHUB_WORKFLOW", "")), "automerge")
        return
    end

    try
        copy!(py_sys, pyimport("sys"))
        if length(ARGS) > 0
            py_sys.argv = ARGS
        end

        if !(dirname(@__FILE__) in py_sys."path")
            pushfirst!(py_sys."path", dirname(@__FILE__))
        end

        copy!(rclpy, pyimport_conda("rclpy", "rclpy", "conda-forge"))

        if !haskey(ENV, "AMENT_PREFIX_PATH")
            @warn "ROS2 environment not sourced"
            return
        end

        copy!(rclpy_node, pyimport("rclpy.node"))

        Time.__init_time__()
        TF2.__init_tf2__()

    catch e
        @warn "ROS2 initialization deferred: $e"
    end
end

using .Core
using .Time
using .PubSub
using .Timer
using .Service
using .Parameter
using .Action
using .Logging
using .TF2
using .Messages

export ROSNode,
    init,
    shutdown,
    spin,
    spin_once,
    is_ok,  # from Core
    ROSTime,
    ROSDuration,
    ROSClock,
    now,
    ros_time_now,
    to_sec,
    to_nsec,  # from Time
    to_msg_time,
    from_msg_time,
    to_msg_duration,
    from_msg_duration,  # from Time
    builtin_interfaces,  # from Time
    Publisher,
    Subscriber,
    create_msg,
    publish,  # from PubSub
    ROSTimer,
    timer_cancel,
    timer_reset,
    timer_is_ready,
    timer_time_since_last_call,  # from Timer
    ServiceServer,
    ServiceClient,
    create_request,
    call,
    call_async,  # from Service
    wait_for_service,
    service_is_ready,
    declare_parameter,
    declare_parameters,
    get_parameter,
    set_parameter,  # from Parameter
    has_parameter,
    get_parameter_types,
    add_on_set_parameters_callback,
    remove_on_set_parameters_callback,
    ActionServer,
    ActionClient,
    GoalHandle,
    send_goal,
    send_goal_sync,  # from Action
    cancel_goal,
    create_goal,
    accept_goal,
    reject_goal,
    publish_feedback,
    succeed,
    abort,
    get_logger,
    debug,
    info,
    warn,
    log_error,
    fatal,
    set_level,
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
    TransformListener,
    TransformBroadcaster,
    StaticTransformBroadcaster,
    lookup_transform,
    can_transform,
    send_transform,
    send_static_transform,
    create_transform_stamped, # from TF2
    get_available_message_types,
    create_msg_dynamic,
    get_message_fields,
    is_valid_message_type,
    QoSProfile,
    create_qos_profile,
    DEFAULT_QOS,
    SENSOR_DATA_QOS  # from Messages

end
