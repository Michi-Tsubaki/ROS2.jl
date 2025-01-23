#Wrappers for functions directly in the rospy namespace
export init, create_node, ok, spin,
       get_param, has_param, set_param, delete_param,
       logdebug, loginfo, logwarn, logerr, logfatal

"""
    init(_py_sys.argv)
    create_node(name::AbstractString; kwargs...)

Initialize this node, registering it with the ROS master. All arguments are passed on directly to
the rclpy init_node function.
"""
init(args=ARGS) == __rclpy__.init(args=args)
create_node(name::AbstractString; kwargs...) = __rclpy__.create_node(name; kwargs...)

"""
    ok()

Return the shutdown status of the node.
"""
ok() = __rclpy__.ok()

get_published_topics() = __rclpy__.get_published_topics()
get_ros_root()         = __rclpy__.get_ros_root()

"""
    spin()

Block execution and process callbacks/service calls until the node is shut down.
"""
function spin(node)
    while ok()
        rclpy.spin_once(node)
        sleep(0.001)
    end
end

#Parameter server API
"""
    get_param(param_name, default=nothing)

Request the value of a parameter from the parameter server, with optional default value. If no
default is given, throws a `KeyError` if the parameter cannot be found.
"""
function get_param(param_name::AbstractString, def=nothing)
    try
        if def == nothing
            __rclpy__.get_param(ascii(param_name))
        else
            __rclpy__.get_param(ascii(param_name), def)
        end
    catch ex
        throw(KeyError(pycall(pybuiltin("str"), PyAny, ex.val)[2:end-1]))
    end
end

"""
    set_param(param_name, val)

Set the value of a parameter on the parameter server.
"""
set_param(param_name::AbstractString, val) =
    __rclpy__.set_param(ascii(param_name), val)

"""
    has_param(param_name)

Return a boolean specifying if a parameter exists on the parameter server.
"""
has_param(param_name::AbstractString) =
    __rclpy__.has_param(ascii(param_name))

"""
    delete_param(param_name)

Delete a parameter from the parameter server. Throws a `KeyError` if no such parameter exists.
"""
function delete_param(param_name::AbstractString)
    try
        __rclpy__.delete_param(ascii(param_name))
    catch ex
        throw(KeyError(pycall(pybuiltin("str"), PyAny, ex.val)[2:end-1]))
    end
end

#Doesn't work for some reason
#rclpy_search_param(param_name::AbstractString) =
#    __rclpy__.clpy_search_param(ascii(param_name))
get_param_names() = __rclpy__.get_param_names()

#Logging API
logdebug(msg, args...) = __rclpy__.logdebug(msg, args...)
loginfo(msg, args...)  = __rclpy__.loginfo(msg, args...)
logwarn(msg, args...)  = __rclpy__.logwarn(msg, args...)
logerr(msg, args...)   = __rclpy__.logerr(msg, args...)
logfatal(msg, args...) = __rclpy__.logfatal(msg, args...)

"""
logdebug, loginfo, logwarn, logerr, logfatal

Call the rclpy logging system at the corresponding message level, passigeng a message and other
arguments directly.
"""
logdebug, loginfo, logwarn, logerr, logfatal

#Node information
get_name(node) = node.get_name()
get_namespace(node) = node.get_namespace()
get_fully_qualified_name(node) = node.get_fully_qualified_name()