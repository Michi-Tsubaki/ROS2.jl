module Parameter

using PyCall
using ..Core

# Parameter operations
function declare_parameter(node::ROSNode, name::String, value::Any)
    node.pynode.declare_parameter(name, value)
end

function declare_parameters(node::ROSNode, parameters::Dict{String,Any})
    for (name, value) in parameters
        declare_parameter(node, name, value)
    end
end

function get_parameter(node::ROSNode, name::String)
    return node.pynode.get_parameter(name).value
end

function set_parameter(node::ROSNode, name::String, value::Any)
    node.pynode.set_parameter(rclpy.parameter.Parameter(name, value=value))
end

function has_parameter(node::ROSNode, name::String)
    return node.pynode.has_parameter(name)
end

function get_parameter_types(node::ROSNode, names::Vector{String})
    return node.pynode.get_parameter_types(names)
end

# Parameter callback registration
function add_on_set_parameters_callback(node::ROSNode, callback::Function)
    py"""
    def add_callback(node, callback):
        from rcl_interfaces.msg import SetParametersResult
        def wrapper(parameters):
            results = callback(parameters)
            return SetParametersResult(successful=results)
        return node.add_on_set_parameters_callback(wrapper)
    """
    return py"add_callback"(node.pynode, callback)
end

function remove_on_set_parameters_callback(node::ROSNode, handle::PyObject)
    node.pynode.remove_on_set_parameters_callback(handle)
end

export declare_parameter, declare_parameters, get_parameter, set_parameter,
       has_parameter, get_parameter_types, add_on_set_parameters_callback,
       remove_on_set_parameters_callback

end # module