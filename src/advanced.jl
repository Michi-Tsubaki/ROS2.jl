module Advanced

using PyCall
using ..Core
using ..Service: ServiceClient, ServiceServer

const rclpy_lifecycle = PyNULL()
const rclpy_executors = PyNULL()

function __init_advanced__()
    try
        copy!(rclpy_lifecycle, pyimport("rclpy.lifecycle"))
        copy!(rclpy_executors, pyimport("rclpy.executors"))
    catch e
        @warn "Advanced features not available: $e"
    end
end

mutable struct LifecycleNode
    _py_node::PyObject

    function LifecycleNode(node_name::String)
        if rclpy_lifecycle.ptr == C_NULL
            error("Lifecycle functionality not available")
        end
        py_node = rclpy_lifecycle.LifecycleNode(node_name)
        new(py_node)
    end
end

mutable struct MultiThreadedExecutor
    _py_executor::PyObject

    function MultiThreadedExecutor(num_threads::Int = 4)
        py"""
        import rclpy.executors
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=$num_threads)
        """
        new(py"executor")
    end
end

function add_node(executor::MultiThreadedExecutor, node)
    if isa(node, ROSNode)
        executor._py_executor.add_node(node.pynode)
    elseif isa(node, LifecycleNode)
        executor._py_executor.add_node(node._py_node)
    end
end

function spin_executor(executor::MultiThreadedExecutor)
    executor._py_executor.spin()
end

function shutdown_executor(executor::MultiThreadedExecutor)
    executor._py_executor.shutdown()
end

mutable struct ComponentManager
    _node::ROSNode
    _load_client::ServiceClient
    _unload_client::ServiceClient
    _list_client::ServiceClient

    function ComponentManager(container_name::String = "ComponentManager")
        node = ROSNode(container_name)

        load_client = ServiceClient(
            node,
            "$(container_name)/_container/load_node",
            "composition_interfaces.srv.LoadNode",
        )
        unload_client = ServiceClient(
            node,
            "$(container_name)/_container/unload_node",
            "composition_interfaces.srv.UnloadNode",
        )
        list_client = ServiceClient(
            node,
            "$(container_name)/_container/list_nodes",
            "composition_interfaces.srv.ListNodes",
        )

        new(node, load_client, unload_client, list_client)
    end
end

function load_component(
    manager::ComponentManager,
    package_name::String,
    plugin_name::String,
    node_name::String = "",
)
    actual_node_name = node_name == "" ? plugin_name : node_name

    py"""
    import composition_interfaces.srv
    request = composition_interfaces.srv.LoadNode.Request()
    request.package_name = $package_name
    request.plugin_name = $plugin_name
    request.node_name = $actual_node_name
    request.node_namespace = ""
    """

    try
        client = manager._load_client.pyclient
        ready = client.wait_for_service(timeout_sec = 2.0)

        if ready
            future = client.call_async(py"request")

            py"""
            import rclpy
            node = $(manager._node.pynode)
            rclpy.spin_until_future_complete(node, $future, timeout_sec=5.0)
            """

            if future.done()
                response = future.result()
                return response
            else
                @warn "Service call timed out"
                return nothing
            end
        else
            @warn "Component manager service not available"
            return nothing
        end
    catch e
        @warn "Failed to load component: $e"
        return nothing
    end
end

function unload_component(manager::ComponentManager, unique_id::Int)
    py"""
    import composition_interfaces.srv
    request = composition_interfaces.srv.UnloadNode.Request()
    request.unique_id = $unique_id
    """

    try
        client = manager._unload_client.pyclient
        ready = client.wait_for_service(timeout_sec = 2.0)

        if ready
            future = client.call_async(py"request")

            py"""
            import rclpy
            node = $(manager._node.pynode)
            rclpy.spin_until_future_complete(node, $future, timeout_sec=5.0)
            """

            if future.done()
                response = future.result()
                return response
            else
                @warn "Service call timed out"
                return nothing
            end
        else
            @warn "Component manager service not available"
            return nothing
        end
    catch e
        @warn "Failed to unload component: $e"
        return nothing
    end
end

function list_components(manager::ComponentManager)
    py"""
    import composition_interfaces.srv
    request = composition_interfaces.srv.ListNodes.Request()
    """

    try
        client = manager._list_client.pyclient
        ready = client.wait_for_service(timeout_sec = 2.0)

        if ready
            future = client.call_async(py"request")

            py"""
            import rclpy
            node = $(manager._node.pynode)
            rclpy.spin_until_future_complete(node, $future, timeout_sec=5.0)
            """

            if future.done()
                response = future.result()
                return response
            else
                @warn "Service call timed out"
                return nothing
            end
        else
            @warn "Component manager service not available"
            return nothing
        end
    catch e
        @warn "Failed to list components: $e"
        return nothing
    end
end

function set_domain_id(domain_id::Int)
    ENV["ROS_DOMAIN_ID"] = string(domain_id)
end

function get_domain_id()
    return parse(Int, get(ENV, "ROS_DOMAIN_ID", "0"))
end

function configure(node::LifecycleNode)
    if rclpy_lifecycle.ptr != C_NULL
        return node._py_node.configure()
    else
        @warn "Lifecycle functionality not available"
        return nothing
    end
end

function activate(node::LifecycleNode)
    if rclpy_lifecycle.ptr != C_NULL
        return node._py_node.activate()
    else
        @warn "Lifecycle functionality not available"
        return nothing
    end
end

function deactivate(node::LifecycleNode)
    if rclpy_lifecycle.ptr != C_NULL
        return node._py_node.deactivate()
    else
        @warn "Lifecycle functionality not available"
        return nothing
    end
end

function cleanup(node::LifecycleNode)
    if rclpy_lifecycle.ptr != C_NULL
        return node._py_node.cleanup()
    else
        @warn "Lifecycle functionality not available"
        return nothing
    end
end

function shutdown_lifecycle(node::LifecycleNode)
    if rclpy_lifecycle.ptr != C_NULL
        return node._py_node.shutdown()
    else
        @warn "Lifecycle functionality not available"
        return nothing
    end
end

export LifecycleNode, configure, activate, deactivate, cleanup, shutdown_lifecycle
export MultiThreadedExecutor, add_node, spin_executor, shutdown_executor
export ComponentManager, load_component, unload_component, list_components
export set_domain_id, get_domain_id

end
