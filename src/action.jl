module Action

using PyCall
using ..Core

# Action Server wrapper
mutable struct ActionServer
    pyserver::PyObject
    action_type::PyObject
    
    function ActionServer(node::ROSNode, action_name::String, action_type::String,
                         execute_callback::Function,
                         goal_callback::Union{Function,Nothing}=nothing,
                         cancel_callback::Union{Function,Nothing}=nothing)
        py"""
        def create_action_server(node, action_name, action_type_str, execute_cb, goal_cb=None, cancel_cb=None):
            import importlib
            import rclpy.action
            
            parts = action_type_str.split('.')
            module_path = '.'.join(parts[:-1])
            action_class_name = parts[-1]
            module = importlib.import_module(module_path)
            action_class = getattr(module, action_class_name)
            
            server = rclpy.action.ActionServer(
                node=node,
                action_type=action_class,
                action_name=action_name,
                execute_callback=execute_cb,
                goal_callback=goal_cb,
                cancel_callback=cancel_cb
            )
            return server, action_class
        """
        pyserver, action_type_obj = py"create_action_server"(
            node.pynode, action_name, action_type,
            execute_callback, goal_callback, cancel_callback
        )
        return new(pyserver, action_type_obj)
    end
end

# Action Client wrapper
mutable struct ActionClient
    pyclient::PyObject
    action_type::PyObject
    
    function ActionClient(node::ROSNode, action_name::String, action_type::String)
        py"""
        def create_action_client(node, action_name, action_type_str):
            import importlib
            import rclpy.action
            
            parts = action_type_str.split('.')
            module_path = '.'.join(parts[:-1])
            action_class_name = parts[-1]
            module = importlib.import_module(module_path)
            action_class = getattr(module, action_class_name)
            
            client = rclpy.action.ActionClient(
                node=node,
                action_type=action_class,
                action_name=action_name
            )
            return client, action_class
        """
        pyclient, action_type_obj = py"create_action_client"(node.pynode, action_name, action_type)
        return new(pyclient, action_type_obj)
    end
end

# Goal handle wrapper
mutable struct GoalHandle
    pygoal_handle::PyObject
end

# Action client helpers
function send_goal(client::ActionClient, goal::PyObject;
                  feedback_callback::Union{Function,Nothing}=nothing)
    py"""
    def send_goal_async(client, goal, feedback_callback=None):
        return client.send_goal_async(goal, feedback_callback=feedback_callback)
    """
    future = py"send_goal_async"(client.pyclient, goal, feedback_callback)
    return future
end

function send_goal_sync(client::ActionClient, goal::PyObject;
                       feedback_callback::Union{Function,Nothing}=nothing)
    future = send_goal(client, goal, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(client.pyclient.node, future)
    goal_handle = future.result()
    if goal_handle === nothing || !goal_handle.accepted
        return nothing
    end
    
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client.pyclient.node, result_future)
    return result_future.result()
end

function cancel_goal(goal_handle::GoalHandle)
    py"""
    def cancel_goal_async(goal_handle):
        return goal_handle.cancel_goal_async()
    """
    future = py"cancel_goal_async"(goal_handle.pygoal_handle)
    return future
end

# Action goal creation helper
function create_goal(client::ActionClient)
    return client.action_type.Goal()
end

# Action server helpers
function accept_goal(goal_handle::GoalHandle)
    return goal_handle.pygoal_handle.accept()
end

function reject_goal(goal_handle::GoalHandle)
    return goal_handle.pygoal_handle.reject()
end

function publish_feedback(goal_handle::GoalHandle, feedback::PyObject)
    goal_handle.pygoal_handle.publish_feedback(feedback)
end

function succeed(goal_handle::GoalHandle, result::PyObject)
    goal_handle.pygoal_handle.succeed(result)
end

function abort(goal_handle::GoalHandle, result::PyObject)
    goal_handle.pygoal_handle.abort(result)
end

function cancel(goal_handle::GoalHandle)
    goal_handle.pygoal_handle.cancel()
end

export ActionServer, ActionClient, GoalHandle,
       send_goal, send_goal_sync, cancel_goal, create_goal,
       accept_goal, reject_goal, publish_feedback,
       succeed, abort, cancel

end # module