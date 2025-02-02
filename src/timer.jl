module Timer

using PyCall
using ..Core

mutable struct ROSTimer
    pytimer::PyObject
    
    function ROSTimer(node::ROSNode, period::Float64, callback::Function; 
                     callback_group=nothing)
        py"""
        def create_timer(node, period, callback, callback_group=None):
            timer = node.create_timer(period, callback, callback_group=callback_group)
            return timer
        """
        pytimer = py"create_timer"(node.pynode, period, callback, callback_group)
        return new(pytimer)
    end
end

# Timer control functions
function cancel(timer::ROSTimer)
    timer.pytimer.cancel()
end

function reset(timer::ROSTimer)
    timer.pytimer.reset()
end

function is_ready(timer::ROSTimer)
    return timer.pytimer.is_ready()
end

function time_since_last_call(timer::ROSTimer)
    return timer.pytimer.time_since_last_call()
end

export ROSTimer, cancel, reset, is_ready, time_since_last_call

end # module