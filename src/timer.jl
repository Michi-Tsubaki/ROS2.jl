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
function timer_cancel(timer::ROSTimer)
    timer.pytimer.cancel()
end

function timer_reset(timer::ROSTimer)
    timer.pytimer.reset()
end

function timer_is_ready(timer::ROSTimer)
    return timer.pytimer.is_ready()
end

function timer_time_since_last_call(timer::ROSTimer)
    return timer.pytimer.time_since_last_call()
end

# Export functions
export ROSTimer, timer_cancel, timer_reset, timer_is_ready, timer_time_since_last_call

end # module
