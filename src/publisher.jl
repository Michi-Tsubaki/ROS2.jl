# API for publishing msg topics
export Publisher, publish

"""
    Publisher{T}(topic; kwargs...)
    Publisher(topic, T; kwargs...)

Create an object to publish messages of type `T` on a topic. Keyword arguments are directly passed
to rospy.
"""
struct Publisher{MsgType<:AbstractMsg}
    o::PyObject

    function Publisher{MT}(topic::AbstractString; kwargs...) where MT <: AbstractMsg
        @debug("Creating <$(string(MT))> publisher on topic: '$topic'")
        rospycls = _get_rospy_class(MT)
        return new{MT}(__rospy__.Publisher(ascii(topic), rospycls; kwargs...))
    end
end

Publisher(topic::AbstractString, ::Type{MT}; kwargs...) where {MT <: AbstractMsg} =
    Publisher{MT}(ascii(topic); kwargs...)

"""
    publish(p::Publisher{T}, msg::T)

Publish `msg` on `p`, a `Publisher` with matching message type.
"""
function publish(p::Publisher{MT}, msg::MT) where MT <: AbstractMsg
    pycall(p.o."publish", PyAny, convert(PyObject, msg))
end