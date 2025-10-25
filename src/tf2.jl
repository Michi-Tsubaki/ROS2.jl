module TF2

using PyCall
using ..Core: ROSNode

const tf2_ros = PyNULL()
const tf2_geometry_msgs = PyNULL()
const geometry_msgs = PyNULL()

function __init_tf2__()
    copy!(tf2_ros, pyimport("tf2_ros"))
    copy!(tf2_geometry_msgs, pyimport("tf2_geometry_msgs"))
    copy!(geometry_msgs, pyimport("geometry_msgs.msg"))
end

mutable struct TransformListener
    _py_listener::PyObject
    _buffer::PyObject

    function TransformListener(node::ROSNode)
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, node.pynode)
        new(listener, buffer)
    end
end

mutable struct TransformBroadcaster
    _py_broadcaster::PyObject

    function TransformBroadcaster(node::ROSNode)
        broadcaster = tf2_ros.TransformBroadcaster(node.pynode)
        new(broadcaster)
    end
end

mutable struct StaticTransformBroadcaster
    _py_broadcaster::PyObject

    function StaticTransformBroadcaster(node::ROSNode)
        broadcaster = tf2_ros.StaticTransformBroadcaster(node.pynode)
        new(broadcaster)
    end
end

function lookup_transform(
    listener::TransformListener,
    target_frame::String,
    source_frame::String,
    time = nothing,
)
    if time === nothing
        return listener._buffer.lookup_transform(target_frame, source_frame, tf2_ros.Time())
    else
        return listener._buffer.lookup_transform(target_frame, source_frame, time)
    end
end

function can_transform(
    listener::TransformListener,
    target_frame::String,
    source_frame::String,
    time = nothing,
)
    if time === nothing
        return listener._buffer.can_transform(target_frame, source_frame, tf2_ros.Time())
    else
        return listener._buffer.can_transform(target_frame, source_frame, time)
    end
end

function send_transform(broadcaster::TransformBroadcaster, transform)
    broadcaster._py_broadcaster.sendTransform(transform)
end

function send_static_transform(broadcaster::StaticTransformBroadcaster, transform)
    broadcaster._py_broadcaster.sendTransform(transform)
end

function create_transform_stamped(
    parent_frame::String,
    child_frame::String,
    translation::Tuple{Float64,Float64,Float64},
    rotation::Tuple{Float64,Float64,Float64,Float64},
)
    transform = geometry_msgs.TransformStamped()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = translation[1]
    transform.transform.translation.y = translation[2]
    transform.transform.translation.z = translation[3]
    transform.transform.rotation.x = rotation[1]
    transform.transform.rotation.y = rotation[2]
    transform.transform.rotation.z = rotation[3]
    transform.transform.rotation.w = rotation[4]
    return transform
end

export TransformListener, TransformBroadcaster, StaticTransformBroadcaster
export lookup_transform, can_transform, send_transform, send_static_transform
export create_transform_stamped

end
