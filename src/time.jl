module Time

using PyCall

const rclpy_time = PyNULL()
const rclpy_clock = PyNULL()
const rclpy_duration = PyNULL()

function __init_time__()
    copy!(rclpy_time, pyimport("rclpy.time"))
    copy!(rclpy_clock, pyimport("rclpy.clock"))
    copy!(rclpy_duration, pyimport("rclpy.duration"))
end

mutable struct ROSTime
    _py_time::PyObject

    function ROSTime(seconds::Real = 0.0, nanoseconds::Integer = 0)
        py_time = rclpy_time.Time(seconds = seconds, nanoseconds = nanoseconds)
        new(py_time)
    end

    function ROSTime(py_time::PyObject)
        new(py_time)
    end
end

mutable struct ROSDuration
    _py_duration::PyObject

    function ROSDuration(seconds::Real = 0.0, nanoseconds::Integer = 0)
        py_duration = rclpy_duration.Duration(seconds = seconds, nanoseconds = nanoseconds)
        new(py_duration)
    end

    function ROSDuration(py_duration::PyObject)
        new(py_duration)
    end
end

mutable struct ROSClock
    _py_clock::PyObject

    function ROSClock(clock_type = nothing)
        if clock_type === nothing
            py_clock = rclpy_clock.Clock()
        else
            py_clock = rclpy_clock.Clock(clock_type = clock_type)
        end
        new(py_clock)
    end

    function ROSClock(py_clock::PyObject)
        new(py_clock)
    end
end

Base.:(+)(t::ROSTime, d::ROSDuration) = ROSTime(t._py_time + d._py_duration)
Base.:(-)(t::ROSTime, d::ROSDuration) = ROSTime(t._py_time - d._py_duration)
Base.:(-)(t1::ROSTime, t2::ROSTime) = ROSDuration(t1._py_time - t2._py_time)

Base.:(+)(d1::ROSDuration, d2::ROSDuration) = ROSDuration(d1._py_duration + d2._py_duration)
Base.:(-)(d1::ROSDuration, d2::ROSDuration) = ROSDuration(d1._py_duration - d2._py_duration)
Base.:(*)(d::ROSDuration, scalar::Real) = ROSDuration(d._py_duration * scalar)
Base.:(*)(scalar::Real, d::ROSDuration) = ROSDuration(scalar * d._py_duration)

Base.:(==)(t1::ROSTime, t2::ROSTime) = t1._py_time == t2._py_time
Base.:(<)(t1::ROSTime, t2::ROSTime) = t1._py_time < t2._py_time
Base.:(<=)(t1::ROSTime, t2::ROSTime) = t1._py_time <= t2._py_time

Base.:(==)(d1::ROSDuration, d2::ROSDuration) = d1._py_duration == d2._py_duration
Base.:(<)(d1::ROSDuration, d2::ROSDuration) = d1._py_duration < d2._py_duration
Base.:(<=)(d1::ROSDuration, d2::ROSDuration) = d1._py_duration <= d2._py_duration

function now(clock::ROSClock = ROSClock())
    return ROSTime(clock._py_clock.now())
end

function ros_time_now()
    return now()
end

function to_sec(t::ROSTime)
    return t._py_time.nanoseconds / 1e9
end

function to_nsec(t::ROSTime)
    return t._py_time.nanoseconds
end

function to_sec(d::ROSDuration)
    return d._py_duration.nanoseconds / 1e9
end

function to_nsec(d::ROSDuration)
    return d._py_duration.nanoseconds
end

module builtin_interfaces
struct Time
    sec::Int32
    nanosec::UInt32
end

struct Duration
    sec::Int32
    nanosec::UInt32
end
end

function to_msg_time(t::ROSTime)
    py"""
    def create_time_msg(sec, nanosec):
        from builtin_interfaces.msg import Time
        msg = Time()
        msg.sec = int(sec)
        msg.nanosec = int(nanosec)
        return msg
    """
    ns = t._py_time.nanoseconds
    sec = ns ÷ 1_000_000_000
    nanosec = ns % 1_000_000_000
    return py"create_time_msg"(sec, nanosec)
end

function from_msg_time(msg_time)
    sec = msg_time.sec
    nanosec = msg_time.nanosec
    total_ns = sec * 1_000_000_000 + nanosec
    return ROSTime(0.0, total_ns)
end

function to_msg_duration(d::ROSDuration)
    py"""
    def create_duration_msg(sec, nanosec):
        from builtin_interfaces.msg import Duration
        msg = Duration()
        msg.sec = int(sec)
        msg.nanosec = int(nanosec)
        return msg
    """
    ns = d._py_duration.nanoseconds
    sec = ns ÷ 1_000_000_000
    nanosec = ns % 1_000_000_000
    return py"create_duration_msg"(sec, nanosec)
end

function from_msg_duration(msg_duration)
    sec = msg_duration.sec
    nanosec = msg_duration.nanosec
    total_ns = sec * 1_000_000_000 + nanosec
    return ROSDuration(0.0, total_ns)
end

export ROSTime, ROSDuration, ROSClock
export now, ros_time_now, to_sec, to_nsec
export to_msg_time, from_msg_time, to_msg_duration, from_msg_duration
export builtin_interfaces

end
