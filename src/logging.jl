module Logging

using PyCall
using ..Core

# Logging severity levels
const DEBUG = 10
const INFO = 20
const WARN = 30
const ERROR = 40
const FATAL = 50

# Logging functions
function get_logger(node::ROSNode)
    return node.pynode.get_logger()
end

function debug(logger::PyObject, msg::String)
    logger.debug(msg)
end

function info(logger::PyObject, msg::String)
    logger.info(msg)
end

function warn(logger::PyObject, msg::String)
    logger.warn(msg)
end

function error(logger::PyObject, msg::String)
    logger.error(msg)
end

function fatal(logger::PyObject, msg::String)
    logger.fatal(msg)
end

# Set logging level
function set_level(logger::PyObject, level::Integer)
    py"""
    def set_logger_level(logger, level):
        import rclpy.logging
        logger.set_level(level)
    """
    py"set_logger_level"(logger, level)
end

export get_logger, debug, info, warn, error, fatal, set_level,
       DEBUG, INFO, WARN, ERROR, FATAL

end # module