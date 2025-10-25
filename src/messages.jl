module Messages

using PyCall

function get_available_message_types()
    py"""
    import pkgutil
    import importlib

    def find_message_types():
        message_types = []
        for importer, modname, ispkg in pkgutil.iter_modules():
            if modname.endswith('_msgs'):
                try:
                    pkg = importlib.import_module(f"{modname}.msg")
                    for attr in dir(pkg):
                        if not attr.startswith('_'):
                            message_types.append(f"{modname}.msg.{attr}")
                except:
                    pass
        return message_types
    """
    return py"find_message_types"()
end

function create_msg_dynamic(msg_type::String)
    py"""
    def create_message(msg_type_str):
        parts = msg_type_str.split('.')
        pkg_name = parts[0]
        msg_name = parts[2]
        
        import importlib
        pkg = importlib.import_module(f"{pkg_name}.msg")
        msg_class = getattr(pkg, msg_name)
        return msg_class()
    """
    return py"create_message"(msg_type)
end

function get_message_fields(msg_type::String)
    py"""
    def get_fields(msg_type_str):
        parts = msg_type_str.split('.')
        pkg_name = parts[0]
        msg_name = parts[2]
        
        import importlib
        pkg = importlib.import_module(f"{pkg_name}.msg")
        msg_class = getattr(pkg, msg_name)
        msg_instance = msg_class()
        
        fields = {}
        for slot in msg_instance.__slots__:
            field_type = type(getattr(msg_instance, slot)).__name__
            fields[slot] = field_type
        return fields
    """
    return py"get_fields"(msg_type)
end

function is_valid_message_type(msg_type::String)
    return msg_type in get_available_message_types()
end

struct QoSProfile
    reliability::String  # "reliable" or "best_effort"
    durability::String   # "volatile" or "transient_local"
    history::String      # "keep_last" or "keep_all"
    depth::Int
end

const DEFAULT_QOS = QoSProfile("reliable", "volatile", "keep_last", 10)
const SENSOR_DATA_QOS = QoSProfile("best_effort", "volatile", "keep_last", 1)

function create_qos_profile(qos::QoSProfile)
    py"""
    from rclpy.qos import QoSProfile as PyQoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    def create_qos(reliability, durability, history, depth):
        qos = PyQoSProfile(depth=depth)
        
        if reliability == "reliable":
            qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            
        if durability == "transient_local":
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        else:
            qos.durability = DurabilityPolicy.VOLATILE
            
        if history == "keep_all":
            qos.history = HistoryPolicy.KEEP_ALL
        else:
            qos.history = HistoryPolicy.KEEP_LAST
            
        return qos
    """
    return py"create_qos"(qos.reliability, qos.durability, qos.history, qos.depth)
end

export get_available_message_types, create_msg_dynamic, get_message_fields
export is_valid_message_type, QoSProfile, create_qos_profile
export DEFAULT_QOS, SENSOR_DATA_QOS

end
