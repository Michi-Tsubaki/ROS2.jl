module PubSub

using PyCall
using ..Core

# Publisher wrapper
mutable struct Publisher
    pypub::PyObject
    msg_type::PyObject
    
    function Publisher(node::ROSNode, topic::String, msg_type::String; qos_profile=10)
        py"""
        def create_publisher(node, topic, msg_type_str, qos_profile):
            import importlib
            from rclpy.qos import QoSProfile
            
            parts = msg_type_str.split('.')
            module_path = '.'.join(parts[:-1])
            msg_class_name = parts[-1]
            module = importlib.import_module(module_path)
            msg_class = getattr(module, msg_class_name)
            
            if isinstance(qos_profile, int):
                qos = QoSProfile(depth=qos_profile)
            else:
                qos = qos_profile
                
            pub = node.create_publisher(msg_class, topic, qos)
            return pub, msg_class
        """
        pypub, msg_type_obj = py"create_publisher"(node.pynode, topic, msg_type, qos_profile)
        return new(pypub, msg_type_obj)
    end
end

# Subscriber wrapper
mutable struct Subscriber
    pysub::PyObject
    
    function Subscriber(node::ROSNode, topic::String, msg_type::String, callback::Function; qos_profile=10)
        py"""
        def create_subscriber(node, topic, msg_type_str, callback, qos_profile):
            import importlib
            from rclpy.qos import QoSProfile
            
            parts = msg_type_str.split('.')
            module_path = '.'.join(parts[:-1])
            msg_class_name = parts[-1]
            module = importlib.import_module(module_path)
            msg_class = getattr(module, msg_class_name)
            
            if isinstance(qos_profile, int):
                qos = QoSProfile(depth=qos_profile)
            else:
                qos = qos_profile
                
            return node.create_subscription(msg_class, topic, callback, qos)
        """
        pysub = py"create_subscriber"(node.pynode, topic, msg_type, callback, qos_profile)
        return new(pysub)
    end
end

# Message creation helper
function create_msg(msg_type::String)
    py"""
    def create_message(msg_type_str):
        import importlib
        
        parts = msg_type_str.split('.')
        module_path = '.'.join(parts[:-1])
        msg_class_name = parts[-1]
        module = importlib.import_module(module_path)
        msg_class = getattr(module, msg_class_name)
        
        msg = msg_class()
        return msg
    """
    return py"create_message"(msg_type)
end

# Publish helper
function publish(pub::Publisher, msg::PyObject)
    pub.pypub.publish(msg)
end

export Publisher, Subscriber, create_msg, publish

end # module