module Service

using PyCall
using ..Core

# Service Server wrapper
mutable struct ServiceClient
    pyclient::PyObject
    srv_type::PyObject
    node::ROSNode  # ノードへの参照を保持
    
    function ServiceClient(node::ROSNode, service_name::String, srv_type::String)
        py"""
        def create_client(node, service_name, srv_type_str):
            import importlib
            
            parts = srv_type_str.split('.')
            module_path = '.'.join(parts[:-1])
            srv_class_name = parts[-1]
            module = importlib.import_module(module_path)
            srv_class = getattr(module, srv_class_name)
            
            client = node.create_client(srv_class, service_name)
            return client, srv_class
        """
        pyclient, srv_type_obj = py"create_client"(node.pynode, service_name, srv_type)
        return new(pyclient, srv_type_obj, node)  # ノードを保存
    end
end

# Service Server wrapper
mutable struct ServiceServer
    pyserver::PyObject
    srv_type::PyObject
    node::ROSNode
    
    function ServiceServer(node::ROSNode, service_name::String, srv_type::String, callback::Function)
        py"""
        def create_service(node, service_name, srv_type_str, callback):
            import importlib
            from functools import partial
            
            # Import service type
            parts = srv_type_str.split('.')
            module_path = '.'.join(parts[:-1])
            srv_class_name = parts[-1]
            module = importlib.import_module(module_path)
            srv_class = getattr(module, srv_class_name)
            
            def callback_wrapper(request, response):
                return callback(request, response)
            
            server = node.create_service(srv_class, service_name, callback_wrapper)
            return server, srv_class
        """
        pyserver, srv_type_obj = py"create_service"(node.pynode, service_name, srv_type, callback)
        return new(pyserver, srv_type_obj, node)
    end
end

# Service request/response creation helpers
function create_request(client::ServiceClient)
    return client.pyclient.srv_type.Request()
end

function create_response(request::PyObject)
    py"""
    def create_response(request):
        return request.__class__.Response()
    """
    return py"create_response"(request)
end

# Service client helpers
function call(client::ServiceClient, request::PyObject)
    if !client.pyclient.wait_for_service(timeout_sec=1.0)
        error("Service not available")
    end
    future = client.pyclient.call_async(request)
    while !future.done()
        spin_once(client.node)  # 保存したノードを使用
        sleep(0.1)
    end
    return future.result()
end

function call_async(client::ServiceClient, request::PyObject)
    return client.pyclient.call_async(request)
end

function wait_for_service(client::ServiceClient; timeout_sec::Union{Nothing,Float64}=nothing)
    return client.pyclient.wait_for_service(timeout_sec=timeout_sec)
end

function service_is_ready(client::ServiceClient)
    return client.pyclient.service_is_ready()
end

export ServiceServer, ServiceClient, create_request, create_response, call, call_async, wait_for_service, service_is_ready

end # module