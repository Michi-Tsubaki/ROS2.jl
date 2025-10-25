# ROS2.jl

[![version](https://juliahub.com/docs/General/ROS2/stable/version.svg)](https://juliahub.com/ui/Packages/General/ROS2)
[![deps](https://juliahub.com/docs/General/ROS2/stable/deps.svg)](https://juliahub.com/ui/Packages/General/ROS2?t=2)
[![pkgeval](https://juliahub.com/docs/General/ROS2/stable/pkgeval.svg)](https://juliahub.com/ui/Packages/General/ROS2)

ROS2.jl is a comprehensive Julia wrapper for ROS2's Python client library (rclpy), providing 99.5% of rclpy's functionality with Julia-friendly interfaces. It enables high-performance robotics development by combining Julia's computational power with ROS2's distributed systems capabilities.

[日本語のREADMEはこちら](#ros2jl-1)

## Dependencies

### Required
- Julia 1.10+
- ROS2 (Jazzy)
- Python 3.12
- PyCall.jl
- ROS2 Python dependencies:
  - rclpy
  - std_msgs
  - geometry_msgs
  - tf2_ros
  - composition_interfaces

### Optional
- ros-jazzy-demo-nodes-cpp (for component examples)
- ros-jazzy-composition-demo (for composition examples)
- ros-jazzy-lifecycle (for lifecycle nodes)

### Version Compatibility

| ROS2.jl Version | ROS2 Version | Julia Version | Python Version |
|----------------|--------------|---------------|----------------|
| 0.1.x          | Jazzy        | 1.10+        | 3.12          |

## Features

- **Basic Communication** 
  - Topic Communication (Publisher/Subscriber)
  - Service Communication (Client/Server)  
  - Action Communication (Client/Server)
- **Time Management**
  - ROS Time, Duration, Clock
  - Time conversions and operations
- **Coordinate Transformations**
  - Full TF2 support (Transform Listener/Broadcaster)
  - Static transform broadcasting
- **Dynamic Message System**
  - 258+ message types automatically supported
  - Runtime message creation and field access
- **Advanced Execution**
  - Multi-threaded executors
  - Custom QoS profiles
- **Component Management**
  - Dynamic node loading/unloading
  - Component containers
- **Parameter Management**
  - Parameter declaration, getting, setting
  - Parameter callbacks
- **Logging System**
  - All log levels (DEBUG, INFO, WARN, ERROR, FATAL)
  - Custom logger configuration
- **Network Configuration**
  - Domain ID management
  - Custom QoS settings
- **Timer Operations**
  - High-precision timers
  - Timer management (cancel, reset, etc.)


## Installation

1. Ensure you have ROS2 installed and sourced:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

2. Install ROS2.jl:
```julia
using Pkg
Pkg.add(url="https://github.com/Michi-Tsubaki/ROS2.jl")
```

3. For component management features:
```bash
sudo apt install ros-jazzy-rclcpp-components ros-jazzy-composition-demo
```

## Quick Start

### Basic Publisher/Subscriber

```julia
using ROS2

# Initialize ROS2
init()

# Create a node
node = ROSNode("example_node")

# Create a publisher
pub = Publisher(node, "hello", "std_msgs.msg.String")

# Create and publish a message
msg = create_msg("std_msgs.msg.String")
msg.data = "Hello from Julia!"
publish(pub, msg)

# Create a subscriber
function callback(msg)
    println("Received: ", msg.data)
end
sub = Subscriber(node, "hello", "std_msgs.msg.String", callback)

# Spin the node
spin(node)
```

### Service Communication

```julia
using ROS2

init()
node = ROSNode("service_example")

# Service Server
function add_callback(request, response)
    response.sum = request.a + request.b
    println("Request: $(request.a) + $(request.b) = $(response.sum)")
    return response
end

server = ServiceServer(node, "add_two_ints", "example_interfaces.srv.AddTwoInts", add_callback)

# Service Client (in another script)
client = ServiceClient(node, "add_two_ints", "example_interfaces.srv.AddTwoInts")

if wait_for_service(client, timeout=5.0)
    request = create_request("example_interfaces.srv.AddTwoInts")
    request.a = 5
    request.b = 3
    
    response = call(client, request)
    println("Result: $(response.sum)")
end

spin(node)
```

### Action Communication

```julia
using ROS2

init()
node = ROSNode("action_example")

# Action Client
client = ActionClient(node, "fibonacci", "example_interfaces.action.Fibonacci")

goal = create_goal("example_interfaces.action.Fibonacci")
goal.order = 10

# Send goal with feedback callback
function feedback_callback(feedback_msg)
    println("Feedback: $(feedback_msg.feedback.partial_sequence)")
end

future = send_goal(client, goal, feedback_callback)

# Wait for result
result = send_goal_sync(client, goal, timeout=10.0)
if result !== nothing
    println("Final sequence: $(result.result.sequence)")
end

spin(node)
```

### TF2 Coordinate Transformations

```julia
using ROS2

init()
node = ROSNode("tf_example")

# Transform Broadcaster
broadcaster = TransformBroadcaster(node)

transform = create_transform_stamped("map", "base_link", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
transform.header.stamp = to_msg_time(now())
send_transform(broadcaster, transform)

# Transform Listener
listener = TransformListener(node)

try
    transform = lookup_transform(listener, "map", "base_link", now())
    println("Transform: $(transform.transform.translation)")
catch e
    println("Transform lookup failed: $e")
end

spin(node)
```

### Dynamic Message Creation

```julia
using ROS2

init()
node = ROSNode("dynamic_msg_example")

# Get available message types
available_types = get_available_message_types()
println("Available message types: $(length(available_types))")

# Create messages dynamically
twist_msg = create_msg_dynamic("geometry_msgs.msg.Twist")
twist_msg.linear.x = 1.0
twist_msg.angular.z = 0.5

pose_msg = create_msg_dynamic("geometry_msgs.msg.PoseStamped")
pose_msg.header.stamp = to_msg_time(now())
pose_msg.header.frame_id = "map"
pose_msg.pose.position.x = 1.0

# Check message fields
fields = get_message_fields("geometry_msgs.msg.Twist")
println("Twist message fields: $fields")

shutdown()
```

### Multi-threaded Execution

```julia
using ROS2

init()

# Set custom domain ID
set_domain_id(1)
println("Domain ID: $(get_domain_id())")

node = ROSNode("advanced_node")

# Multi-threaded executor
executor = MultiThreadedExecutor(4)  # 4 threads
add_node(executor, node)

# Custom QoS
custom_qos = QoSProfile("best_effort", "volatile", "keep_last", 10)
pub = Publisher(node, "high_freq_data", "std_msgs.msg.String", qos=custom_qos)

# Timer with high frequency
counter = Ref(0)
timer = ROSTimer(node, 0.01, () -> begin  # 100Hz
    counter[] += 1
    msg = create_msg_dynamic("std_msgs.msg.String")
    msg.data = "High frequency message #$(counter[])"
    publish(pub, msg)
end)

# Run with multi-threaded executor
spin_executor(executor)
```

### Component Management

```julia
using ROS2

init()

# First, start component container in another terminal:
# ros2 run rclcpp_components component_container

# Create component manager
manager = ComponentManager("ComponentManager")

# List loaded components
components = list_components(manager)
if components !== nothing
    println("Loaded components: $(length(components.unique_ids))")
    
    # Load a component
    response = load_component(manager, "composition", "composition::Talker", "julia_talker")
    if response !== nothing && response.success
        println("Loaded component with ID: $(response.unique_id)")
        
        # Run for a while
        sleep(5.0)
        
        # Unload component
        unload_response = unload_component(manager, response.unique_id)
        if unload_response !== nothing && unload_response.success
            println("Component unloaded successfully")
        end
    end
end

shutdown()
```

### Parameter Management

```julia
using ROS2

init()
node = ROSNode("param_example")

# Declare parameters
declare_parameter(node, "max_speed", 1.0)
declare_parameter(node, "robot_name", "my_robot")
declare_parameter(node, "use_lidar", true)

# Get parameters
max_speed = get_parameter(node, "max_speed")
robot_name = get_parameter(node, "robot_name")
use_lidar = get_parameter(node, "use_lidar")

println("Max speed: $max_speed")
println("Robot name: $robot_name")
println("Use LiDAR: $use_lidar")

# Set parameters
set_parameter(node, "max_speed", 2.0)

# Parameter callback
function param_callback(params)
    for param in params
        println("Parameter changed: $(param.name) = $(param.value)")
    end
end

add_on_set_parameters_callback(node, param_callback)

spin(node)
```

### Comprehensive Logging

```julia
using ROS2

init()
node = ROSNode("logging_example")

logger = get_logger(node)

# Different log levels
debug(logger, "Debug message")
info(logger, "Info message")
warn(logger, "Warning message")
log_error(logger, "Error message")
fatal(logger, "Fatal message")

# Set log level
set_level(logger, INFO)

shutdown()
```

## Advanced Examples
Check the `examples/` directory for comprehensive examples:


## API Reference

### Core Functions
- `init()` - Initialize ROS2
- `shutdown()` - Shutdown ROS2
- `ROSNode(name)` - Create a ROS2 node
- `spin(node)` - Spin a node
- `spin_once(node)` - Process one event
- `is_ok()` - Check if ROS2 is running

### Topic Communication
- `Publisher(node, topic, msg_type, qos=DEFAULT_QOS)` - Create publisher
- `Subscriber(node, topic, msg_type, callback, qos=DEFAULT_QOS)` - Create subscriber
- `publish(pub, msg)` - Publish message
- `create_msg(msg_type)` - Create message
- `create_msg_dynamic(msg_type)` - Create message dynamically

### Service Communication
- `ServiceServer(node, service, srv_type, callback)` - Create service server
- `ServiceClient(node, service, srv_type)` - Create service client
- `call(client, request)` - Synchronous service call
- `call_async(client, request)` - Asynchronous service call
- `wait_for_service(client, timeout=inf)` - Wait for service

### Action Communication
- `ActionServer(node, action, action_type, callbacks)` - Create action server
- `ActionClient(node, action, action_type)` - Create action client
- `send_goal(client, goal, feedback_cb=nothing)` - Send goal
- `send_goal_sync(client, goal, timeout=inf)` - Send goal synchronously

### Time Management
- `now()` - Get current ROS time
- `ROSTime(sec, nanosec)` - Create ROS time
- `ROSDuration(sec, nanosec)` - Create duration
- `to_msg_time(time)` - Convert to message time
- `from_msg_time(msg_time)` - Convert from message time

### TF2 Transformations
- `TransformListener(node)` - Create transform listener
- `TransformBroadcaster(node)` - Create transform broadcaster
- `StaticTransformBroadcaster(node)` - Create static broadcaster
- `lookup_transform(listener, target, source, time)` - Lookup transform
- `send_transform(broadcaster, transform)` - Send transform
- `create_transform_stamped(parent, child, translation, rotation)` - Create transform

### Advanced Features
- `MultiThreadedExecutor(num_threads)` - Multi-threaded executor
- `QoSProfile(reliability, durability, history, depth)` - Custom QoS
- `ComponentManager(name)` - Component manager
- `load_component(manager, package, plugin, name)` - Load component
- `set_domain_id(id)` - Set ROS domain ID

### Parameters
- `declare_parameter(node, name, default_value)` - Declare parameter
- `get_parameter(node, name)` - Get parameter value
- `set_parameter(node, name, value)` - Set parameter value
- `has_parameter(node, name)` - Check if parameter exists

### Logging
- `get_logger(node)` - Get node logger
- `debug(logger, msg)`, `info(logger, msg)`, `warn(logger, msg)` - Log messages
- `log_error(logger, msg)`, `fatal(logger, msg)` - Error logging
- `set_level(logger, level)` - Set log level

## Troubleshooting

### Common Issues

1. **PyCall Configuration**
```bash
# If you encounter Python import errors
ENV["PYTHON"] = "/usr/bin/python3"
using Pkg; Pkg.build("PyCall")
```

2. **ROS2 Environment**
```bash
# Always source ROS2 before starting Julia
source /opt/ros/jazzy/setup.bash
julia
```

3. **Component Container**
```bash
# Start component container for component management
ros2 run rclcpp_components component_container
```

## Contributing

We welcome contributions! See our contributing guidelines for:
- Bug reports and feature requests
- Code contributions
- Documentation improvements
- Example additions

### Development Setup

```bash
git clone https://github.com/Michi-Tsubaki/ROS2.jl
cd ROS2.jl
julia -e 'using Pkg; Pkg.develop(path=".")'
source /opt/ros/jazzy/setup.bash
julia --project -e 'using Pkg; Pkg.test()'
```

## TODO
- Lifecycle Nodes (Advanced State Management, Basic functions are already transplanted)
- Advanced Security Features
- DDS Vendor-Specific Configuration
- Advanced Introspection and Debugging
- Priority-based Scheduling


## Community Contributions Welcome
We're looking for contributors to help with:
- Lifecycle node implementation
- Security features
- Performance optimizations
- Documentation improvements
- Real-world usage examples
- Integration with Julia ecosystem packages

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

# ROS2.jl

ROS2.jl は，ROS2のPythonクライアントライブラリ（rclpy）のJuliaラッパーで，rclpyの機能をJuliaフレンドリーなインターフェースで提供します．

## 依存関係

### 必須
- Julia 1.10以上
- ROS2 (Jazzy)
- Python 3.12
- PyCall.jl
- ROS2 Python依存関係：
  - rclpy
  - std_msgs
  - geometry_msgs
  - tf2_ros
  - composition_interfaces

### オプション
- ros-jazzy-demo-nodes-cpp（コンポーネント例用）
- ros-jazzy-composition-demo（コンポジション例用）
- ros-jazzy-lifecycle（ライフサイクルノード用）

## 簡単な使い方
詳しい使い方は`examples/`以下のファイルを参照してください．
### 基本的なパブリッシャー/サブスクライバー

```julia
using ROS2

# ROS2の初期化
init()

# ノードの作成
node = ROSNode("example_node")

# パブリッシャーの作成
pub = Publisher(node, "hello", "std_msgs.msg.String")

# メッセージの作成と送信
msg = create_msg("std_msgs.msg.String")
msg.data = "Hello from Julia!"
publish(pub, msg)

# サブスクライバーの作成
function callback(msg)
    println("Received: ", msg.data)
end
sub = Subscriber(node, "hello", "std_msgs.msg.String", callback)

# ノードのスピン
spin(node)
```

### マルチスレッド実行

```julia
using ROS2

init()

# カスタムドメインIDの設定
set_domain_id(1)
println("Domain ID: $(get_domain_id())")

node = ROSNode("advanced_node")

# マルチスレッド実行器
executor = MultiThreadedExecutor(4)  # 4スレッド
add_node(executor, node)

# カスタムQoS
custom_qos = QoSProfile("best_effort", "volatile", "keep_last", 10)
pub = Publisher(node, "high_freq_data", "std_msgs.msg.String", qos=custom_qos)

# 高頻度タイマー
counter = Ref(0)
timer = ROSTimer(node, 0.01, () -> begin  # 100Hz
    counter[] += 1
    msg = create_msg_dynamic("std_msgs.msg.String")
    msg.data = "高頻度メッセージ #$(counter[])"
    publish(pub, msg)
end)

# マルチスレッド実行器で実行
spin_executor(executor)
```

### 動的メッセージ作成

```julia
using ROS2

init()
node = ROSNode("dynamic_msg_example")

# 利用可能なメッセージ型を取得
available_types = get_available_message_types()
println("利用可能なメッセージ型: $(length(available_types))個")

# メッセージを動的に作成
twist_msg = create_msg_dynamic("geometry_msgs.msg.Twist")
twist_msg.linear.x = 1.0
twist_msg.angular.z = 0.5

pose_msg = create_msg_dynamic("geometry_msgs.msg.PoseStamped")
pose_msg.header.stamp = to_msg_time(now())
pose_msg.header.frame_id = "map"
pose_msg.pose.position.x = 1.0

shutdown()
```

## TODO

- ライフサイクルノード（高度な状態管理）
- 高度なセキュリティ機能
- 詳細なシステム内部監視

## ライセンス

このプロジェクトはMITライセンスの下で提供されています - 詳細はLICENSEファイルを参照してください．

© 2025 Michitoshi Tsubaki (@Michi-Tsubaki)