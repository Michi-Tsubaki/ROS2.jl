#!/usr/bin/env julia
using ROS2

init()
node = ROSNode("message_example")

# Get available message types
available_types = get_available_message_types()
println("Available message types: ", available_types[1:5])

println("\n--- Testing std_msgs ---")
string_msg = create_msg_dynamic("std_msgs.msg.String")
string_msg.data = "Hello ROS2 from Julia!"
string_fields = get_message_fields("std_msgs.msg.String")
println("String fields: ", string_fields)

int_msg = create_msg_dynamic("std_msgs.msg.Int32")
int_msg.data = 42
println("Int32 message created with value: ", int_msg.data)

println("\n--- Testing sensor_msgs ---")
laser_msg = create_msg_dynamic("sensor_msgs.msg.LaserScan")
laser_msg.header.frame_id = "laser"
laser_msg.angle_min = -1.57
laser_msg.angle_max = 1.57
laser_msg.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
laser_fields = get_message_fields("sensor_msgs.msg.LaserScan")
println("LaserScan fields: ", keys(laser_fields))

image_msg = create_msg_dynamic("sensor_msgs.msg.Image")
image_msg.header.frame_id = "camera"
image_msg.width = 640
image_msg.height = 480
image_msg.encoding = "rgb8"
println(
    "Image message: $(image_msg.width)x$(image_msg.height), encoding: $(image_msg.encoding)",
)

println("\n--- Testing geometry_msgs ---")
twist_msg = create_msg_dynamic("geometry_msgs.msg.Twist")
twist_msg.linear.x = 1.0
twist_msg.angular.z = 0.5

pose_msg = create_msg_dynamic("geometry_msgs.msg.PoseStamped")
pose_msg.header.frame_id = "map"
pose_msg.pose.position.x = 1.0
pose_msg.pose.position.y = 2.0
pose_msg.pose.orientation.w = 1.0

println("\n--- Testing nav_msgs ---")
if is_valid_message_type("nav_msgs.msg.Odometry")
    odom_msg = create_msg_dynamic("nav_msgs.msg.Odometry")
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"
    odom_msg.pose.pose.position.x = 5.0
    println("Odometry message created")
else
    println("nav_msgs.msg.Odometry not available")
end

string_pub = Publisher(node, "test_string", "std_msgs.msg.String")
laser_pub = Publisher(node, "test_laser", "sensor_msgs.msg.LaserScan")
twist_pub = Publisher(node, "test_twist", "geometry_msgs.msg.Twist")
pose_pub = Publisher(node, "test_pose", "geometry_msgs.msg.PoseStamped")

publish(string_pub, string_msg)
publish(laser_pub, laser_msg)
publish(twist_pub, twist_msg)
publish(pose_pub, pose_msg)

println("\n--- Published messages ---")
println("String: '$(string_msg.data)'")
println(
    "LaserScan: $(length(laser_msg.ranges)) ranges, frame: $(laser_msg.header.frame_id)",
)
println("Twist: linear.x=$(twist_msg.linear.x), angular.z=$(twist_msg.angular.z)")
println(
    "PoseStamped: x=$(pose_msg.pose.position.x), y=$(pose_msg.pose.position.y), frame: $(pose_msg.header.frame_id)",
)

# eg. for sensor data process
laser_data = create_msg_dynamic("sensor_msgs.msg.LaserScan")
laser_data.ranges = collect_lidar_data()

# eg. for robot control
cmd_vel = create_msg_dynamic("geometry_msgs.msg.Twist")
cmd_vel.linear.x = calculate_velocity()

# eg. for navigation
path = create_msg_dynamic("nav_msgs.msg.Path")
path.poses = generate_waypoints()

test_types = [
    "std_msgs.msg.Float64",
    "sensor_msgs.msg.PointCloud2",
    "geometry_msgs.msg.TransformStamped",
    "nav_msgs.msg.Path",
    "visualization_msgs.msg.Marker",
]

println("\n--- Message type validation ---")
for msg_type in test_types
    if is_valid_message_type(msg_type)
        println("✓ $(msg_type) is valid")
    else
        println("✗ $(msg_type) is not valid")
    end
end

shutdown()
