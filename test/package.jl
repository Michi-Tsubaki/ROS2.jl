using Test
using RobotOperatingClient
using PyCall

# @testset "Package Tests" begin
#     @testset "Core Functionality" begin
#         @test isdefined(RobotOperatingClient, :ROSNode)
#         @test isdefined(RobotOperatingClient, :Publisher)
#         @test isdefined(RobotOperatingClient, :Subscriber)
#         @test isdefined(RobotOperatingClient, :ServiceServer)
#         @test isdefined(RobotOperatingClient, :ServiceClient)
#         @test isdefined(RobotOperatingClient, :ROSTimer)
#     end

#     @testset "Python Environment" begin
#         # ROSの環境変数のチェック
#         @test haskey(ENV, "AMENT_PREFIX_PATH") "ROS2 environment is not sourced"
        
#         # Pythonの基本的なインポートのチェック
#         ros_imports_ok = try
#             # rclpyのインポートチェック
#             rclpy = pyimport("rclpy")
            
#             # Node クラスのインポートチェック
#             node = pyimport("rclpy.node")
            
#             # Publisher クラスのインポートチェック
#             publisher = pyimport("rclpy.publisher")
            
#             # Subscription クラスのインポートチェック
#             subscription = pyimport("rclpy.subscription")
            
#             true
#         catch e
#             @error "Failed to import ROS2 Python modules" exception=e
#             false
#         end
        
#         @test ros_imports_ok "Failed to import required ROS2 Python modules"
#     end

#     @testset "Node Creation" begin
#         node = nothing
#         node_creation_ok = try
#             # ROSノードの作成
#             node = ROSNode("test_node")
            
#             # ノードがROSNode型であることを確認
#             node isa ROSNode
#         catch e
#             @error "Failed to create ROS node" exception=e
#             false
#         finally
#             # 作成したノードをシャットダウン
#             if node !== nothing
#                 shutdown()
#             end
#         end
        
#         @test node_creation_ok "Failed to create ROS node"
#     end

#     @testset "Lifecycle Management" begin
#         lifecycle_ok = try
#             # ROSノードのライフサイクル管理のテスト
#             init()
#             node = ROSNode("lifecycle_test_node")
#             true
#         catch e
#             @error "Failed lifecycle management test" exception=e
#             false
#         finally
#             # テストが終わったらシャットダウン
#             shutdown()
#         end
        
#         @test lifecycle_ok "Failed lifecycle management test"
#     end
# end
