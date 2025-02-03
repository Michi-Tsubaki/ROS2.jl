using Test
using ROS2

@testset "Communication Tests" begin
    @testset "Publisher/Subscriber" begin
        node = ROSNode("pubsub_test_node")
        
        # メッセージ受信用の変数
        received_msg = nothing
        
        # サブスクライバーのコールバック
        function callback(msg)
            received_msg = msg.data
        end
        
        pub = Publisher(node, "test_topic", "std_msgs.msg.String")
        sub = Subscriber(node, "test_topic", "std_msgs.msg.String", callback)
        
        msg = create_msg("std_msgs.msg.String")
        msg.data = "test_message"
        
        # メッセージの送信
        publish(pub, msg)
        
        # メッセージが受信されるまでスピン
        for i in 1:20  # ループ回数を増やして、十分にスピンさせる
            spin_once(node)
            sleep(0.5)  # sleep時間を少し長めにしてみる
        end
        
        # メッセージの受信を確認
        @test received_msg == "test_message"
        
        shutdown()
    end

    # @testset "Service Communication" begin
    #     server_node = ROSNode("service_test_server")
    #     client_node = ROSNode("service_test_client")
        
    #     # サービスのコールバック
    #     function service_callback(request, response)
    #         response.sum = request.a + request.b
    #         return response
    #     end
        
    #     server = ServiceServer(server_node, "test_add", "example_interfaces.srv.AddTwoInts", service_callback)
    #     client = ServiceClient(client_node, "test_add", "example_interfaces.srv.AddTwoInts")
        
    #     # サービスが利用可能になるまで待機
    #     sleep(1.0)
    #     @test wait_for_service(client, timeout_sec=1.0)
        
    #     # リクエストの送信
    #     request = create_request(client)
    #     request.a = 2
    #     request.b = 3
    #     response = call(client, request)
        
    #     @test response.sum == 5
        
    #     shutdown()
    # end

    # @testset "Timer" begin
    #     node = ROSNode("timer_test_node")
    #     counter = 0
        
    #     timer = ROSTimer(node, 0.1) do
    #         counter += 1
    #     end
        
    #     # タイマーのテスト
    #     for i in 1:5
    #         spin_once(node)
    #         sleep(0.2)
    #     end
        
    #     @test counter > 0
        
    #     shutdown()
    # end
end
