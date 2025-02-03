using Test
using RobotJuliaClient

@testset "RobotJuliaClient.jl" begin
    include("package.jl")      # パッケージの基本機能のテスト
    include("example_test.jl") # 各種通信機能のテスト
    include("aqua_test.jl")    # Aqua.jlによる品質テスト
end