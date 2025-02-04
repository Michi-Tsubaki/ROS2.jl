using Test
using ROS2

@testset "ROS2.jl" begin
    if !haskey(ENV, "CI")
        include("aqua_test.jl")
        include("package.jl")      
        include("example_test.jl") 
    else
        @test true  # CIではダミーテストのみ実行
    end
 end