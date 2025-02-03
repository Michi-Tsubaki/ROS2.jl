using Test
using Aqua
using ROS2

@testset "Aqua.jl Quality Tests" begin
    Aqua.test_all(ROS2;
        ambiguities=false,  # 多重ディスパッチの曖昧さをチェック
        unbound_args=true,  # 未束縛の型パラメータをチェック
        undefined_exports=true,  # 未定義のエクスポートをチェック
        project_extras=true,  # Project.tomlの追加チェック
        deps_compat=true,  # 依存関係の互換性チェック
        stale_deps=true,  # 使用されていない依存関係のチェック
        piracies=true)  # メソッドの衝突チェック
end