@test !isempty(ROJ.package.command("list"))
@test !isempty(ROJ.package.getAll())
@test !isempty(ROJ.package.getPath("std_msgs"))

println("All $(basename(@__FILE__)) tests passed.")