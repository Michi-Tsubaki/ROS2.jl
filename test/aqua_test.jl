@testitem "code quality" begin
	using Aqua
	using ROJ
	@testset "Aqua" begin
	  	Aqua.test_all(ROJ)
	end
end