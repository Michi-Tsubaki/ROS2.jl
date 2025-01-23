@testitem "code quality" begin
	using Aqua
	using ROJ
	@testset "Aqua" begin
	  	Aqua.test__all(ROJ)
	end
end