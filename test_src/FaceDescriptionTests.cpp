#include <catch.hpp>

#include <FaceDescription.hpp>

TEST_CASE("constructors", "[FaceDescription]")
{
	
	SECTION("default constructor")
	{
//		FaceDescription()
		
		FaceDescription fd;
		
		REQUIRE(fd._id == -1);
		REQUIRE(fd._description.size() == 0);
	}
	
	SECTION("setting constructor")
	{
//		FaceDescription(int id, dlib::matrix<float,0,1> desc)
		
		dlib::matrix<float,0,1> matrixFD = {0.1, 0.2, 0.1, 0.4};
		int index = 10;
		FaceDescription fd(index, matrixFD);
		
		REQUIRE(fd._id == index);
		REQUIRE(fd._description.size() == matrixFD.size());
		
		for(size_t i = 0; i < fd._description.size(); i++)
		{
			REQUIRE(fd._description(i) == matrixFD(i));
		}
	}
}
