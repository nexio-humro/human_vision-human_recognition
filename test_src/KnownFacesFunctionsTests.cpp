#include <catch.hpp>

#include <cmath>
#include <KnownFacesFunctions.hpp>
/*

TEST_CASE("findBestFaceFitFromFaces, dlib", "[KnownFacesFunctions]")
{
//	std::pair<double, int> findBestFaceFitFromFaces(dlib::matrix<float,0,1>& description, std::vector<FaceDescription>& knownFaces, double minTreshold)

	dlib::matrix<float,0,1> firstMatrix = {1.0, 2.0, 2.0};
	dlib::matrix<float,0,1> secondMatrix = {3.0, 2.0, 2.0};
	dlib::matrix<float,0,1> thirdMatrix = {1.5, 2.4, 5.0};
	
	std::vector<FaceDescription> known_faces;
	known_faces.push_back( FaceDescription(0, firstMatrix) );
	known_faces.push_back( FaceDescription(1, secondMatrix) );
	known_faces.push_back( FaceDescription(2, thirdMatrix) );
	
	SECTION("search for secondMatrix, index should be equal to 1, fit score should be equal to 0.0")
	{
		std::pair<double, int> result;
		result = KFF::findBestFaceFitFromFaces(secondMatrix, known_faces, 0.6);
		
		REQUIRE(result.first == 0.0);
		REQUIRE(result.second == 1);
	}
	
	SECTION("search for thirdMatrix, index should be equal to 2, fit score should be equal to 0.0")
	{
		std::pair<double, int> result;
		result = KFF::findBestFaceFitFromFaces(thirdMatrix, known_faces, 0.6);
		
		REQUIRE(result.first == 0.0);
		REQUIRE(result.second == 2);
	}
	
	SECTION("add fourthMatrix differs from firstMatrix")
	{
		dlib::matrix<float,0,1> fourthMatrix = {1.1, 2.0, 2.0};
		std::pair<double, int> result;
		
		SECTION("search for fourthdMatrix, index should be equal to 0, fit score should be equal to 0.1")
		{
			result = KFF::findBestFaceFitFromFaces(fourthMatrix, known_faces, 0.6);
			
			float difference = 0.0;
			for(size_t i = 0; i < fourthMatrix.size(); i++)
			{
				difference += std::sqrt(std::pow( (firstMatrix(i) - fourthMatrix(i)), 2));
			}	
			
			REQUIRE(result.first == difference);
			REQUIRE( (result.first - 0.1) < 0.0000001 );
			REQUIRE(result.second == 0);
		}
	
		SECTION("search for fourthdMatrix, set very restricted threshold, index should be equal to -1")
		{
			result = KFF::findBestFaceFitFromFaces(fourthMatrix, known_faces, 0.01);
		
			REQUIRE(result.second == -1);
		}	
	}
}

TEST_CASE("findBestFaceFitFromFaces, vector", "[KnownFacesFunctions]")
{
//	std::pair<double, int> findBestFaceFitFromFaces(dlib::matrix<float,0,1>& description, std::vector< dlib::matrix<float,0,1> >& descriptionsVector, double minTreshold)

	dlib::matrix<float,0,1> firstMatrix = {1.0, 2.0, 2.0};
	dlib::matrix<float,0,1> secondMatrix = {3.0, 2.0, 2.0};
	dlib::matrix<float,0,1> thirdMatrix = {1.5, 2.4, 5.0};
	
	std::vector<dlib::matrix<float,0,1>> known_faces;
	known_faces.push_back( firstMatrix );
	known_faces.push_back( secondMatrix );
	known_faces.push_back( thirdMatrix );
	
	SECTION("search for secondMatrix, index should be equal to 1, fit score should be equal to 0.0")
	{
		std::pair<double, int> result;
		result = KFF::findBestFaceFitFromFaces(secondMatrix, known_faces, 0.6);
		
		REQUIRE(result.first == 0.0);
		REQUIRE(result.second == 1);
	}
	
	SECTION("search for thirdMatrix, index should be equal to 2, fit score should be equal to 0.0")
	{
		std::pair<double, int> result;
		result = KFF::findBestFaceFitFromFaces(thirdMatrix, known_faces, 0.6);
		
		REQUIRE(result.first == 0.0);
		REQUIRE(result.second == 2);
	}
	
	SECTION("add fourthMatrix differs from firstMatrix")
	{
		dlib::matrix<float,0,1> fourthMatrix = {1.1, 2.0, 2.0};
		std::pair<double, int> result;
		
		SECTION("search for fourthdMatrix, index should be equal to 0, fit score should be equal to 0.1")
		{
			result = KFF::findBestFaceFitFromFaces(fourthMatrix, known_faces, 0.6);
			
			float difference = 0.0;
			for(size_t i = 0; i < fourthMatrix.size(); i++)
			{
				difference += std::sqrt(std::pow( (firstMatrix(i) - fourthMatrix(i)), 2));
			}	
			
			REQUIRE(result.first == difference);
			REQUIRE( (result.first - 0.1) < 0.0000001 );
			REQUIRE(result.second == 0);
		}
	
		SECTION("search for fourthdMatrix, set very restricted threshold, index should be equal to -1")
		{
			result = KFF::findBestFaceFitFromFaces(fourthMatrix, known_faces, 0.01);
		
			REQUIRE(result.second == -1);
		}	
	}
}

*/
