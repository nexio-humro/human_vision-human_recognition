#include <catch.hpp>

#include <cmath>
#include <limits>
#include <KnownFaces.hpp>

TEST_CASE("instance()", "[KnownFaces]")
{	
//	KnownFaces::instance()
		
	REQUIRE(KnownFaces::instance()->getFacesAmount() == 0);
	REQUIRE(KnownFaces::instance()->getFace(0)._description.size() == 0);
	REQUIRE(KnownFaces::instance()->getFace(0)._id == -1);
}

TEST_CASE("addFaces(), getFace(), getFaceAmounts()", "[KnownFaces]")
{
	dlib::matrix<float,0,1> description1 = {0.0, 1.0, 2.0, 3.0};
	dlib::matrix<float,0,1> description2 = {6.0, 1.6, 2.4, 3.7};
	dlib::matrix<float,0,1> description3 = {-2.0, -1.0, 1.3, 0.8};
	dlib::matrix<float,0,1> description4 = {0.4, 1.8, -2.6, 1.5};
	
	KnownFaces::instance()->addFace(description1, 0);
	REQUIRE(KnownFaces::instance()->getFacesAmount() == 1);
	FaceDescription face1 = KnownFaces::instance()->getFace(0);
	REQUIRE(face1._description == description1);
	REQUIRE(face1._id == 0);
	
	KnownFaces::instance()->addFace(description2, 1);
	REQUIRE(KnownFaces::instance()->getFacesAmount() == 2);
	FaceDescription face2 = KnownFaces::instance()->getFace(1);
	REQUIRE(face2._description == description2);
	REQUIRE(face2._id == 1);
	
	KnownFaces::instance()->addFace(description3, 3);
	REQUIRE(KnownFaces::instance()->getFacesAmount() == 3);
	FaceDescription face3 = KnownFaces::instance()->getFace(2);
	REQUIRE(face3._description == description3);
	REQUIRE(face3._id == 3);
	
	KnownFaces::instance()->addFace(description4, 7);
	REQUIRE(KnownFaces::instance()->getFacesAmount() == 4);
	FaceDescription face4 = KnownFaces::instance()->getFace(3);
	REQUIRE(face4._description == description4);
	REQUIRE(face4._id == 7);
	
	SECTION("wrong indexes")
	{
		REQUIRE(KnownFaces::instance()->getFace(9)._description.size() == 0);
		REQUIRE(KnownFaces::instance()->getFace(9)._id == -1);
		REQUIRE(KnownFaces::instance()->getFace(-9)._description.size() == 0);
		REQUIRE(KnownFaces::instance()->getFace(-9)._id == -1);
	}
}

TEST_CASE("KnownFace::findBestFaceFitFromFaces, sending only description", "[KnownFaces]")
{
//	std::pair<double, int> findBestFaceFitFromFaces(dlib::matrix<float,0,1>& description, double minTreshold = 0.6); 

	REQUIRE(KnownFaces::instance()->getFacesAmount() == 4); // in previous TEST_CASE 4 faces were added
	
	SECTION("Send similar face description to third face description")
	{
		dlib::matrix<float,0,1> description = {-2.1, -0.9, 1.3, 0.9};
		std::pair<double, int> similarFace = KnownFaces::instance()->findBestFaceFitFromFaces(description);
	
		REQUIRE(similarFace.second == 2);
		REQUIRE( (similarFace.first - std::sqrt(0.03)) < 0.0000001 );
	}
	
	SECTION("send empty description")
	{
		dlib::matrix<float,0,1> empty_description;
		std::pair<double, int> nullComparison = KnownFaces::instance()->findBestFaceFitFromFaces(empty_description);
	
		REQUIRE(nullComparison.second == KFF::wrongIndexResult);
		REQUIRE(nullComparison.first == KFF::wrongScoreResult);
	}
} 

TEST_CASE("KnownFace::findBestFaceFitFromFaces, sending description's vactor and face index", "[KnownFaces]")
{
//	std::pair<double, int> KnownFaces::findBestFaceFitFromFaces(std::vector<dlib::matrix<float,0,1>>& descriptionVector, int faceIndex, double minTreshold) 

	dlib::matrix<float,0,1> description1 = {0.0, 1.0, 2.0, 3.0};
	dlib::matrix<float,0,1> description2 = {-2.1, -0.9, 1.3, 0.9};
	dlib::matrix<float,0,1> description3 = {6.0, 1.6, 2.4, 3.7};
	dlib::matrix<float,0,1> description4 = {0.4, 1.8, -2.6, 1.5};
	
	std::vector< dlib::matrix<float,0,1> > descriptions;
	descriptions.push_back(description1);
	descriptions.push_back(description2);
	descriptions.push_back(description3);
	descriptions.push_back(description4);

	REQUIRE(KnownFaces::instance()->getFacesAmount() == 4); // in previous TEST_CASE 4 faces were added
	
	SECTION("find similar face description to third known face description from vector of descriptions")
	{
		std::pair<double, int> similarFace = KnownFaces::instance()->findBestFaceFitFromFaces(descriptions, 2);
	
		REQUIRE(similarFace.second == 1);
		REQUIRE( (similarFace.first - std::sqrt(0.03)) < 0.0000001 );
	}
	
	SECTION("find similar face description to third known face description from vector of descriptions, send wrong index")
	{
		std::pair<double, int> tooBigIndex = KnownFaces::instance()->findBestFaceFitFromFaces(descriptions, 88);
	
		REQUIRE(tooBigIndex.second == KFF::wrongIndexResult);
		REQUIRE(tooBigIndex.first == KFF::wrongScoreResult);
		
		std::pair<double, int> negativeIndex = KnownFaces::instance()->findBestFaceFitFromFaces(descriptions, -2);
	
		REQUIRE(negativeIndex.second == KFF::wrongIndexResult);
		REQUIRE(negativeIndex.first == KFF::wrongScoreResult);
		
		std::pair<double, int> bigNegativeIndex = KnownFaces::instance()->findBestFaceFitFromFaces(descriptions, -97);
	
		REQUIRE(bigNegativeIndex.second == KFF::wrongIndexResult);
		REQUIRE(bigNegativeIndex.first == KFF::wrongScoreResult);
		
		std::pair<double, int> theBiggestNegativeIndex = KnownFaces::instance()->findBestFaceFitFromFaces(descriptions, -(std::numeric_limits<int>::max()-5) );
	
		REQUIRE(theBiggestNegativeIndex.second == KFF::wrongIndexResult);
		REQUIRE(theBiggestNegativeIndex.first == KFF::wrongScoreResult);
	}
	
	SECTION("send empty vector of face descriptions")
	{
		descriptions.clear();
		std::pair<double, int> nullComparison = KnownFaces::instance()->findBestFaceFitFromFaces(descriptions, 1);
	
		REQUIRE(nullComparison.second == KFF::wrongIndexResult);
		REQUIRE(nullComparison.first == KFF::wrongScoreResult);
	}
} 

TEST_CASE("rmFace", "[KnownFaces]")
{
//	bool KnownFaces::rmFace(int index);

	REQUIRE(KnownFaces::instance()->getFacesAmount() == 4); // in previous TEST_CASE 4 faces were added
	
	FaceDescription face2 = KnownFaces::instance()->getFace(1);
	FaceDescription face3 = KnownFaces::instance()->getFace(2);
	KnownFaces::instance()->rmFace(1);
	
	REQUIRE(KnownFaces::instance()->getFacesAmount() == 3);
	REQUIRE(face3._description == KnownFaces::instance()->getFace(1)._description);
	REQUIRE(face3._id == KnownFaces::instance()->getFace(1)._id);
	
	for(size_t i = 0; i < KnownFaces::instance()->getFacesAmount(); i++)
	{
		REQUIRE(face2._description != KnownFaces::instance()->getFace(i)._description);
		REQUIRE(face2._id != KnownFaces::instance()->getFace(i)._id);
	}
}

TEST_CASE("clear", "[KnownFaces]")
{
//	void clear();

	REQUIRE(KnownFaces::instance()->getFacesAmount() == 3); // in previous TEST_CASE 3 faces were added
	
	KnownFaces::instance()->clear();
	
	REQUIRE(KnownFaces::instance()->getFacesAmount() == 0);
}
