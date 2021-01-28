#include <catch.hpp>

#include <cmath>
#include <DlibFrontalFaceDetection.hpp>
#include <DlibFrontalFaceDetectionPaths.hpp>

#include <dlib/image_saver/save_png.h>

TEST_CASE("class", "[DlibFrontalFaceDetection]")
{
	SECTION("instance")
	{
		REQUIRE(DlibFrontalFaceDetection::instance() != nullptr);
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == false);
	}

	SECTION("load model wrong path")
	{
		REQUIRE(DlibFrontalFaceDetection::instance()->loadModel("") == false);
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == false);
	}
	
	SECTION("load correct model")
	{
		std::string path = DFFDP::getPathToDetector();
		REQUIRE(DlibFrontalFaceDetection::instance()->loadModel(path.c_str()) == true);
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == true);
	}
	
	SECTION("load correct model twice")
	{
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == true);
		std::string path = DFFDP::getPathToDetector();
		REQUIRE(DlibFrontalFaceDetection::instance()->loadModel(path.c_str()) == true);
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == true);
	}
}

TEST_CASE("computeFaceVectors", "[DlibFrontalFaceDetection]")
{
	std::vector< dlib::matrix<dlib::rgb_pixel> > faces;
	dlib::matrix<dlib::rgb_pixel> firstFace = dlib::uniform_matrix<dlib::rgb_pixel, 150, 150>(dlib::rgb_pixel(255,0,0));
	dlib::matrix<dlib::rgb_pixel> secondFace = dlib::uniform_matrix<dlib::rgb_pixel, 150, 150>(dlib::rgb_pixel(0,255,0));
	dlib::matrix<dlib::rgb_pixel> thirdFace = dlib::uniform_matrix<dlib::rgb_pixel, 150, 150>(dlib::rgb_pixel(0,0,255));
	
	// save firstFace image in output directory
	if(false)
	{
		std::string firstImagePath = SF::getPathToCurrentDirectory() + "/../output/red_frame.png";
		dlib::save_png(firstFace, firstImagePath.c_str());
	}
	
	faces.push_back(firstFace);
	faces.push_back(secondFace);
	faces.push_back(thirdFace);
	
	SECTION("computeFaceVectors check results amounts")
	{
		std::vector<dlib::matrix<float,0,1>> result;
		
		if(DlibFrontalFaceDetection::instance()->isModelLoaded() == false)
		{
			std::string path = DFFDP::getPathToDetector();
			DlibFrontalFaceDetection::instance()->loadModel(path.c_str());
		}
		
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == true);

		result = DlibFrontalFaceDetection::instance()->computeFaceVectors(faces);
		
		REQUIRE(result.size() == faces.size());
		for(size_t i = 0; i < result.size(); i++)
		{
			REQUIRE(result.at(i).size() == 128);
		}
		
		REQUIRE(result.at(0) != result.at(1));
		REQUIRE(result.at(0) == result.at(0));
	}
	
	SECTION("load model twice and check if results are same")
	{
		std::string path = DFFDP::getPathToDetector();
		
		DlibFrontalFaceDetection::instance()->loadModel(path.c_str());
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == true);
		std::vector<dlib::matrix<float,0,1>> firstResult = DlibFrontalFaceDetection::instance()->computeFaceVectors(faces);
		
		DlibFrontalFaceDetection::instance()->loadModel(path.c_str());
		REQUIRE(DlibFrontalFaceDetection::instance()->isModelLoaded() == true);
		std::vector<dlib::matrix<float,0,1>> secondResult = DlibFrontalFaceDetection::instance()->computeFaceVectors(faces);
		
		REQUIRE(firstResult == secondResult);
	}
}
