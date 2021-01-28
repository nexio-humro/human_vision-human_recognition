#include <catch.hpp>

#include <OpenCvToDlibFunctions.hpp>

TEST_CASE("cvMat2dlibMatrix", "[OpenCvToDlibFuncitons]")
{
//	dlib::matrix<dlib::rgb_pixel> cvMat2dlibMatrix(cv::Mat& input);

	int imageWidth = 80;
	int imageHeight = 40;
	cv::Mat faceImage(imageHeight, imageWidth, CV_8UC3, cv::Scalar(255,0,0));
	
	SECTION("send cv::Mat with 3 channels")
	{
		dlib::matrix<dlib::rgb_pixel> result;
		result = OTDF::cvMat2dlibMatrix(faceImage);
		
		REQUIRE(faceImage.empty() == false);
		REQUIRE(result.nr() == faceImage.rows);
		REQUIRE(result.nc() == faceImage.cols);
	}

	SECTION("send cv::Mat with 4 channels")
	{
		cv::Mat faceImage(imageHeight, imageWidth, CV_8UC4, cv::Scalar(255,0,0));
		
		dlib::matrix<dlib::rgb_pixel> result;
		result = OTDF::cvMat2dlibMatrix(faceImage);
		
		REQUIRE(faceImage.empty() == false);
		REQUIRE(result.nr() == 0);
		REQUIRE(result.nc() == 0);
	}
	
	SECTION("send empty cv::Mat")
	{
		cv::Mat faceImage;
		
		dlib::matrix<dlib::rgb_pixel> result;
		result = OTDF::cvMat2dlibMatrix(faceImage);
		
		REQUIRE(faceImage.empty() == true);
		REQUIRE(result.nr() == 0);
		REQUIRE(result.nc() == 0);
	}
	
	SECTION("release cv::Mat after getting dlib::matrix")
	{	
		dlib::matrix<dlib::rgb_pixel> result;
		result = OTDF::cvMat2dlibMatrix(faceImage);
		faceImage.release();
		
		REQUIRE(faceImage.empty() == true);
		REQUIRE(result.nr() == imageHeight);
		REQUIRE(result.nc() == imageWidth);
	}
}
