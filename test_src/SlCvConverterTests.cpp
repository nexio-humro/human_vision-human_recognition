#include <catch.hpp>

#include <SlCvConverter.hpp>

TEST_CASE("slMat2cvMat", "[SlCvConverter]")
{
//	cv::Mat slMat2cvMat(sl::Mat& input)
	
	SECTION("correct sl::Mat")
	{
		int imageWidth = 150;
		int imageHeight = 150;
		sl::Mat slMat(imageWidth, imageHeight, sl::MAT_TYPE::U8_C4);
		
		cv::Mat result = SCC::slMat2cvMat(slMat);
		
		REQUIRE(result.cols == slMat.getWidth());
		REQUIRE(result.rows == slMat.getHeight());
		REQUIRE(result.channels() == slMat.getChannels());
	}
	
	SECTION("empty sl::Mat")
	{
		sl::Mat emptySlMat;
		
		cv::Mat resultCv = SCC::slMat2cvMat(emptySlMat);
		
		REQUIRE(resultCv.cols == 0);
		REQUIRE(resultCv.rows == 0);
		REQUIRE(resultCv.channels() == 4);
	}
}
