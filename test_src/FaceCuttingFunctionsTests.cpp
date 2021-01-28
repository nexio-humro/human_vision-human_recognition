#include <catch.hpp>

#include <cmath>
#include <FaceCuttingFunctions.hpp>
#include <SystemFunctions.hpp>
#include <opencv2/imgcodecs.hpp>

TEST_CASE("getFrameMinMax", "[FaceCuttingFunctions]")
{
//	cv::Rect getFrameMinMax(std::vector<sl::float2>& points)

	std::vector<sl::float2> objectsPoints;
	for(size_t i = 0; i < FCF::bodyElementsAmount; i++)
	{
		objectsPoints.push_back(sl::float2(1*i, 2*i));
	}
	
	int smallest_index = 19;
	int biggest_index = -1;
	
	for(size_t i = 0; i < FCF::framePartsMinMax.size(); i++)
	{
		int actualIndex = getIdx(FCF::framePartsMinMax.at(i));
		
		if(actualIndex < smallest_index)
		{
			smallest_index = actualIndex;
		}
		
		if(actualIndex > biggest_index)
		{
			biggest_index = actualIndex;
		}
	}
		
	sl::float2 leftTop;
	if(smallest_index < FCF::bodyElementsAmount)
	{
		leftTop = objectsPoints.at(smallest_index);
	}
	
	sl::float2 rightBottom;
	if(biggest_index < FCF::bodyElementsAmount)
	{
		rightBottom = objectsPoints.at(biggest_index);
	}
	
	int width = rightBottom.x - leftTop.x;
	int height = rightBottom.y - leftTop.y;
	
	SECTION("send objectPoints without changes")
	{
		cv::Rect frame;
		frame = FCF::getFrameMinMax(objectsPoints);
		
		REQUIRE(leftTop.x == frame.x);
		REQUIRE(leftTop.y == frame.y);
		REQUIRE(width == frame.width);
		REQUIRE(height == frame.height);
	}
	
	SECTION("set X nose point to -1, before sending")
	{
		objectsPoints.at(getIdx(sl::BODY_PARTS::NOSE)).x = -1;
		cv::Rect frame;
		
		frame = FCF::getFrameMinMax(objectsPoints);
		
		REQUIRE(0 == frame.x);
		REQUIRE(0 == frame.y);
		REQUIRE(0 == frame.width);
		REQUIRE(0 == frame.height);
	}
	
	SECTION("set X left_knee point to -1, before sending")
	{
		objectsPoints.at(getIdx(sl::BODY_PARTS::LEFT_KNEE)).x = -1;
		cv::Rect frame;
		
		frame = FCF::getFrameMinMax(objectsPoints);
		
		REQUIRE(leftTop.x == frame.x);
		REQUIRE(leftTop.y == frame.y);
		REQUIRE(width == frame.width);
		REQUIRE(height == frame.height);
	}
}

TEST_CASE("getFrameEarNoseDistance", "[FaceCuttingFunctions]")
{
//	cv::Rect getFrameEarNoseDistance(std::vector<sl::float2>& points)

	std::vector<sl::float2> objectsPoints;
	for(size_t i = 0; i < FCF::bodyElementsAmount; i++)
	{
		objectsPoints.push_back(sl::float2(1*i, 2*i));
	}
	
	sl::float2 nose_point = objectsPoints.at( getIdx(sl::BODY_PARTS::NOSE) );
	sl::float2 left_ear_point = objectsPoints.at( getIdx(sl::BODY_PARTS::LEFT_EAR) );
	sl::float2 right_ear_point = objectsPoints.at( getIdx(sl::BODY_PARTS::RIGHT_EAR) );
	
	float biggest_distance;
	float left_nose = std::sqrt( std::pow((nose_point.x - left_ear_point.x),2) + std::pow((nose_point.y - left_ear_point.y),2) );
	float right_nose = std::sqrt( std::pow((nose_point.x - right_ear_point.x),2) + std::pow((nose_point.y - right_ear_point.y),2) );
	
	if(left_nose > right_nose)
	{
		biggest_distance = left_nose;
	}
	else
	{
		biggest_distance = right_nose;
	}
		
	sl::float2 leftTop( (nose_point.x - static_cast<int>(biggest_distance)), (nose_point.y - static_cast<int>(biggest_distance)) );
	sl::float2 rightBottom( (nose_point.x + static_cast<int>(biggest_distance)), (nose_point.y + static_cast<int>(biggest_distance)) );
	
	int width = rightBottom.x - leftTop.x;
	int height = rightBottom.y - leftTop.y;
	
	SECTION("send objectPoints without changes")
	{
		cv::Rect frame;
		frame = FCF::getFrameEarNoseDistance(objectsPoints);
		
		REQUIRE(leftTop.x == frame.x);
		REQUIRE(leftTop.y == frame.y);
		REQUIRE(width == frame.width);
		REQUIRE(height == frame.height);
	}
	
	SECTION("set X nose point to -1, before sending")
	{
		objectsPoints.at(getIdx(sl::BODY_PARTS::NOSE)).x = -1;
		cv::Rect frame;
		
		frame = FCF::getFrameEarNoseDistance(objectsPoints);
		
		REQUIRE(0 == frame.x);
		REQUIRE(0 == frame.y);
		REQUIRE(0 == frame.width);
		REQUIRE(0 == frame.height);
	}
	
	SECTION("set X left_knee point to -1, before sending")
	{
		objectsPoints.at(getIdx(sl::BODY_PARTS::LEFT_KNEE)).x = -1;
		cv::Rect frame;
		
		frame = FCF::getFrameEarNoseDistance(objectsPoints);
		
		REQUIRE(leftTop.x == frame.x);
		REQUIRE(leftTop.y == frame.y);
		REQUIRE(width == frame.width);
		REQUIRE(height == frame.height);
	}
	
	SECTION("set specific points for nose, left_ear, right_ear, before sending")
	{		
		objectsPoints.at(getIdx(sl::BODY_PARTS::LEFT_EAR)) = sl::float2(12, 12);
		objectsPoints.at(getIdx(sl::BODY_PARTS::RIGHT_EAR)) = sl::float2(22, 24);
		objectsPoints.at(getIdx(sl::BODY_PARTS::NOSE)) = sl::float2(16, 16);
		
		cv::Rect frame;
		frame = FCF::getFrameEarNoseDistance(objectsPoints);
		
		REQUIRE(6.0 == frame.x);
		REQUIRE(6.0 == frame.y);
		REQUIRE(20.0 == frame.width);
		REQUIRE(20.0 == frame.height);
		REQUIRE(frame.width == frame.height);
	}
}

TEST_CASE("areNosePointsCorrect", "[FaceCuttingFunctions]")
{
//	bool areHeadPointsCorrect(std::vector<sl::float2>& points);

	std::vector<sl::float2> objectsPoints;
	for(size_t i = 0; i < FCF::bodyElementsAmount; i++)
	{
		objectsPoints.push_back(sl::float2(1*i, 2*i));
	}
	
	SECTION("send correct version of object points")
	{
		REQUIRE(true == FCF::areNosePointsCorrect(objectsPoints));
	}
	
	SECTION("set X nose point to -1")
	{
		objectsPoints.at(getIdx(sl::BODY_PARTS::NOSE)).x = -1;
		REQUIRE(false == FCF::areNosePointsCorrect(objectsPoints));
	}
	            
	SECTION("remove last element")
	{
		objectsPoints.pop_back();
		REQUIRE(false == FCF::areNosePointsCorrect(objectsPoints));
	}
}

TEST_CASE("extractFace", "[FaceCuttingFunctions]")
{
//	cv::Mat extractFace(const cv::Mat& image, cv::Rect frame)    

	std::string image_path = SF::getPathToCurrentDirectory() + "../examples/cool_cat.jpg";
    cv::Mat img = imread(image_path, cv::IMREAD_COLOR);
    
    cv::Point leftTop(880, 620);
    cv::Point rightBottom(1500, 1080);
    cv::Rect rect(leftTop, rightBottom);
    
    cv::Mat frame = FCF::extractFace(img, rect);
    
//    std::string result_image_path = SF::getPathToCurrentDirectory() + "../output/frame_cool_cat.jpg";
//    cv::imwrite(result_image_path, result);
	
	SECTION("check few selected points")
	{	
		cv::Vec3b originalPixel;
		cv::Vec3b framePixel;
				
		auto findVectors = [&](cv::Point framPixelPosition)
		{
			originalPixel = img.at<cv::Vec3b>(framPixelPosition.y + rect.y, framPixelPosition.x + rect.x);
			framePixel = frame.at<cv::Vec3b>(framPixelPosition.y, framPixelPosition.x);
		};
		
		findVectors( cv::Point(0, 0) );
		REQUIRE(originalPixel == framePixel);
		
		findVectors( cv::Point(rect.width, 0) );
		REQUIRE(originalPixel == framePixel);
		
		findVectors( cv::Point(rect.width, rect.height) );
		REQUIRE(originalPixel == framePixel);
		
		findVectors( cv::Point(0, rect.height) );
		REQUIRE(originalPixel == framePixel);
		
		findVectors( cv::Point(199, 75) );
		REQUIRE(originalPixel == framePixel);
	}
	
	SECTION("check all pixels between original image and frame")
	{
		unsigned char* originalPointer;
		unsigned char* framePointer;
		bool arePixelsSame = true;
		
		for(size_t i = 0; i < frame.rows; ++i)
		{
			originalPointer = img.ptr<unsigned char>(i + rect.y);
			framePointer = frame.ptr<unsigned char>(i);
			
			for (size_t j = 0; j < frame.cols; ++j)
			{
				for(size_t k = 0; k < frame.channels(); k++)
				{
					if( !(framePointer[ j*frame.channels() + k ] == originalPointer[ (j + rect.x)*frame.channels() + k ]) )
					{
						arePixelsSame = false;
					}
				}
			}
			
			if(arePixelsSame == false)
			{
				break;
			}
		}
			
		REQUIRE(arePixelsSame == true);
	}

	SECTION("release original image")
	{
		img.release();
		
		REQUIRE(img.empty() == true);
		REQUIRE(frame.cols == rect.width);
		REQUIRE(frame.rows == rect.height);
	}
}

TEST_CASE("prepareCvFace", "[FaceCuttingFunctions]")
{
//	bool prepareCvFace(cv::Mat& faceImage, int imageSize);

	int imageWidth = 800;
	int imageHeight = 400;
	cv::Mat faceImage(imageHeight, imageWidth, CV_16UC4, cv::Scalar(255,0,0));
	int imageSize = 150;
	
	SECTION("send correct image and size format")
	{
		REQUIRE(FCF::prepareCvFace(faceImage, imageSize) == true);
		REQUIRE(faceImage.cols == imageSize);
		REQUIRE(faceImage.rows == imageSize);
		REQUIRE(faceImage.channels() == 3);
	}
	
	SECTION("send empty imageSize")
	{
		REQUIRE(FCF::prepareCvFace(faceImage, 0) == false);
		REQUIRE(faceImage.cols == imageWidth);
		REQUIRE(faceImage.rows == imageHeight);
		REQUIRE(faceImage.channels() == 4);
	}
	
	SECTION("send huge imageSize")
	{
		int hugeImageSize = 1000;
		
		REQUIRE(FCF::prepareCvFace(faceImage, hugeImageSize) == true);
		REQUIRE(faceImage.cols == hugeImageSize);
		REQUIRE(faceImage.rows == hugeImageSize);
		REQUIRE(faceImage.channels() == 3);
	}
	
	SECTION("send empty image")
	{
		cv::Mat emptyImage;
		
		REQUIRE(FCF::prepareCvFace(emptyImage, imageSize) == false);
		REQUIRE(emptyImage.empty() == true);
		REQUIRE(emptyImage.cols == 0);
		REQUIRE(emptyImage.rows == 0);
		REQUIRE(emptyImage.channels() == 1);
	}
	
}

TEST_CASE("isFrameCorrect", "[FaceCuttingFunctions]")
{
//	bool isFrameCorrect(cv::Rect& rect);

	int width = 30;
	int height = 79;
	cv::Mat image(height, width, CV_16SC3);
	cv::Rect frame;
	
	SECTION("check image")
	{
		REQUIRE(width == image.cols);
		REQUIRE(height == image.rows);
	}
	
	SECTION("send empty frame")
	{
		REQUIRE(false == FCF::isFrameCorrect(frame, image));
	}
	
	frame.x = 10;
	frame.y = 0;
	frame.width = 15;
	frame.height = 70;
	
	SECTION("send correct frame")
	{
		REQUIRE(true == FCF::isFrameCorrect(frame, image));
	}
	
	SECTION("set negative x in frame")
	{
		frame.x = -1;
		REQUIRE(false == FCF::isFrameCorrect(frame, image));
	}
	
	SECTION("set negative y in frame")
	{
		frame.y = -10;
		REQUIRE(false == FCF::isFrameCorrect(frame, image));
	}
	
	SECTION("set 0 width in frame")
	{
		frame.width = 0;
		REQUIRE(false == FCF::isFrameCorrect(frame, image));
	}
	
	SECTION("set 0 height in frame")
	{
		frame.height = 0;
		REQUIRE(false == FCF::isFrameCorrect(frame, image));
	}
	
	SECTION("send correct frame")
	{
		REQUIRE(true == FCF::isFrameCorrect(frame, image));
	}
}

TEST_CASE("extractAllFaces", "[FaceCuttingFunctions]")
{
//	std::pair< std::vector< dlib::matrix<dlib::rgb_pixel> >, std::vector<size_t> > extractAllFaces(sl::Mat& image, sl::Objects& objects, int imageSize = 150);   

/*	std::string image_path = SF::getPathToCurrentDirectory() + "../examples/cool_cat.jpg";
	sl::String image_path_sl(image_path.c_str());
    sl::Mat img;
    img.read(image_path_sl); */
    
    std::string image_path = SF::getPathToCurrentDirectory() + "../examples/cool_cat.jpg";
    cv::Mat img = imread(image_path, cv::IMREAD_COLOR);
    
    std::vector<sl::ObjectData> objects_vector;
    
    // first object
    sl::ObjectData firstObject;
    std::vector<sl::float2> firstObjectPoints;
    firstObjectPoints.resize(FCF::bodyElementsAmount);
    
    for(size_t i = 0; i < firstObjectPoints.size(); i++)
    {
		firstObjectPoints.at(i) = sl::float2(0.0, 0.0);
	}
    
    firstObjectPoints[getIdx(sl::BODY_PARTS::LEFT_EAR)] = sl::float2(180,200);
    firstObjectPoints[getIdx(sl::BODY_PARTS::RIGHT_EAR)] = sl::float2(230,210);
    firstObjectPoints[getIdx(sl::BODY_PARTS::NOSE)] = sl::float2(200,200);
    firstObject.keypoint_2d = firstObjectPoints;
    
    for(size_t i = 0; i < firstObjectPoints.size(); i++)
    {
		sl::float2 firstPoints = firstObjectPoints.at(i);
		sl::float2 secondPoints = firstObject.keypoint_2d.at(i);
		for(size_t j = 0; j < firstPoints.size(); j++)
		{
			REQUIRE(firstPoints[j] == secondPoints[j]);
		}
	} 
    
    // second object 
    sl::ObjectData secondObject;
    std::vector<sl::float2> secondObjectPoints;
    secondObjectPoints.resize(FCF::bodyElementsAmount);
    secondObjectPoints[getIdx(sl::BODY_PARTS::LEFT_EAR)] = sl::float2(500,480);
    secondObjectPoints[getIdx(sl::BODY_PARTS::RIGHT_EAR)] = sl::float2(540,480);
    secondObjectPoints[getIdx(sl::BODY_PARTS::NOSE)] = sl::float2(510,470);
    secondObject.keypoint_2d = secondObjectPoints;
    
    // third object
    sl::ObjectData thirdObject;
    std::vector<sl::float2> thirdObjectPoints;
    thirdObjectPoints.resize(FCF::bodyElementsAmount);
    thirdObjectPoints[getIdx(sl::BODY_PARTS::LEFT_EAR)] = sl::float2(900,480);
    thirdObjectPoints[getIdx(sl::BODY_PARTS::RIGHT_EAR)] = sl::float2(935,480);
    thirdObjectPoints[getIdx(sl::BODY_PARTS::NOSE)] = sl::float2(920,475);
    thirdObject.keypoint_2d = thirdObjectPoints;

	objects_vector.push_back(firstObject);
	objects_vector.push_back(secondObject);
	objects_vector.push_back(thirdObject);
	
	sl::Objects objects;
	objects.object_list = objects_vector;

	SECTION("sent correct data")
	{
		FCF::pairFacesImages result = FCF::extractAllFaces(img, objects);
		
		REQUIRE(result.first.size() == 3);   
		REQUIRE(result.second.size() == 3);   
		REQUIRE(result.first.at(0).nc() == 150);   
		REQUIRE(result.first.at(0).nr() == 150);   
		REQUIRE(result.first.at(1).nc() == 150);   
		REQUIRE(result.first.at(1).nr() == 150);   
		REQUIRE(result.first.at(2).nc() == 150);   
		REQUIRE(result.first.at(2).nr() == 150);
		   
		REQUIRE(result.second.at(0) == 0);   
		REQUIRE(result.second.at(1) == 1);   
		REQUIRE(result.second.at(2) == 2);   
	}
	
	SECTION("send wrong second object")
	{
//		int imageWidth = img.getWidth();
//		int imageHeight = img.getHeight();
		int imageWidth = img.cols;
		int imageHeight = img.rows;
		objects.object_list.at(1).keypoint_2d.at(getIdx(sl::BODY_PARTS::LEFT_EAR)) = sl::float2(imageWidth - 100, imageHeight);
		objects.object_list.at(1).keypoint_2d.at(getIdx(sl::BODY_PARTS::RIGHT_EAR)) = sl::float2(imageWidth + 10, imageHeight);
		objects.object_list.at(1).keypoint_2d.at(getIdx(sl::BODY_PARTS::NOSE)) = sl::float2(imageWidth - 10, imageHeight - 10);
		
		FCF::pairFacesImages result = FCF::extractAllFaces(img, objects);
		
		REQUIRE(result.first.size() == 2);   
		REQUIRE(result.second.size() == 2);   
		REQUIRE(result.first.at(0).nc() == 150);   
		REQUIRE(result.first.at(0).nr() == 150);   
		REQUIRE(result.first.at(1).nc() == 150);   
		REQUIRE(result.first.at(1).nr() == 150);   
		   
		REQUIRE(result.second.at(0) == 0);   
		REQUIRE(result.second.at(1) == 2);   
	}
}
