#include <ZedData.hpp>

namespace
{
	std::mutex zedDataMutex;
	
	cv::Mat imageData;
	std::vector<human_vision_exchange::Keypoints2d> keypoints2dVector;
	human_vision_exchange::Objects objectsData;
}

namespace ZD
{
	void saveImage(cv_bridge::CvImagePtr image)
	{
		zedDataMutex.lock();
		imageData = image->image.clone();
		zedDataMutex.unlock();
	}
	
	cv::Mat getImage()
	{
		cv::Mat result;
		
		zedDataMutex.lock();
		result = imageData.clone();
		zedDataMutex.unlock();
		
		return result;
	}
	
	void saveKeypoints2d(std::vector<human_vision_exchange::Keypoints2d>& objects)
	{
		zedDataMutex.lock();
		keypoints2dVector = objects;
		zedDataMutex.unlock();
	}
	
	std::vector<human_vision_exchange::Keypoints2d> getKeypoints2d()
	{
		std::vector<human_vision_exchange::Keypoints2d> result;
		
		zedDataMutex.lock();
		result = keypoints2dVector;
		zedDataMutex.unlock();
		
		return result;
	}
	
	void saveObjects(const human_vision_exchange::Objects& objects)
	{
		zedDataMutex.lock();
		objectsData = objects;
		zedDataMutex.unlock();
	}
	
	human_vision_exchange::Objects getObjects()
	{
		human_vision_exchange::Objects result;
		
		zedDataMutex.lock();
		result = objectsData;
		zedDataMutex.unlock();
		
		return result;
	}
}
