#ifndef MAIN_FUNCTIONS_HPP
#define MAIN_FUNCTIONS_HPP

#include <iostream>
#include <mutex>

#include "ros/ros.h"
#include "human_vision_exchange/CutFaces.h"
#include "human_vision_exchange/FindFaceVectors.h"
#include "human_vision_exchange/Objects.h"
#include "BodyPartEnum.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <MainData.hpp>
#include <SystemFunctions.hpp>
#include <FileManager.hpp>

namespace MF
{
	const double wrongScoreResult = 9999999;
	const int wrongIndexResult = -1;
	
	void getFaceVectors(human_vision_exchange::Objects& objects, cv::Mat& photo, std::vector<human_vision_exchange::FaceDescription>& faceVectors);
	int findID_WithinObjects(human_vision_exchange::Objects& objects, size_t objectID);
	int findFaceVectorWithinObjects(human_vision_exchange::FaceDescription& faceDescription, std::vector<human_vision_exchange::FaceDescription>& faceDescriptionVector, double minTreshold = 0.6);
	
	double lengthBetweenFaceVectors(human_vision_exchange::FaceDescription& firstFaceVector, human_vision_exchange::FaceDescription& secondFaceVector);
	void saveFaceImages(human_vision_exchange::CutFaces::Response &res);
	void saveFaceDescriptions(human_vision_exchange::Objects& objects);
	void saveSceneImage(cv::Mat& sceneImage);
	size_t getCounter();
	void increaseCounter();
	
	// facenet 
	void getFaceVectorsFacenet(human_vision_exchange::Objects& objects, cv::Mat& photo, std::vector<human_vision_exchange::FaceDescriptionFacenet>& faceVectors);
	int findFaceVectorWithinObjectsFacenet(human_vision_exchange::FaceDescriptionFacenet& faceDescription, std::vector<human_vision_exchange::FaceDescriptionFacenet>& faceDescriptionVector, double minTreshold = 0.6);
	double lengthBetweenFaceVectors(human_vision_exchange::FaceDescriptionFacenet& firstFaceVector, human_vision_exchange::FaceDescriptionFacenet& secondFaceVector);
	
}

#endif
