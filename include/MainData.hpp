#ifndef MAIN_DATA_HPP
#define MAIN_DATA_HPP

#include <iostream>

#include <iostream>
#include <mutex>

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "human_vision_exchange/CutFaces.h"
#include "human_vision_exchange/FindFaceVectors.h"
#include "human_vision_exchange/FaceDescription.h"

namespace MD
{
	const ros::Publisher* getClientPositionPublisher();
	void setClientPositionPublisher(ros::NodeHandle& node, std::string topicName);
	
	void sendFindFaceVectorsClientRequest(human_vision_exchange::FindFaceVectors& ffv);
	void setFindFaceVectorsClient(ros::NodeHandle& node, std::string serviceName);
	void sendCutFacesClientRequest(human_vision_exchange::CutFaces& cf);
	void setCutFacesClient(ros::NodeHandle& node, std::string serviceName);
	
	int getFocusedObjectID();
	void setFocusedObjectID(const int newID);
	bool getFocusedObjectActivated();
	void setFocusedObjectActivated(const bool activation);
	human_vision_exchange::FaceDescription getFocusedFaceDescription();
	void setFocusedFaceDescription(const human_vision_exchange::FaceDescription& faceVector);
}

#endif
