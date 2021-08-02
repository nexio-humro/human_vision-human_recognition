#ifndef ROS_SERVICES_HPP
#define ROS_SERVICES_HPP

#include <iostream>

#include "ros/ros.h"

#include "human_vision_exchange/FaceVectorReceiver.h"
#include "human_vision_exchange/FaceVectorReceiverFacenet.h"

#include <KnownFaces.hpp>
#include <MainData.hpp>

namespace RS
{
	bool processFaceVector(human_vision_exchange::FaceVectorReceiver::Request  &req, human_vision_exchange::FaceVectorReceiver::Response &res);
	bool processFaceVectorFacenet(human_vision_exchange::FaceVectorReceiverFacenet::Request  &req, human_vision_exchange::FaceVectorReceiverFacenet::Response &res);
}

#endif
