#ifndef MAIN_DATA_HPP
#define MAIN_DATA_HPP

#include <iostream>

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"

namespace MD
{
	const ros::Publisher* getClientPositionPublisher();
	void setClientPositionPublisher(ros::NodeHandle& node, std::string topicName);
}

#endif
