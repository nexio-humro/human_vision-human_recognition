#include <MainData.hpp>

namespace
{
	ros::Publisher clientPositionPublisher;
}

namespace MD
{
	const ros::Publisher* getClientPositionPublisher()
	{
		return &clientPositionPublisher;
	}
	
	void setClientPositionPublisher(ros::NodeHandle& node, std::string topicName)
	{
		clientPositionPublisher = node.advertise<geometry_msgs::Point32>(topicName.c_str(), 10);
	}
}
