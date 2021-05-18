#include <MainData.hpp>

namespace
{
	ros::Publisher clientPositionPublisher;
	ros::ServiceClient findFaceVectorsClient;
	ros::ServiceClient cutFaceClient;
	
	int focusedObjectID = -1;
	bool focusedObjectActivated = false;
	human_vision_exchange::FaceDescription focusedFaceVector;
	
	std::mutex _mutex;
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
	
	void sendFindFaceVectorsClientRequest(human_vision_exchange::FindFaceVectors& ffv)
	{
		findFaceVectorsClient.call(ffv);
	}
	
	void setFindFaceVectorsClient(ros::NodeHandle& node, std::string serviceName)
	{
		findFaceVectorsClient = node.serviceClient<human_vision_exchange::FindFaceVectors>(serviceName.c_str());
	}
	
	void sendCutFacesClientRequest(human_vision_exchange::CutFaces& cf)
	{
		cutFaceClient.call(cf);
	}
	
	void setCutFacesClient(ros::NodeHandle& node, std::string serviceName)
	{
		cutFaceClient = node.serviceClient<human_vision_exchange::CutFaces>(serviceName.c_str());
	}
	
	int getFocusedObjectID()
	{
		int result;
		
		_mutex.lock();
		result = focusedObjectID;
		_mutex.unlock();
		
		return result;
	}
	
	void setFocusedObjectID(const int newID)
	{
		_mutex.lock();
		focusedObjectID = newID;
		_mutex.unlock();
	}
	
	bool getFocusedObjectActivated()
	{
		bool result;
		
		_mutex.lock();
		result = focusedObjectActivated;
		_mutex.unlock();
		
		return result;
	}
	
	void setFocusedObjectActivated(const bool activation)
	{
		_mutex.lock();
		focusedObjectActivated = activation;
		_mutex.unlock();
	}
	
	human_vision_exchange::FaceDescription getFocusedFaceDescription()
	{
		human_vision_exchange::FaceDescription result;
		
		_mutex.lock();
		result = focusedFaceVector;
		_mutex.unlock();
		
		return result;
	}
	
	void setFocusedFaceDescription(const human_vision_exchange::FaceDescription& faceVector)
	{
		_mutex.lock();
		focusedFaceVector = faceVector;
		_mutex.unlock();
	}
}
