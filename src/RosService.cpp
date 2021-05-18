#include <RosServices.hpp>

namespace RS
{
	bool processFaceVector(human_vision_exchange::FaceVectorReceiver::Request  &req, human_vision_exchange::FaceVectorReceiver::Response &res)
	{
		MD::setFocusedObjectActivated(true);
		MD::setFocusedFaceDescription(req.faceDescription);
		
		res.status = true;
		
		return true;
	}
}
