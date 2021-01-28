#include <RosServices.hpp>

namespace RS
{
	bool processFaceVector(human_vision_exchange::FaceVectorReceiver::Request  &req, human_vision_exchange::FaceVectorReceiver::Response &res)
	{
		dlib::matrix<float,0,1> faceVectorData;
		faceVectorData.set_size(128);
		for(size_t i = 0; i < req.points.size(); i++)
		{
			faceVectorData(i) = req.points[i];
			std::cout<<i<<" = "<<req.points[i]<<std::endl;
		}
		
		KnownFaces::instance()->clear();
		KnownFaces::instance()->addFace(faceVectorData, 1);
		
		res.status = true;
		
		return true;
	}
}
