#include <MainFunctions.hpp>

namespace 
{
	std::mutex mutexData;
	static size_t counter = 0;
}

namespace MF
{
	void getFaceVectors(human_vision_exchange::Objects& objects, cv::Mat& photo, std::vector<human_vision_exchange::FaceDescription>& faceVectors)
	{
		faceVectors.clear(); 
		 
		// cut faces
		human_vision_exchange::CutFaces cutFaces;
		cutFaces.request.keypoints.resize(objects.objects.size());
		for(size_t i = 0; i < objects.objects.size(); i++)
		{
			for(size_t j = 0; j < objects.objects[i].keypoint_2d.size(); j++)
			{
				cutFaces.request.keypoints[i].points[j] = objects.objects[i].keypoint_2d[j];
			}
		}
				
		cv_bridge::CvImage cvImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", photo);	
		cvImage.toImageMsg(cutFaces.request.image);
				
		MD::sendCutFacesClientRequest(cutFaces);
		
		std::cout<<"getFaceVectors(): objects.objects.size() = "<<objects.objects.size()<<std::endl;
		std::cout<<"getFaceVectors(): faces amount = "<<cutFaces.response.faces.size()<<std::endl;
		
		// save images
		if(true)
		{
			MF::saveSceneImage(photo);
			MF::saveFaceImages(cutFaces.response);
		}
				
		if(cutFaces.response.faces.size() > 0)
		{
			// find faces vectors
			human_vision_exchange::FindFaceVectors findFaceVectors;
			findFaceVectors.request.faces = cutFaces.response.faces;
					
			MD::sendFindFaceVectorsClientRequest(findFaceVectors);
			
			if(findFaceVectors.response.faceVectors.faces.size() > 0)
			{
				faceVectors.resize(findFaceVectors.response.faceVectors.faces.size());
				
				for(size_t i = 0; i < findFaceVectors.response.faceVectors.faces.size(); i++)
				{
					for(size_t j = 0; j < findFaceVectors.response.faceVectors.faces[i].points.size(); j++)
					{
						faceVectors.at(i).points[j] = findFaceVectors.response.faceVectors.faces[i].points[j];
					}
				}
			}
		}

		MF::increaseCounter();
	}
	 
	int findID_WithinObjects(human_vision_exchange::Objects& objects, size_t objectID)
	{
		 int result = -1;
		 
		 for(int i = 0; i < objects.objects.size(); i++)
		 {
			 std::cout<<"MF::findID_WithinObjects(): label_id = "<<objects.objects[i].label_id<<std::endl;
			 if(objects.objects[i].label_id == objectID)
			 {
				 result = i;
				 break;
			 }
		 }
		 
		 return result;
	}
	 
	int findFaceVectorWithinObjects(human_vision_exchange::FaceDescription& faceDescription, std::vector<human_vision_exchange::FaceDescription>& faceDescriptionVector, double minTreshold)
	{
		int best_index = wrongIndexResult;
		double best_score = wrongScoreResult;
		
		for(size_t i = 0; i < faceDescriptionVector.size(); i++)
		{
			if(faceDescription.points.size() == faceDescriptionVector.at(i).points.size())
			{
				double comparisonResult = MF::lengthBetweenFaceVectors(faceDescription, faceDescriptionVector.at(i));
				std::cout<<"findFaceVectorWithinObjects(): comparisonResult = "<<comparisonResult<<std::endl;
			
				if( (comparisonResult < minTreshold) && (comparisonResult < best_score) )
				{
					best_index = i;
					best_score = comparisonResult;
				}   
			}
		}
		
		return best_index;
	}
	
	double lengthBetweenFaceVectors(human_vision_exchange::FaceDescription& firstFaceVector, human_vision_exchange::FaceDescription& secondFaceVector)
	{
		double result = std::numeric_limits<double>::max();
		
		if(firstFaceVector.points.size() == secondFaceVector.points.size())
		{
			double sum = 0.0;
			
			for(size_t i = 0; i < firstFaceVector.points.size(); i++)
			{
				double vectorElementSubstraction = firstFaceVector.points[i] - secondFaceVector.points[i];
				double substractionPower = std::pow(vectorElementSubstraction, 2); 
				sum += substractionPower; 
			}
			
			result = std::sqrt(sum);
		}
		
		return result;
	}
	
	void saveFaceImages(human_vision_exchange::CutFaces::Response &res)
	{
		std::cout<<"saveFaceImages(): faces amount = "<<res.faces.size()<<", counter "<<MF::getCounter()<<std::endl;	  
		for(size_t i = 0; i < res.faces.size(); i++)
		{
			std::string path = SF::getPathToCurrentDirectory() + "../output/face_" + std::to_string(MF::getCounter()) + "_" + std::to_string(i) + ".png";
//			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(res.faces[i], sensor_msgs::image_encodings::BGR8);
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(res.faces[i], sensor_msgs::image_encodings::RGB8);
			cv::Mat cvImage = cv_ptr->image;
			
			// swap faceImage channels imwrtie needs cm::mat in BGR
			cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR);
			cv::imwrite(path, cvImage);
		}
	}
	
	void saveSceneImage(cv::Mat& sceneImage)
	{
		std::cout<<"saveSceneImage()"<<std::endl;
		std::string pathImage = SF::getPathToCurrentDirectory() + "../output/image_" + std::to_string(MF::getCounter()) + ".png";
		cv::imwrite (pathImage.c_str(), sceneImage);
	}

	void removeImages()
	{
		std::filesystem::remove_all(SF::getPathToCurrentDirectory() + "../output/");
	}
	
	size_t getCounter()
	{
		size_t result;
		
		mutexData.lock();
		result = counter;
		mutexData.unlock();
		
		return result;
	}
	
	void increaseCounter()
	{
		mutexData.lock();
		counter++;
		mutexData.unlock();
	}
	
	void getFaceVectorsFacenet(human_vision_exchange::Objects& objects, cv::Mat& photo, std::vector<human_vision_exchange::FaceDescriptionFacenet>& faceVectors)
	{
		faceVectors.clear(); 
		 
		// cut faces
		human_vision_exchange::CutFaces cutFaces;
		cutFaces.request.keypoints.resize(objects.objects.size());
		for(size_t i = 0; i < objects.objects.size(); i++)
		{
			for(size_t j = 0; j < objects.objects[i].keypoint_2d.size(); j++)
			{
				cutFaces.request.keypoints[i].points[j] = objects.objects[i].keypoint_2d[j];
			}
		}
				
		cv_bridge::CvImage cvImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", photo);	
		cvImage.toImageMsg(cutFaces.request.image);
				
		MD::sendCutFacesClientRequest(cutFaces);
		
		std::cout<<"getFaceVectorsFacenet(): objects.objects.size() = "<<objects.objects.size()<<std::endl;
		std::cout<<"getFaceVectorsFacenet(): faces amount = "<<cutFaces.response.faces.size()<<std::endl;
		
		// save images
		if(true)
		{
			MF::saveSceneImage(photo);
			MF::saveFaceImages(cutFaces.response);
		}
		
		// save 2D keypoints
		if(true)
		{
			MF::saveKeypoints(objects);
		}
				
		if(cutFaces.response.faces.size() > 0)
		{
			// find faces vectors
			human_vision_exchange::FindFaceVectorsFacenet findFaceVectorsFacenet;
			findFaceVectorsFacenet.request.faces = cutFaces.response.faces;
					
			MD::sendFindFaceVectorsFacenetClientRequest(findFaceVectorsFacenet);
			
			if(findFaceVectorsFacenet.response.faceVectors.faces.size() > 0)
			{
				faceVectors.resize(findFaceVectorsFacenet.response.faceVectors.faces.size());
				
				for(size_t i = 0; i < findFaceVectorsFacenet.response.faceVectors.faces.size(); i++)
				{
					for(size_t j = 0; j < findFaceVectorsFacenet.response.faceVectors.faces[i].points.size(); j++)
					{
						faceVectors.at(i).points[j] = findFaceVectorsFacenet.response.faceVectors.faces[i].points[j];
					}
				}
			}
		}
		
		// save faceDescriptions
		if(true)
		{
			MF::saveFaceDescriptions(faceVectors);
		}

		MF::removeImages();
		
		MF::increaseCounter();
	}
	
	int findFaceVectorWithinObjectsFacenet(human_vision_exchange::FaceDescriptionFacenet& faceDescription, std::vector<human_vision_exchange::FaceDescriptionFacenet>& faceDescriptionVector, double minTreshold)
	{
		std::cout<<"findFaceVectorWithinObjectsFacenet(): start"<<std::endl;
		int best_index = wrongIndexResult;
		double best_score = wrongScoreResult;
		
		for(size_t i = 0; i < faceDescriptionVector.size(); i++)
		{
			if(faceDescription.points.size() == faceDescriptionVector.at(i).points.size())
			{
				double comparisonResult = MF::lengthBetweenFaceVectors(faceDescription, faceDescriptionVector.at(i));
				std::cout<<"findFaceVectorWithinObjectsFacenet(): comparisonResult = "<<comparisonResult<<std::endl;
			
				if( (comparisonResult < minTreshold) && (comparisonResult < best_score) )
				{
					best_index = i;
					best_score = comparisonResult;
				}   
			}
		}
		
		return best_index;
	}
	
	double lengthBetweenFaceVectors(human_vision_exchange::FaceDescriptionFacenet& firstFaceVector, human_vision_exchange::FaceDescriptionFacenet& secondFaceVector)
	{
		double result = std::numeric_limits<double>::max();
		
		if(firstFaceVector.points.size() == secondFaceVector.points.size())
		{
			double sum = 0.0;
			
			for(size_t i = 0; i < firstFaceVector.points.size(); i++)
			{
				double vectorElementSubstraction = firstFaceVector.points[i] - secondFaceVector.points[i];
				double substractionPower = std::pow(vectorElementSubstraction, 2); 
				sum += substractionPower; 
			}
			
			result = std::sqrt(sum);
		}
		
		return result;
	}
	
	void saveKeypoints(human_vision_exchange::Objects& objects)
	{
		for(size_t i = 0; i < objects.objects.size(); i++)
		{
			std::string path = SF::getPathToCurrentDirectory() + "../output/keypoints_" + std::to_string(MF::getCounter()) + "_" + std::to_string(i);
			std::stringstream keypointsStream;
			for(size_t j = 0; j < objects.objects[i].keypoint_2d.size(); j++)
			{
				keypointsStream<<objects.objects[i].keypoint_2d[j].x<<"\t"<<objects.objects[i].keypoint_2d[j].y<<std::endl;
			}
			FM::saveFile(path, keypointsStream.str());
		}
	}
	
	void saveFaceDescriptions(std::vector<human_vision_exchange::FaceDescriptionFacenet>& faceDescriptionsVector)
	{
		for(size_t i = 0; i < faceDescriptionsVector.size(); i++)
		{
			std::string path = SF::getPathToCurrentDirectory() + "../output/faceDescription_" + std::to_string(MF::getCounter()) + "_" + std::to_string(i);
			std::stringstream faceDescriptionStream;
			for(size_t j = 0; j < faceDescriptionsVector.at(i).points.size(); j++)
			{
				faceDescriptionStream<<faceDescriptionsVector.at(i).points[j]<<std::endl;
			}
			FM::saveFile(path, faceDescriptionStream.str());
		}
	}
}
