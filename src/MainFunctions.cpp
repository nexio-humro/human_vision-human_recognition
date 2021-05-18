#include <MainFunctions.hpp>

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
						faceVectors.at(0).points[j] = findFaceVectors.response.faceVectors.faces[i].points[j];
					}
				}
			}
		}
	}
	 
	int findID_WithinObjects(human_vision_exchange::Objects& objects, size_t objectID)
	{
		 int result = -1;
		 
		 for(int i = 0; i < objects.objects.size(); i++)
		 {
			 if(objects.objects[i].label_id == objectID)
			 {
				 result = objectID;
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
		std::cout<<"saveFaceImages(): faces amount = "<<res.faces.size()<<std::endl;	  
		for(size_t i = 0; i < res.faces.size(); i++)
		{
			std::string path = SF::getPathToCurrentDirectory() + "../output/face_" + std::to_string(counter) + "_" + std::to_string(i) + ".png";
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(res.faces[i], sensor_msgs::image_encodings::BGR8);
			cv::Mat cvImage = cv_ptr->image;
			cv::imwrite(path, cvImage);
		}
		 
		counter++;
	}
	
	void saveSceneImage(cv::Mat& sceneImage)
	{
		std::cout<<"saveSceneImage()"<<std::endl;
		std::string pathImage = SF::getPathToCurrentDirectory() + "../output/image_" + std::to_string(counter) + ".png";
		cv::imwrite (pathImage.c_str(), sceneImage);
	}
}
