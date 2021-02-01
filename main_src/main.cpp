#include <sl/Camera.hpp>
#include <dlib/image_io.h>

#include <SystemFunctions.hpp>
#include <DlibFrontalFaceDetection.hpp>
#include <FaceCuttingFunctions.hpp>
#include <DlibFrontalFaceDetectionPaths.hpp>
#include <RosTopics.hpp>
#include <RosServices.hpp>
#include <MainData.hpp>

#include "ros/ros.h"
#include "ros/package.h"

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "human_recognition");
	ros::NodeHandle node("/human_recognition/");
	
	ros::Rate loop_rate(10);

	ros::Subscriber imageGetter = node.subscribe("/zed2/zed_node/left/image_rect_color", 1000, RT::grab_image);
	ros::Subscriber objectsGetter = node.subscribe("/zed2/zed_node/obj_det/objects", 1000, RT::grab_objects);
	ros::ServiceServer faceVectorService = node.advertiseService("facesVector", RS::processFaceVector);
	MD::setClientPositionPublisher(node, "clientPosition");
	
	std::string path = DFFDP::getPathToDetector();
	DlibFrontalFaceDetection::instance()->loadModel(path.c_str());
		
	FCF::pairFacesImages faces;
	
	while (ros::ok())
	{
		sl::Objects objects = ZD::getObjects();
		cv::Mat photo = ZD::getImage();
		faces = FCF::extractAllFaces(photo, objects);
		
		// save images
		if(true)
		{
			static size_t counter = 0;
			std::string pathImage = SF::getPathToCurrentDirectory() + "../output/image_" + std::to_string(counter) + ".png";
			cv::imwrite (pathImage.c_str(), photo);
			std::cout<<"-----------------------------"<<std::endl;
			std::cout<<"counter "<<counter<<" = "<<std::endl;
			for(size_t i = 0; i < faces.first.size(); i++)
			{
				std::string path = SF::getPathToCurrentDirectory() + "../output/face_" + std::to_string(counter) + "_" + std::to_string(i) + ".png";
				dlib::save_png (faces.first.at(i), path);
			}
			counter++;
		}
		
		std::vector<dlib::matrix<float,0,1>> faceDescriptions;
		faceDescriptions = DlibFrontalFaceDetection::instance()->computeFaceVectors(faces.first);

		// print face vector
		if(false)
		{
			std::cout<<"face vectors:"<<std::endl;
			for(size_t i = 0; i < faceDescriptions.size(); i++)
			{
				std::cout<<"face["<<i<<"]: cols = "<<faceDescriptions.at(i).nc()<<", rows = "<<faceDescriptions.at(i).nr()<<", size = "<<faceDescriptions.at(i).size()<<std::endl;
				for(size_t j = 0; j < faceDescriptions.at(i).size(); j++)
				{
					std::cout<<faceDescriptions.at(i)(j)<<std::endl;
				}
				std::cout<<"---------------------------"<<std::endl;
			}
		}

		// find face
		std::pair<double, int> findFace;
		findFace = KnownFaces::instance()->findBestFaceFitFromFaces(faceDescriptions, 0, 8.0);
		std::cout<<"face amount = "<<faceDescriptions.size()<<std::endl;
		std::cout<<"KnownFaces::instance()->getFacesAmount() = "<<KnownFaces::instance()->getFacesAmount()<<std::endl;
		std::cout<<"findFaee: result = "<<findFace.first<<", index = "<<findFace.second<<std::endl;
		
		if(findFace.second != -1)
		{
			geometry_msgs::Point32 humanPos;
//        	humanPos.x = objects.object_list.at(findFace.second).position[0];
//        	humanPos.y = objects.object_list.at(findFace.second).position[1];
//        	humanPos.z = objects.object_list.at(findFace.second).position[2];
			sl::float3 noseData = objects.object_list.at(findFace.second).keypoint[sl::getIdx(sl::BODY_PARTS::NOSE)];
        	humanPos.x = noseData[0];
        	humanPos.y = noseData[1];
        	humanPos.z = noseData[2];
//			std::cout<<"noseData[0] = "<<noseData[0]<<", noseData[1] = "<<noseData[1]<<", noseData[2] = "<<noseData[2]<<std::endl;
			MD::getClientPositionPublisher()->publish(humanPos);
		}

		ros::spinOnce();

		loop_rate.sleep();
	}
    
    return 0;
}
