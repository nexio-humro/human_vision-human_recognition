#include <SystemFunctions.hpp>
#include <RosTopics.hpp>
#include <RosServices.hpp>
#include <MainData.hpp>
#include <MainFunctions.hpp>

#include "ros/ros.h"
#include "ros/package.h"

#include <human_vision_exchange/CutFaces.h>

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "human_recognition");
	ros::NodeHandle node("/human_recognition/");
	
	ros::Rate loop_rate(10);

	ros::Subscriber imageGetter = node.subscribe("/human_zed_talker/leftImage", 1000, RT::grab_image);
	ros::Subscriber objectsGetter = node.subscribe("/human_zed_talker/objects", 1000, ZD::saveObjects);
//	ros::ServiceServer faceVectorService = node.advertiseService("facesVector", RS::processFaceVector);
	ros::ServiceServer faceVectorService = node.advertiseService("facesVectorFacenet", RS::processFaceVectorFacenet);
	MD::setClientPositionPublisher(node, "clientPosition");
	MD::setCutFacesClient(node, "/human_face_cutter/cutFaces");
	MD::setFindFaceVectorsFacenetClient(node, "/human_facenet_face_recognition/findFaceVectors");
//	MD::setFindFaceVectorsClient(node, "/human_dlib_face_recognition/findFaceVectors");
	
	while (ros::ok())
	{
		std::cout<<std::endl<<"--------------------------------"<<std::endl;
		human_vision_exchange::Objects objects = ZD::getObjects();
		cv::Mat photo = ZD::getImage();
		int focusedObjectID = MD::getFocusedObjectID();
//		human_vision_exchange::FaceDescription focusedObjectFaceDescription = MD::getFocusedFaceDescription();
		human_vision_exchange::FaceDescriptionFacenet focusedObjectFaceDescriptionFacenet = MD::getFocusedFaceDescriptionFacenet();
		
		std::cout<<"main(): focusedObjectID = "<<focusedObjectID<<std::endl;
//		std::cout<<"counter = "<<MF::getCounter()<<std::endl;
		
		// check if tracking object is activated
		if(MD::getFocusedObjectActivated() == true)
		{
			// chcek if tracking object ID is pressented on scene
			int focused_ID_Index;
			focused_ID_Index = MF::findID_WithinObjects(objects, focusedObjectID);
			
			// display objects and focused_ID_Index
			if(true)
			{
				std::cout<<"main(): focused_ID_Index = "<<focused_ID_Index<<std::endl;
				
				std::cout<<"avaible object indexes: ";
				for(size_t i = 0; i < objects.objects.size(); i++)
				{
					std::cout<<objects.objects[i].label_id<<", ";
				}
				std::cout<<std::endl; 
			}
			
			if(focused_ID_Index >= 0)
			{
				// send focused object nose coordinates
				geometry_msgs::Point32 humanNosePos;
				humanNosePos = objects.objects[focused_ID_Index].keypoint_3d[static_cast<int>(BODY_PARTS::NOSE)];
				
//				std::cout<<"humanNosePos = "<<std::endl<<humanNosePos<<std::endl;	
				
				if(humanNosePos != geometry_msgs::Point32())
				{	
					MD::getClientPositionPublisher()->publish(humanNosePos);
				}
			}
			else
			{
				// check if tracking faceVector is presseneted on scene
//				std::vector<human_vision_exchange::FaceDescription> faceDescriptionVector;
//				MF::getFaceVectors(objects, photo, faceDescriptionVector);
				
				std::vector<human_vision_exchange::FaceDescriptionFacenet> faceDescriptionVectorFacenet;
				MF::getFaceVectorsFacenet(objects, photo, faceDescriptionVectorFacenet);
				
				int new_focused_ID_Index = -1;
//				new_focused_ID_Index = MF::findFaceVectorWithinObjects(focusedObjectFaceDescription, faceDescriptionVector);
				new_focused_ID_Index = MF::findFaceVectorWithinObjectsFacenet(focusedObjectFaceDescriptionFacenet, faceDescriptionVectorFacenet);

				// temporary when face_recognition is not working
/*				if( !(objects.objects.size() == 0) )
				{
					new_focused_ID_Index = objects.objects[0].label_id;
				}*/
				
				if(new_focused_ID_Index >= 0)
				{
					// change tracking object ID
					std::cout<<"setFocusedObjectID: new_focused_ID_Index = "<<new_focused_ID_Index<<", itteration = "<<MF::getCounter()<<std::endl;
					MD::setFocusedObjectID(objects.objects[new_focused_ID_Index].label_id);
				} 
			}	
		}

		ros::spinOnce();

		loop_rate.sleep();
	}
    
    return 0;
}
