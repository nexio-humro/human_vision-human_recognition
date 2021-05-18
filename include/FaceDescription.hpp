#ifndef FACE_DESCRIPTION_HPP
#define FACE_DESCRIPTION_HPP

#include <iostream>
#include <vector>

struct FaceDescription
{
	FaceDescription(): _id(-1) {}
	FaceDescription(int id, std::vector<double> desc):_id(id), _description(desc) {}
	
	int _id;
	std::vector<double> _description;
};

#endif
