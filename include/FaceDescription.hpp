#ifndef FACE_DESCRIPTION_HPP
#define FACE_DESCRIPTION_HPP

#include <iostream>
#include <dlib/matrix.h>

struct FaceDescription
{
	FaceDescription(): _id(-1) {}
	FaceDescription(int id, dlib::matrix<float,0,1> desc):_id(id), _description(desc) {}
	
	int _id;
	dlib::matrix<float,0,1> _description;
};

#endif
