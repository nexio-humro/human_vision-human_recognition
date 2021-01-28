#ifndef KNOWN_FACES_FUNCTIONS_HPP
#define KNOWN_FACES_FUNCTIONS_HPP

#include <iostream>
#include <vector>
#include <FaceDescription.hpp>

namespace KFF
{
	const double wrongScoreResult = 9999999;
	const int wrongIndexResult = -1;
	
	// return fit score and face index, if not found return negative face index 
	std::pair<double, int> findBestFaceFitFromFaces(dlib::matrix<float,0,1>& description, std::vector<FaceDescription>& knownFaces, double minTreshold); 
	std::pair<double, int> findBestFaceFitFromFaces(dlib::matrix<float,0,1>& description, std::vector< dlib::matrix<float,0,1> >& descriptionsVector, double minTreshold); 
}

#endif
