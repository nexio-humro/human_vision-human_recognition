#ifndef KNOWN_FACES_FUNCTIONS_HPP
#define KNOWN_FACES_FUNCTIONS_HPP

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <FaceDescription.hpp>

namespace KFF
{
	const double wrongScoreResult = 9999999;
	const int wrongIndexResult = -1;
	
	// return fit score and face index, if not found return negative face index 
	std::pair<double, int> findBestFaceFitFromFaces(std::vector<double>& description, std::vector<FaceDescription>& knownFaces, double minTreshold); 
	std::pair<double, int> findBestFaceFitFromFaces(std::vector<double>& description, std::vector< std::vector<double> >& descriptionsVector, double minTreshold); 
	
	bool areFaceVectorSimilar(std::vector<double>& firstFaceVector, std::vector<double>& secondFaceVector);
	double lengthBetweenFaceVectors(std::vector<double>& firstFaceVector, std::vector<double>& secondFaceVector);
}

#endif
