#ifndef KNOWN_FACES_HPP
#define KNOWN_FACES_HPP

#include <iostream>
#include <vector>
#include <mutex>
#include <FaceDescription.hpp>
#include <KnownFacesFunctions.hpp>

class KnownFaces
{
public:
	static KnownFaces* instance();
	void addFace(std::vector<double> description, int index = -1);
	FaceDescription getFace(int index);
	size_t getFacesAmount();
	bool rmFace(int index);
	void clear();
	
	// return fit score and face index, if not found return negative face index 
	std::pair<double, int> findBestFaceFitFromFaces(std::vector<double>& description, double minTreshold = 0.6); 
	std::pair<double, int> findBestFaceFitFromFaces(std::vector<std::vector<double>>& descriptionVector, int faceIndex, double minTreshold = 0.6); 
	
private:
	KnownFaces();
	
	std::mutex _mutex;
	std::vector<FaceDescription> _faces;
};

#endif
