#include <KnownFaces.hpp>

KnownFaces* KnownFaces::instance()
{
	static KnownFaces* instance = nullptr;
	
	if(instance == nullptr)
	{
		instance = new KnownFaces;
	}
	
	return instance;
}

KnownFaces::KnownFaces()
{
	
}

void KnownFaces::addFace(std::vector<double> description, int index)
{
	FaceDescription newFaceDes;
	newFaceDes._description = description; 
	newFaceDes._id = index; 
	
	this->_mutex.lock();
	this->_faces.push_back(newFaceDes);
	this->_mutex.unlock();
}

FaceDescription KnownFaces::getFace(int index)
{
	FaceDescription result;
	
	this->_mutex.lock();
	if( (index < this->_faces.size()) && (index >= 0) )
	{
		result = this->_faces.at(index);
	}
	this->_mutex.unlock();
	
	return result;
}

size_t KnownFaces::getFacesAmount()
{
	size_t amount;
	
	this->_mutex.lock();
	amount = this->_faces.size();
	this->_mutex.unlock();
	
	return amount;
}

bool KnownFaces::rmFace(int index)
{
	bool result = true;
	
	if( (index < this->_faces.size()) && (index >= 0) )
	{
		this->_mutex.lock();
		this->_faces.erase(this->_faces.begin() + index);
		this->_mutex.unlock();
	}
	else
	{
		result = false;
	}
	
	return result;
}

void KnownFaces::clear()
{
	this->_mutex.lock();
	this->_faces.clear();
	this->_mutex.unlock();
}

std::pair<double, int> KnownFaces::findBestFaceFitFromFaces(std::vector<double>& description, double minTreshold)
{
	std::pair<double, int> result;
	
	this->_mutex.lock();
	result = KFF::findBestFaceFitFromFaces(description, this->_faces, minTreshold);
	this->_mutex.unlock();
	
	return result;
}

std::pair<double, int> KnownFaces::findBestFaceFitFromFaces(std::vector<std::vector<double>>& descriptionVector, int faceIndex, double minTreshold)
{
	std::pair<double, int> result(KFF::wrongScoreResult, KFF::wrongIndexResult);
	
	if( (faceIndex < this->_faces.size()) && (faceIndex >= 0) ) 
	{
		this->_mutex.lock();
		result = KFF::findBestFaceFitFromFaces(this->_faces.at(faceIndex)._description, descriptionVector, minTreshold);
		this->_mutex.unlock();
	}
	
	return result;
}

