#include <KnownFacesFunctions.hpp>


namespace KFF
{
	std::pair<double, int> findBestFaceFitFromFaces(dlib::matrix<float,0,1>& description, std::vector<FaceDescription>& knownFaces, double minTreshold)
	{
		int best_index = wrongIndexResult;
		double best_score = wrongScoreResult;
		
		for(size_t i = 0; i < knownFaces.size(); i++)
		{
			if(description.size() == knownFaces.at(i)._description.size())
			{
				double comparisonResult = dlib::length(description - knownFaces.at(i)._description);
			
				if( (comparisonResult < minTreshold) && (comparisonResult < best_score) )
				{
					best_index = i;
					best_score = comparisonResult;
				}
			}
		}
		
		std::pair<double, int> result(best_score, best_index);
		
		return result;
	}
	
	std::pair<double, int> findBestFaceFitFromFaces(dlib::matrix<float,0,1>& description, std::vector< dlib::matrix<float,0,1> >& descriptionsVector, double minTreshold)
	{
		int best_index = wrongIndexResult;
		double best_score = wrongScoreResult;
		
		for(size_t i = 0; i < descriptionsVector.size(); i++)
		{
			if(description.size() == descriptionsVector.at(i).size())
			{
				double comparisonResult = dlib::length(description - descriptionsVector.at(i));
			
				if( (comparisonResult < minTreshold) && (comparisonResult < best_score) )
				{
					best_index = i;
					best_score = comparisonResult;
				}
			}
		}
		
		std::pair<double, int> result(best_score, best_index);
		
		return result;
	}
}
