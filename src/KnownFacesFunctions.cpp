#include <KnownFacesFunctions.hpp>


namespace KFF
{
	std::pair<double, int> findBestFaceFitFromFaces(std::vector<double>& description, std::vector<FaceDescription>& knownFaces, double minTreshold)
	{
		int best_index = wrongIndexResult;
		double best_score = wrongScoreResult;
		
		for(size_t i = 0; i < knownFaces.size(); i++)
		{
			if(description.size() == knownFaces.at(i)._description.size())
			{
				double comparisonResult = KFF::lengthBetweenFaceVectors(description, knownFaces.at(i)._description);
			
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
	
	std::pair<double, int> findBestFaceFitFromFaces(std::vector<double>& description, std::vector< std::vector<double> >& descriptionsVector, double minTreshold)
	{
		int best_index = wrongIndexResult;
		double best_score = wrongScoreResult;
		
		for(size_t i = 0; i < descriptionsVector.size(); i++)
		{
			if(description.size() == descriptionsVector.at(i).size())
			{
				double comparisonResult = KFF::lengthBetweenFaceVectors(description, descriptionsVector.at(i));
			
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
	
	bool areFaceVectorSimilar(std::vector<double>& firstFaceVector, std::vector<double>& secondFaceVector)
	{
		bool result;
		
		return result;
	}
	
	double lengthBetweenFaceVectors(std::vector<double>& firstFaceVector, std::vector<double>& secondFaceVector)
	{
		double result = std::numeric_limits<double>::max();
		
		if(firstFaceVector.size() == secondFaceVector.size())
		{
			double sum = 0.0;
			
			for(size_t i = 0; i < firstFaceVector.size(); i++)
			{
				double vectorElementSubstraction = firstFaceVector.at(i) - secondFaceVector.at(i);
				double substractionPower = std::pow(vectorElementSubstraction, 2); 
				sum += substractionPower; 
			}
			
			result = std::sqrt(sum);
		}
		
		return result;
	}
}
