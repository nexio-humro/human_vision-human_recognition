#include <SystemFunctions.hpp>

namespace
{
	std::string pathToCurrentDirectory;
}

namespace SF
{
	std::string getPathToCurrentDirectory()
	{
		if(pathToCurrentDirectory == "")
		{
			pathToCurrentDirectory = ros::package::getPath("human_recognition") + "/src/";
		}
		
		return pathToCurrentDirectory;
	}
}
