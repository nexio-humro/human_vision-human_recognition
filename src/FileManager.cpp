#include <FileManager.hpp>

namespace
{
	const char* endSequence = "\n";
	size_t endSequenceSize = 1;
}

namespace FM
{
	void writeLineToFile(const std::string& path, const std::string& text)
	{
	  std::fstream file;
	  file.open(path.c_str(), std::ios::in | std::ios::out | std::ios::binary);
	  if(file.good() == false)
	  {
		file.open(path.c_str(), std::ios::out | std::ios::binary);
	  }
	  file.seekp(0, std::ios::end);
	  file.write(text.c_str(), text.length());
	  file.write(endSequence, endSequenceSize);
	  file.close();
	}
	
	void saveFile(const std::string path, const std::string text)
	{
		FM::deleteFileIfExist(path);
		FM::writeLineToFile(path, text);
	}
	
	void deleteFileIfExist(const std::string path)
	{
		if (FILE *file = fopen(path.c_str(), "r")) 
		{
			fclose(file);
			remove(path.c_str());
		} 
	}
	
	void readAllFromFile(const std::string& path, std::vector<std::string> *table)
	{
		std::fstream file;
		std::string text;
		file.open(path.c_str(), std::ios::in | std::ios::binary);
		if(file.good() == false)
		{
			std::cout<<path<<" is not exist"<<std::endl;
		}
		else
		{
			while(!file.eof())
			{
				std::getline(file, text);
				table->push_back(text);
			}
			//~ table->pop_back();
		}
		file.close();
	}
}
