#ifndef FILE_MANAGER_HPP
#define FILE_MANAGER_HPP

#include <iostream>
#include <fstream>
#include <vector>

namespace FM
{
	void writeLineToFile(const std::string& path, const std::string& text);
	void saveFile(const std::string path, const std::string text);
	void deleteFileIfExist(const std::string path);
	void readAllFromFile(const std::string& path, std::vector<std::string> *table);
}

#endif

