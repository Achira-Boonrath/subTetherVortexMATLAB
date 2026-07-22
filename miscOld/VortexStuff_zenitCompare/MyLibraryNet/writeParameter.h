// Created by Eleonora Botta, September 30, 2016
// To substitute one value in a file containing data

#ifndef WRITEPARAMETER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define WRITEPARAMETER_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <fstream>

// define headers of cpp functions
template <typename Type> 
void writeParameter(Type newParameter, const std::string& wantedParameter, std::stringstream& fileName) 
{
	// read all the input file and store it
	std::ifstream inFile;
	inFile.open(fileName.str());
	std::string a_array[30], b_array[30]; 
	std::string line,lineTemp;

	int i = 0;
	if (inFile.is_open()) 
	{
		while ( std::getline(inFile,line) ) // read one line
		{ 
			std::stringstream  lineStream(line);
			std::getline(lineStream,a_array[i],'\t');
			std::getline(lineStream,b_array[i]);
			
			i = i+1;
		} 
		
	}
	inFile.close();
	
	// create a copy to store the original file
	std::ofstream outFile;
	outFile.open("originalFile_nowModified.txt");
	for (int j = 0; j < i; j++)
	{
		outFile << a_array[j] << "\t" << b_array[j] << "\n";
	}
	outFile.close();

	// substitute the wanted parameter
	std::stringstream ssNewParameter;
	ssNewParameter << newParameter;
	std::string stringNewParameter = ssNewParameter.str();
	for (int j = 0; j < i; j++)
	{
		//if ( a_array[j] == stringWantedParameter )
		if (a_array[j].find(wantedParameter) != std::string::npos)
		{
			b_array[j] = stringNewParameter;
		}
	}

	// recreate the file to be read - now modified
	outFile.open(fileName.str());
	for (int j = 0; j < i; j++)
	{
		outFile << a_array[j] << "\t" << b_array[j] << "\n";
	}
	outFile.close();
	
  

}

#endif