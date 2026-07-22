// Created by Eleonora Botta, September 8, 2016

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/writeNaturalFrequencies.h"

#include <iostream>
#include <fstream>
#include <sstream>

void writeNaturalFrequencies(std::string pathData, float wn1Axial, float wn1BendingRot, float wn1BendingTransl)
{
	std::stringstream natFreqFileName;
	natFreqFileName << pathData << "naturalFrequencies.txt" ;
	std::ofstream outFile;

	outFile.open(natFreqFileName.str());
	std::string a; 
	if (outFile.is_open()) 
	{
		outFile << "wn1Axial \t" << wn1Axial << '\n'; 
		outFile << "wn1BendingRot \t" << wn1BendingRot << '\n'; 
		outFile << "wn1BendingTransl \t" << wn1BendingTransl << '\n';
	}
	outFile.close();
}