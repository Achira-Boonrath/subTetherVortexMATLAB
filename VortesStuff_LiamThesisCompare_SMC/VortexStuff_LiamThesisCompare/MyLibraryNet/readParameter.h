// Created by Eleonora Botta, September 6, 2016

#ifndef READPARAMETER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define READPARAMETER_H

// Include all libraries needed by cpp files in this header
#include <iostream>

// define headers of cpp functions
template <typename Type> 
Type readParameter(const std::string& wantedParameter, std::stringstream& fileName) 
{
	std::cout << "I am reading " << fileName.str() << "to find " << wantedParameter << std::endl;
	Type valueParameter;
	std::ifstream inFile;

	inFile.open(fileName.str());
	std::string a; 
	if (inFile.is_open()) 
	{
		do 
		{ inFile >> a >> valueParameter;
		} while ( (a != wantedParameter) && (!inFile.eof()));

		if (a != wantedParameter)
		{
			valueParameter = -1;
			std::cout << "\n Attention: the value of " << wantedParameter << " is not read from file! \n"; 
		}
		/*std::cout << a << " = " << valueParameter << std::endl;
		std::cin >> a;*/
	}
	inFile.close();
    return valueParameter;
}

#endif