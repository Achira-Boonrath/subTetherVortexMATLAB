// Created by Eleonora Botta, September 27, 2016: for int
// Modified by Eleonora Botta, September 29, 2016: for float
// Modified by Eleonora Botta, October 11, 2016: added additional parameter for int
// Modified by Eleonora Botta, October 12, 2016: can now read arrays of strings
// Modified by Eleonora Botta, January 29, 2017: added character Q for integers

#ifndef READARRAY_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define READARRAY_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <sstream>
#include <fstream>

// define headers of cpp functions
int readArray(std::string wantedParameter, std::stringstream& fileName, int myArray[], int intForCharacterC, int intForCharacterL);

int readArray(std::string wantedParameter, std::stringstream& fileName, int myArray[], int intForCharacterC, int intForCharacterL, int intForCharacter, int intForCharacterP );

int readArray(std::string wantedParameter, std::stringstream& fileName, float myArray[] );

int readArray(std::string wantedParameter, std::stringstream& fileName, std::string myArray[] );

#endif


