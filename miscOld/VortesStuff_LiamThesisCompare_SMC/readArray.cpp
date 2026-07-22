// Created by Eleonora Botta, September 27, 2016: for int
// Modified by Eleonora Botta, September 29, 2016: for float
// Modified by Eleonora Botta, October 11, 2016: can now translate parameter "l" for int
// Modified by Eleonora Botta, October 12, 2016: can now read arrays of strings
// Modified by Eleonora Botta, January 29, 2017: added character Q for integers

// Read an array from input file

#include "MyLibraryNet/readArray.h"

#include <iostream>
#include <sstream>
#include <fstream>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// read arrays of integers
int readArray(std::string wantedParameter, std::stringstream& fileName, int myArray[], int intForCharacterC, int intForCharacterL ) 
{
	// open file
	std::ifstream inFile;
	inFile.open(fileName.str());

	std::string a,stringArray; 
	std::string line,lineTemp;

	int currentElement; 
	int sizeArray=0;
	if (inFile.is_open()) 
	{
		// find the wanted line and put the elements of array in string stringArray
		do
		{
			std::getline(inFile,line);  // read one line
			std::stringstream  lineStream(line);
			std::getline(lineStream,a,'\t');
			std::getline(lineStream,stringArray);
		}
		while ( (a != wantedParameter) && (!inFile.eof()));

		// send warning in case the quantity is not found
		if (a != wantedParameter)
		{
			stringArray = -1;
			std::cout << "\n Attention: the value of " << wantedParameter << " is not read from file! \n"; 
		}

		// put stringArray in stringstream
		std::stringstream streamArray(stringArray);

		while (std::getline(streamArray,a,';'))
		{
			// Transform string into integer
			if (a.find("c") != std::string::npos)
				currentElement = intForCharacterC; 
			else if (a.find("l") != std::string::npos)
				currentElement = intForCharacterL; 
			else
				currentElement = stoi(a); 
			
			// put element in myArray
			myArray[sizeArray] = currentElement;

			// increase sizeArray
			sizeArray = sizeArray+1;
		}
	}
	inFile.close();

    return sizeArray;
}


int readArray(std::string wantedParameter, std::stringstream& fileName, int myArray[], int intForCharacterC, int intForCharacterL, int intForCharacterQ, int intForCharacterP ) 
{
	// open file
	std::ifstream inFile;
	inFile.open(fileName.str());

	std::string a,stringArray; 
	std::string line,lineTemp;

	int currentElement; 
	int sizeArray=0;
	if (inFile.is_open()) 
	{
		// find the wanted line and put the elements of array in string stringArray
		do
		{
			std::getline(inFile,line);  // read one line
			std::stringstream  lineStream(line);
			std::getline(lineStream,a,'\t');
			std::getline(lineStream,stringArray);
		}
		while ( (a != wantedParameter) && (!inFile.eof()));

		// send warning in case the quantity is not found
		if (a != wantedParameter)
		{
			stringArray = -1;
			std::cout << "\n Attention: the value of " << wantedParameter << " is not read from file! \n"; 
		}

		// put stringArray in stringstream
		std::stringstream streamArray(stringArray);

		while (std::getline(streamArray,a,';'))
		{
			// Transform string into integer
			if (a.find("c") != std::string::npos)
				currentElement = intForCharacterC; 
			else if (a.find("l") != std::string::npos)
				currentElement = intForCharacterL; 
			else if (a.find("q") != std::string::npos)
				currentElement = intForCharacterQ; 
			else if (a.find("p") != std::string::npos)
				currentElement = intForCharacterP;
			else if (a.find("0") != std::string::npos)
				currentElement = 0; 
			else
				currentElement = stoi(a); 
			
			// put element in myArray
			myArray[sizeArray] = currentElement;

			// increase sizeArray
			sizeArray = sizeArray+1;
		}
	}
	inFile.close();

    return sizeArray;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// read arrays of floats
int readArray(std::string wantedParameter, std::stringstream& fileName, float myArray[] ) 
{
	// open file
	std::ifstream inFile;
	inFile.open(fileName.str());

	std::string a,stringArray; 
	std::string line,lineTemp;

	float currentElement; 
	int sizeArray=0;
	if (inFile.is_open()) 
	{
		// find the wanted line and put the elements of array in string stringArray
		do
		{
			std::getline(inFile,line);  // read one line
			std::stringstream  lineStream(line);
			std::getline(lineStream,a,'\t');
			std::getline(lineStream,stringArray);
		}
		while ( (a != wantedParameter) && (!inFile.eof()));

		// send warning in case the quantity is not found
		if (a != wantedParameter)
		{
			stringArray = -1;
			std::cout << "\n Attention: the value of " << wantedParameter << " is not read from file! \n"; 
		}

		// put stringArray in stringstream
		std::stringstream streamArray(stringArray);

		while (std::getline(streamArray,a,';'))
		{
			currentElement = stof(a); 
			
			// put element in myArray
			myArray[sizeArray] = currentElement;
			
			// increase sizeArray
			sizeArray = sizeArray+1;

		}
	}
	inFile.close();

    return sizeArray;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// read arrays of chars
int readArray(std::string wantedParameter, std::stringstream& fileName, std::string myArray[] ) 
{
	// open file
	std::ifstream inFile;
	inFile.open(fileName.str());

	std::string a,stringArray; 
	std::string line,lineTemp;

	int sizeArray=0;
	if (inFile.is_open()) 
	{
		// find the wanted line and put the elements of array in string stringArray
		do
		{
			std::getline(inFile,line);  // read one line
			std::stringstream  lineStream(line);
			std::getline(lineStream,a,'\t');
			std::getline(lineStream,stringArray);
		}
		while ( (a != wantedParameter) && (!inFile.eof()));

		// send warning in case the quantity is not found
		if (a != wantedParameter)
		{
			stringArray = -1;
			std::cout << "\n Attention: the value of " << wantedParameter << " is not read from file! \n"; 
		}

		// put stringArray in stringstream
		std::stringstream streamArray(stringArray);
		
		// parse the streamArray and put values in array
		while (std::getline(streamArray,a,';'))
		{
			// put element in myArray. a is already the string
			myArray[sizeArray] = a;
			
			// increase sizeArray
			sizeArray = sizeArray+1;			
		}
	}
	inFile.close();

    return sizeArray;
}
