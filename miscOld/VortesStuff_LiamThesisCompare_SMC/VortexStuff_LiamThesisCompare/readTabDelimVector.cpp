#include "MyLibraryNet/readTabDelimVector.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxMath/Transformation.h>
#include <VxMath/VxMath.h>
#include <Vx/VxQuaternion.h>


using namespace std;


Vx::VxVector3 readTabDelimVector(std::stringstream& fileName, int lineNumber)
{
	//std::cout << "I am reading " << fileName.str() << "to find " << "vector" << std::endl;
	std::ifstream inFile;
	inFile.open(fileName.str());

	std::string a, b, c, stringArray, stringArray2;
	std::string line, lineTemp;

	float currentElement;
	int sizeArray = 0;
	Vx::VxVector3 outVector;
	Vx::VxVector3* outVectorPtr = &outVector;
	int lineNum = 1;

	if (inFile.is_open())
	{
		// find the wanted line and put the elements of array in string stringArray
			
		while (lineNum < lineNumber + 1)
		{
			std::getline(inFile, line);
			if (lineNum == lineNumber)
			{
				std::stringstream  lineStream(line);
				std::getline(lineStream, a, '\t');
				std::getline(lineStream, stringArray);
				std::stringstream  lineStream2(stringArray);
				std::getline(lineStream2, b, '\t');
				std::getline(lineStream2, stringArray2);
				std::stringstream  lineStream3(stringArray2);
				std::getline(lineStream3, c, '\t');

				outVector[0] = stod(a);
				outVector[1] = stod(b);
				outVector[2] = stod(c);


			}
			lineNum = lineNum + 1;
		}
	}
	inFile.close();

	return outVector;
}


//std::getline(inFile, line);  // read one line
//std::stringstream  lineStream(line);
//std::getline(lineStream, a, '\t');
//std::getline(lineStream, stringArray);
//std::stringstream  lineStream2(stringArray);
//std::getline(lineStream2, b, '\t');
//std::getline(lineStream2, stringArray2);
//std::stringstream  lineStream3(stringArray2);
//std::getline(lineStream3, c, '\t');
//
//outVector[0] = stof(a);
//outVector[1] = stof(b);
//outVector[2] = stof(c);









