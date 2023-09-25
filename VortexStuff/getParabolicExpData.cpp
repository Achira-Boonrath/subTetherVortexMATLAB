// Created by Eleonora Botta, March 28, 2019

// Read the interpolated residual gravity files for experiment on capture of envisat mockup
// Read the interpolated rotational velocity files for experiment on capture of envisat mockup

#include "MyLibraryNet/getParabolicExpData.h"

#include <iostream>
#include <sstream>
#include <fstream>

//#define MAXTIMESTEPSVALIDATION 20000

//using namespace Vx;
//using namespace VxDynamics;
//using namespace VxSim;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// read arrays of integers
void getParabolicExperimentResidualGravity(std::string pathData, float tstep, float myArray[][3])
{
	// initialize output 
	//int size = (int)(20.0 / tstep);
	//VxReal3[size] expGravity;
	//float myArray[3];

	// get the correct file, depending on simulation time step
	std::stringstream gravityDataFileName;
	if ( abs(tstep-0.0001) < 0.000001) //CW, 26 5 21, added new smallest timestep
		gravityDataFileName << pathData << "InterpolatedGrav_0_1ms.txt";
	else if (abs(tstep - 0.001) < 0.000001)
		gravityDataFileName << pathData << "InterpolatedGrav_1ms.txt";
	else if (abs(tstep - 0.01) < 0.000001)
		gravityDataFileName << pathData << "InterpolatedGrav_10ms.txt";

	std::ifstream gravityReadFileName;
	gravityReadFileName.open(gravityDataFileName.str());

	// read line by line
	std::string a, stringArray;
	if (gravityReadFileName.is_open())
	{
		int i = 0;
		// find the wanted line and put the elements of array in string stringArray
		do
		{
			std::string line, lineTemp;
			getline(gravityReadFileName, line);
			std::stringstream  lineStream(line);
			std::getline(lineStream, a, '\t');
			std::getline(lineStream, stringArray);
			std::stringstream streamArray(stringArray);
			float currentElement;
			int sizeArray = 0;
			while (std::getline(streamArray, a, '\t'))
			{
				currentElement = stof(a);
				// put element in myArray
				myArray[i][sizeArray] = currentElement;

				// increase sizeArray
				sizeArray = sizeArray + 1;
			}
			//expGravity[i].z() = myArray[0];
			//expGravity[i].x() = myArray[1];
			//expGravity[i].y() = myArray[2];
			/*expGravity[0] = myArray[0];
			expGravity[1] = myArray[1];
			expGravity[2] = myArray[2];*/
			i++;

		} while (!gravityReadFileName.eof());
	}
	gravityReadFileName.close();

	//return expGravity;
}


void getParabolicExperimentRotVel(std::string pathData, float tstep, float myArray[][3])
{
	// get the correct file, depending on simulation time step
	std::stringstream angVelDataFileName;
	if (abs(tstep - 0.0001) < 0.000001)
		angVelDataFileName << pathData << "InterpolatedAngVel_0_1ms.txt";
	else if (abs(tstep - 0.001) < 0.000001)
		angVelDataFileName << pathData << "InterpolatedAngVel_1ms.txt";
	else if (abs(tstep - 0.01) < 0.000001)
		angVelDataFileName << pathData << "InterpolatedAngVel_10ms.txt";

	std::ifstream angVelReadFileName;
	angVelReadFileName.open(angVelDataFileName.str());

	// read line by line
	std::string a, stringArray;
	if (angVelReadFileName.is_open())
	{
		int i = 0;
		// find the wanted line and put the elements of array in string stringArray
		do
		{
			std::string line, lineTemp;
			getline(angVelReadFileName, line);
			std::stringstream  lineStream(line);
			std::getline(lineStream, a, '\t');
			std::getline(lineStream, stringArray);
			std::stringstream streamArray(stringArray);
			float currentElement;
			int sizeArray = 0;
			while (std::getline(streamArray, a, '\t'))
			{
				currentElement = stof(a);
				// put element in myArray
				myArray[i][sizeArray] = currentElement;

				// increase sizeArray
				sizeArray = sizeArray + 1;
			}
			
			i++;

		} while (!angVelReadFileName.eof());
	}
	angVelReadFileName.close();

}

