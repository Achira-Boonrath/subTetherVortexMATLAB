// Perform checks on data inserted in the input files, and tell the user to change them, if data don't make sense

// Created by Eleonora Botta, September 19, 2016
// Modified by Eleonora Botta, September 20, 2016: winch modes and number of errors
// Modified by Eleonora Botta, September 30, 2016: rings, function for winch velocity, initial winch velocity if locked
// Modified by Eleonora Botta, October 11, 2016: changed readArray according to new version
// Modified by Eleonora Botta, October 28, 2016: condition on motorized tether and winchRadius
//												 condition on tstep and tsave	
// Modified by Eleonora Botta, June 16, 2017: if target is present and is envisat mockup, check that gravity data for the chosen tstep exists

// To do: other checks

#include "MyLibraryNet/checkData.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/writeParameter.h"
#include "MyLibraryNet/readArray.h"

#include <Vx/VxMessage.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace Vx;

#define MAXGRIDSIZE 100


void checkData( std::string pathData)
{
	// Read data

	// read data for simulation
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt" ;
	int tetherPresent = readParameter<int>("tetherPresent", simulationDataFileName);
	int chaserPresent = readParameter<int>("chaserPresent", simulationDataFileName);
	int targetPresent = readParameter<int>("targetPresent", simulationDataFileName);
	double tstep = readParameter<double>("tstep", simulationDataFileName);
	double tsave = readParameter<double>("tsave", simulationDataFileName);
	
	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	float winchRadius = readParameter<float>("winchRadius", tetherDataFileName);
	
	int ringPartX[MAXGRIDSIZE];
	int ringPartY[MAXGRIDSIZE];
	int sizeRingPartX = readArray("ringsPartX", tetherDataFileName, ringPartX, 0, 0, 0, 0);
	int sizeRingPartY = readArray("ringsPartY", tetherDataFileName, ringPartY, 0, 0, 0, 0);
	
	float winchSpoolingFunctionTimes[20];
	std::string winchSpoolingFunctionControls[20];
	float winchSpoolingFunctionVelocities[20];
	float winchSpoolingFunctionAccelerations[20];
	int sizeWinchSpoolingFunctionTimes = readArray("winchSpoolingFunctionTimes", tetherDataFileName, winchSpoolingFunctionTimes);
	int sizeWinchSpoolingFunctionControls = readArray("winchSpoolingFunctionControls", tetherDataFileName, winchSpoolingFunctionControls);
	int sizeWinchSpoolingFunctionVelocities = readArray("winchSpoolingFunctionVelocities", tetherDataFileName, winchSpoolingFunctionVelocities);
	int sizeWinchSpoolingFunctionAccelerations = readArray("winchSpoolingFunctionAccelerations", tetherDataFileName, winchSpoolingFunctionAccelerations);
	
	// read data for target
	std::stringstream targetDataFileName;
	targetDataFileName << pathData << "targetData.txt" ;
	std::string targetSpacecraft = readParameter<std::string>("targetSpacecraft", targetDataFileName);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Initialize a quantity to give used the number of errors in the data
	int numErrors = 0;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	// Give messages about errors in data

	// If there is a winch, there must be a tether
	if ( (tetherPresent==0) && (winchRadius>0) )
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThere is no tether but there is a winch. If you want a tether, check tetherPresent=1 in file simulationData! \n");
	}

	// if there is a tether, there must be a chaser, and vice-versa
	/*if ( ( (tetherPresent==0) && (chaserPresent ==1) ) || ( (tetherPresent==1) && (chaserPresent == 0) ) )
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nIf you have a tether, you need a chaser and vice-versa. If you want a tether and a chaser, check tetherPresent=1 and chaserPresent=1 in file simulationData! \n");
	}*/

	// check number of rings against maxgridsize and that x and y are in the same number 
	if ( ( sizeRingPartY != sizeRingPartX) || ( sizeRingPartY > MAXGRIDSIZE ) || ( sizeRingPartX > MAXGRIDSIZE ) )
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThe number of elements in ringsPartX and ringsPartY must be equal, and less than MAXGRIDSIZE. Please check in file tetherData! \n");
	}

	// check number of acceleration, velocities, controls and times specified for winch velocity, and that first element of time == 0
	if ( sizeWinchSpoolingFunctionTimes != sizeWinchSpoolingFunctionAccelerations)
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThe number of elements in winchSpoolingFunctionTimes and winchSpoolingFunctionAccelerations must be equal. Please check in file tetherData! \n");
	}
	if ( sizeWinchSpoolingFunctionTimes != sizeWinchSpoolingFunctionControls)
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThe number of elements in winchSpoolingFunctionTimes and winchSpoolingFunctionControls must be equal. Please check in file tetherData! \n");
	}
	if ( sizeWinchSpoolingFunctionTimes != sizeWinchSpoolingFunctionVelocities)
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThe number of elements in winchSpoolingFunctionTimes and winchSpoolingFunctionVelocities must be equal. Please check in file tetherData! \n");
	}
	if ( sizeWinchSpoolingFunctionTimes >= 20)
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nswinchSpoolingFunctionTimes must have maximum 20 elements. Please check in file tetherData! \n");
	}
	if ( winchSpoolingFunctionTimes[0] != 0)
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThe first element of winchSpoolingFunctionTimes must be 0. Please check in file tetherData! \n");
	}

	// If the tether is motorized, there must be a winch
	if ( (tetherPresent == 1) && (winchSpoolingFunctionControls[0].find("m") != std::string::npos) && (winchRadius<=0) )
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThere is no winch, but you asked for a motorized tether. If you want a motorized tether, specify winchRadius in file tetherData! \n");
	}

	// Save time must be a multiple of time step
	/*if ( (int)(1.0/tstep) % (int)(1.0/tsave) != 0) 
	{
		numErrors = numErrors+1;
		std::cout <<  (1.0/tstep) << std::endl;
		std::cout <<  (int)(1.0/tstep) << std::endl;
		std::cout <<  (int)(1.0/tsave) << std::endl;
		std::cout <<  (int)(1.0/tstep) % (int)(1.0/tsave) << std::endl;

		Vx::LogWarn("\ntsave must be equal to or a multiple of tstep. Please make sure of this in simulationData! \n");
	}*/

	if ( (targetPresent ==1) && (targetSpacecraft=="envisat") && ( (tstep != 0.001) && (tstep != 0.01) ) )
	{
		numErrors = numErrors+1;
		Vx::LogWarn("\nThere is no data on gravity for the chosen time step. Please check in file simulationData! \n");
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Print an error message and wait for user input acknowledging he/she read the errors
	if (numErrors>0)
	{
		std::cout << std::endl << "ERROR: Make sure you read the " << numErrors << " errors and how to fix them. \n ";
		std::cout << std::endl << "ERROR: Now exit the simulation and fix the errors. \n ";
		int errorCheck;
		std::cin >> errorCheck;
	}

}