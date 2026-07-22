// Created by Eleonora Botta, September 16, 2016
// Modified by Eleonora Botta, September 19, 2016: set position, orientation, angular velocity depending on inputs
// Modified by Eleonora Botta, October 4, 2016: you can now capture Zenit
// Modified by Eleonora Botta, June 13, 2017: you can now capture Envisat mock-up, which is frozen
// Modified by Eleonora Botta, June 15, 2017: corrected the position of Envisat mock-up

// Load an existing scene with a target spacecraft, depending on the input file

#include "MyLibraryNet/addTarget.h"

#include "MyLibraryNet/readParameter.h"
#include <Vx/VxSmartPtr.h>
#include <VxSim/VxApplication.h>
#include <VxSim/VxDynamicsModuleICD.h>
#include <VxSim/VxSimulatorModule.h> //was <VxSim/VxSimulatorModuleFactory.h> before
#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxSimulationFileManager.h>

//#include <Vx/Find.h> //#include <VxSim/FindInterface.h> // not needed in the new version
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Box.h>
#include <VxDynamics/MassPropertiesContainer.h>

#include <iostream>
#include <fstream>
#include <cmath>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


VxPart* addTarget(Vx::VxSmartPtr<VxSim::VxApplication> application, std::string pathData)
{
	// read constants
	std::stringstream constantsFileName;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for simulation
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt" ;
	int targetPresent = readParameter<int>("targetPresent", simulationDataFileName);

	if (targetPresent == 1)
	{
		// read data for target
		std::stringstream targetDataFileName;
		targetDataFileName << pathData << "targetData.txt" ;
		VxReal offsetX = readParameter<VxReal>("offsetX", targetDataFileName);
		VxReal offsetY = readParameter<VxReal>("offsetY", targetDataFileName);
		VxReal targetDistanceFromNet = readParameter<VxReal>("targetDistanceFromNet", targetDataFileName);		
		VxReal targetAngularVelocityMagnitude = readParameter<VxReal>("targetAngularVelocityMagnitude", targetDataFileName);
		VxReal targetAngleWithXaxis = readParameter<VxReal>("targetAngleWithXaxis", targetDataFileName);
		std::string targetSpacecraft = readParameter<std::string>("targetSpacecraft", targetDataFileName);
			
		// read data for net
		std::stringstream netDataFileName;
		netDataFileName << pathData << "netData.txt";
		VxReal NetSideLength = readParameter<VxReal>("NetSideLength", netDataFileName);
		VxReal Z = readParameter<VxReal>("Z", netDataFileName);
		VxReal compression = readParameter<VxReal>("compression", netDataFileName);

		Vx::VxPart* targetSpacecraftPart;
		// Get the simulation file manager from the application so as to load and unload scenes. 
		VxSim::VxSimulationFileManager* fileManager = application->getSimulationFileManager();

		// construct file name for target
		std::stringstream targetFilePath;
		targetFilePath << "Models/SpaceScene/Capture_" << targetSpacecraft << ".vxscene"; // far si che legga nome e prenda file corretto: zenit,apollo, different orientations 
		std::string sceneName = targetFilePath.str();
		// load file with target
		if (!fileManager->loadObject(sceneName))
		{
			Vx::LogError("Could not load scene file: %s!\n", sceneName);
		}
		std::cout << "Loaded file with at path: " << sceneName  << std::endl;

		// retrieve part 
		Vx::VxUniverse* universe = VxFrame::instance()->getUniverse(0);
		if (universe != nullptr)
		{
			for (VxSet<Vx::VxPart *>::const_iterator it = universe->getParts().begin(); it != universe->getParts().end(); ++it)
			{
				Vx::VxPart * part = *it;
				if (part->getName() == targetSpacecraft)
				{
					targetSpacecraftPart = part;
					std::cout << " I FOUND the part you wanted, called " << targetSpacecraft << std::endl; 
				}
				else
				{
					std::cout << " I DID NOT FIND the part you wanted, called " << targetSpacecraft << std::endl; 
				}
			}
		}
		
		if (targetSpacecraft != "envisat")
		{
			// give desired angular velocity to target spacecraft
			VxReal targetAngleWithXaxisRad = targetAngleWithXaxis * PI / 180.0;
			VxReal targetAngularVelocityMagnitudeRads = targetAngularVelocityMagnitude * PI / 180.0;
			targetSpacecraftPart->setPosition( 0.0 , -10 , 0.0);   
			targetSpacecraftPart->setOrientationEulerAngles( targetAngleWithXaxisRad, 0.0, 0.0);
			VxVector3 targetAngularVelocityVector = VxVector3(0, targetAngularVelocityMagnitudeRads * cos(targetAngleWithXaxisRad), targetAngularVelocityMagnitudeRads * sin(targetAngleWithXaxisRad) );
			targetSpacecraftPart->setAngularVelocity(targetAngularVelocityVector);
			std::cout << "Ang. vel vector is "<< targetAngularVelocityVector.x() << " " << targetAngularVelocityVector.y() << " " << targetAngularVelocityVector.z() << std::endl;
		}
		else if (targetSpacecraft != "cube")
		{	// for direct connection to tether
			VxReal targetAngleWithXaxisRad = targetAngleWithXaxis * PI / 180.0;
			VxReal targetAngularVelocityMagnitudeRads = targetAngularVelocityMagnitude * PI / 180.0;
			targetSpacecraftPart->setPosition(0,0,-30);
			targetSpacecraftPart->setOrientationEulerAngles(targetAngleWithXaxisRad, 0.0, 0.0);
			VxVector3 targetAngularVelocityVector = VxVector3(0, targetAngularVelocityMagnitudeRads * cos(targetAngleWithXaxisRad), targetAngularVelocityMagnitudeRads * sin(targetAngleWithXaxisRad));
			targetSpacecraftPart->setAngularVelocity(targetAngularVelocityVector);
			std::cout << "Ang. vel vector is " << targetAngularVelocityVector.x() << " " << targetAngularVelocityVector.y() << " " << targetAngularVelocityVector.z() << std::endl;
			
		}
		else
		{
			VxReal targetAngleWithXaxisRad = targetAngleWithXaxis * PI / 180.0;
			VxReal targetAngularVelocityMagnitudeRads = targetAngularVelocityMagnitude * PI / 180.0;
			targetSpacecraftPart->setPosition(NetSideLength * compression / 2.0 + offsetX, NetSideLength * compression / 2.0 + offsetY, Z + targetDistanceFromNet);
			targetSpacecraftPart->setOrientationEulerAngles(targetAngleWithXaxisRad, 0, 0);
			targetSpacecraftPart->freeze();
		}
		// return part
		return targetSpacecraftPart;
	}
}
