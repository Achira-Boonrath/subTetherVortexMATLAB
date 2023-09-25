// Created by Eleonora Botta, September 15, 2016
// Modified by Eleonora Botta, September 23, 2016: chaser passed as pointer, for the main to know it
// Modified by Eleonora Botta, September 26, 2016: possibility to have static chaser
// Modified by Eleonora Botta, December 20, 2016: autocompute inertia properties
// Modified by Eleonora Botta, January 19, 2017: no collision possible for chaser.
// Modified by Eleonora Botta, April 17, 2017:	added  #include <Vx/VxPart.h>, #include <Vx/VxCollisionRule.h>, #include <Vx/VxAssembly.h>  for Vortex Studio 2017a

// Create part for chaser spacecraft
// To do: add material

#include "MyLibraryNet/addChaser.h"
#include "MyLibraryNet/readParameter.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Box.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>

// for Vortex Studio 2017a
#include <Vx/VxPart.h>
#include <Vx/VxCollisionRule.h>
#include <Vx/VxAssembly.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

void addChaser(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part>* chaserPartPtr)
{
	// read data for chaser
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	VxReal chaserMass = readParameter<VxReal>("chaserMass", chaserDataFileName);
	VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);
	VxReal chaserDynamic = readParameter<VxReal>("chaserDynamic", chaserDataFileName);
	
	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	VxReal NetSideLength = readParameter<VxReal>("NetSideLength", netDataFileName);
	VxReal Z = readParameter<VxReal>("Z", netDataFileName);
	VxReal compression = readParameter<VxReal>("compression", netDataFileName);
	
	// read data for tether (winch)
	//std::stringstream tetherDataFileName;
	//tetherDataFileName << pathData << "tetherData.txt";
	//VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);

    // Create a dynamic part that contains a box as collision geometry 
	(*chaserPartPtr)->parameterMassPropertiesContainer.mass = chaserMass;
	VxSmartInterface<Box> box = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
	box->parameterDimension = VxVector3(chaserSideLength,chaserSideLength,chaserSideLength);
	VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
    cg = box;
	//(*chaserPartPtr)->addCollisionGeometry(cg);
	(*chaserPartPtr)->autoComputeInertiaAndCOM();
    //chaserPart->setControl(Vx::VxPart::kControlStatic);
	
	//Original
    //(*chaserPartPtr)->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(NetSideLength*compression/2, NetSideLength*compression/2-winchRadius, Z+chaserDistanceFromNet))); // center it with respect to net
	
	//AB, for box
	(*chaserPartPtr)->setLocalTransform(VxMath::Transformation::createTranslation(Vx::VxVector3(0, 0, Z -1.5)));
	if (chaserDynamic == 0) 
		(*chaserPartPtr)->inputControlType = VxDynamics::Part::kControlStatic;
	

    assembly->addPart(*chaserPartPtr);


	//AB, for Box
	//VxCollisionRule collisionRule1((*chaserPartPtr)->getVxPart(), false); 
	//assembly->getVxAssembly()->appendCollisionRule(collisionRule1);
}