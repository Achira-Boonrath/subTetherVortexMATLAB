// Created by Eleonora Botta, September 15, 2016
// Modified by Eleonora Botta, September 23, 2016: chaser passed as pointer, for the main to know it
// Modified by Eleonora Botta, September 26, 2016: possibility to have static chaser
// Modified by Eleonora Botta, December 20, 2016: autocompute inertia properties
// Modified by Eleonora Botta, January 19, 2017: no collision possible for chaser.
// Modified by Eleonora Botta, April 17, 2017:	added  #include <Vx/VxPart.h>, #include <Vx/VxCollisionRule.h>, #include <Vx/VxAssembly.h>  for Vortex Studio 2017a
// Modified by Liam Field, March 14, 2022: added initial conditions setting for chaser

// To do: 
// - Clean up
// - Add .txt input functionality

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
	VxReal chaserPosX = readParameter<VxReal>("chaserIPosX", chaserDataFileName);
	VxReal chaserPosY = readParameter<VxReal>("chaserIPosY", chaserDataFileName);
	VxReal chaserPosZ = readParameter<VxReal>("chaserIPosZ", chaserDataFileName);

	VxReal chaserVelX = readParameter<VxReal>("chaserIVelX", chaserDataFileName);
	VxReal chaserVelY = readParameter<VxReal>("chaserIVelY", chaserDataFileName);
	VxReal chaserVelZ = readParameter<VxReal>("chaserIVelZ", chaserDataFileName);

	VxReal chaserAngX = readParameter<VxReal>("chaserIAngRateX", chaserDataFileName);
	VxReal chaserAngY = readParameter<VxReal>("chaserIAngRateY", chaserDataFileName);
	VxReal chaserAngZ = readParameter<VxReal>("chaserIAngRateZ", chaserDataFileName);

	VxReal chaserQuatS = readParameter<VxReal>("chaserIQuatS", chaserDataFileName);
	VxReal chaserQuatX = readParameter<VxReal>("chaserIQuatX", chaserDataFileName);
	VxReal chaserQuatY = readParameter<VxReal>("chaserIQuatY", chaserDataFileName);
	VxReal chaserQuatZ = readParameter<VxReal>("chaserIQuatZ", chaserDataFileName);



    // Create a dynamic part that contains a box as collision geometry 
	(*chaserPartPtr)->parameterMassPropertiesContainer.mass = chaserMass;
	VxSmartInterface<Box> box = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
	box->parameterDimension = VxVector3(chaserSideLength,chaserSideLength,chaserSideLength);
	VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
    cg = box;
	(*chaserPartPtr)->addCollisionGeometry(cg);
	(*chaserPartPtr)->autoComputeInertiaAndCOM();
    //chaserPart->setControl(Vx::VxPart::kControlStatic);
	/*auto RA = VxMath::Transformation::createRotation(VxQuaternion(chaserQuatS, chaserQuatX, chaserQuatY, chaserQuatZ));
	auto TA = VxMath::Transformation::createTranslation(VxVector3(chaserPosX, chaserPosY, chaserPosZ));*/
    
	

	VxTransform BB;
	BB.setRotation(VxQuaternion(chaserQuatS, chaserQuatX, chaserQuatY, chaserQuatZ));
	BB.setTranslation(VxVector3(chaserPosX, chaserPosY, chaserPosZ));
	VxMath::Matrix44 tmBB;
	BB.get(tmBB);
	(*chaserPartPtr)->setLocalTransform(tmBB);
	
	
	//(*chaserPartPtr)->setLocalTransform(VxMath::Transformation::createTranslation(VxVector3(chaserPosX, chaserPosY, chaserPosZ))); // center it with respect to net
	//(*chaserPartPtr)->setLocalTransform(VxMath::Transformation::createRotation(VxQuaternion(chaserQuatS, chaserQuatX, chaserQuatY, chaserQuatZ)));	//double B = chaserQuat[0];
	Vx::VxReal4 Qvec;
	(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(Qvec); // center it with respect to net
	
	(*chaserPartPtr)->setAngularVelocity(VxVector3(chaserAngX,chaserAngY,chaserAngZ)); // center it with respect to net
	//(*chaserPartPtr)->setLinearVelocity(VxVector3(chaserVelX, chaserVelY, chaserVelZ)); // center it with respect to net
	(*chaserPartPtr)->setLinearVelocity(VxVector3(0, chaserVelY, chaserVelZ)); // center it with respect to net
		(*chaserPartPtr)->getVxPart()->setControl(Vx::VxPart::kControlDynamic);   
		
    assembly->addPart(*chaserPartPtr);
	
	VxCollisionRule collisionRule1((*chaserPartPtr)->getVxPart(), false); 
	assembly->getVxAssembly()->appendCollisionRule(collisionRule1);
}