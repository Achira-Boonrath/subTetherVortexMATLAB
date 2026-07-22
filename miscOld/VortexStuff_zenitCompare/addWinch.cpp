// Created by Eleonora Botta, September 19, 2016
// Modified by Eleonora Botta, September 26, 2016: working hinge between chaser and winch (locked, motorized)
// Modified by Eleonora Botta, September 28, 2016: winch passed as pointer to createScene for main to know it at runtime
// Modified by Eleonora Botta, September 29, 2016: hinge passed as pointer to createScene for main to know it at runtime
// Modified by Eleonora Botta, October 12, 2016: take initial conditions from vectors with controls, velocities, accelerations for winch spooling
// Modified by Eleonora Botta, December 20, 2016: autocompute inertia properties
// Modified by Eleonora Botta, April 17, 2017:	added #include <Vx/VxCollisionRule.h>, #include <Vx/VxAssembly.h>  for Vortex Studio 2017a

// Create winch to spool tether

#include "MyLibraryNet/addWinch.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/readArray.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Box.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>
#include <Vx/VxCylinder.h>
#include <Vx/VxTransform.h>
#include <VxDynamics/Hinge.h>
#include <Vx/VxHinge.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/Cylinder.h>
#include <Vx/VxEulerAngles.h>

// for Vortex Studio 2017a
#include <Vx/VxCollisionRule.h>
#include <Vx/VxAssembly.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

void addWinch(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part>* winchPartPtr, VxSmartInterface<Hinge>* hingeForWinchPtr)
{
	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	VxReal NetSideLength = readParameter<VxReal>("NetSideLength", netDataFileName);
	VxReal Z = readParameter<VxReal>("Z", netDataFileName);
	VxReal compression = readParameter<VxReal>("compression", netDataFileName);

	// read data for chaser 
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);

	// read data for winch - in tetherData file
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);
	std::string winchSpoolingFunctionControls[20];
	float winchSpoolingFunctionVelocities[20];
	int sizeWinchSpoolingFunctionControls = readArray("winchSpoolingFunctionControls", tetherDataFileName, winchSpoolingFunctionControls);
	int sizeWinchSpoolingFunctionVelocities = readArray("winchSpoolingFunctionVelocities", tetherDataFileName, winchSpoolingFunctionVelocities);
	
	// create winch part
	(*winchPartPtr)->getVxPart()->setControl(Vx::VxPart::kControlDynamic);   
	Vx::VxVector3 winchPosition = VxVector3(NetSideLength*compression/2, NetSideLength*compression/2-winchRadius, Z+chaserDistanceFromNet-chaserSideLength/2);
	(*winchPartPtr)->setLocalTransform(VxMath::Transformation::createTranslation(winchPosition));
	
	// add collision geometry to winch
	VxSmartInterface<Cylinder> cylinderWinch = VxExtensionFactory::create(VxDynamics::Cylinder::kFactoryKey); 
	cylinderWinch->parameterRadius = winchRadius;
	cylinderWinch->parameterHeight = 0.02; //TODO: make input
	(*winchPartPtr)->parameterMassPropertiesContainer.mass = 0.01; //TODO: make input
	VxSmartInterface<CollisionGeometry> cgWinch = VxExtensionFactory::create(VxDynamics::Cylinder::kFactoryKey); 
	cgWinch = cylinderWinch;
	VxTransform tm;
	tm.makeRotation(Vx::VxEulerAngles(0.0, VX_HALF_PI, 0.0));
	(*winchPartPtr)->addCollisionGeometry(cgWinch);
	cgWinch->getVxCollisionGeometry()->setTransformRelative(tm);
    (*winchPartPtr)->autoComputeInertiaAndCOM();

	// create hinge between chaser and winch, centered in center of the winch
	VxVector3 axis(1.0, 0.0, 0.0);
	const VxVector3 part2Offset = VxMath::Transformation::getTranslation((*winchPartPtr)->getLocalTransform()) - VxMath::Transformation::getTranslation(chaserPart->getLocalTransform());
	(*hingeForWinchPtr)->inputAttachment1.part = (*winchPartPtr);
	(*hingeForWinchPtr)->inputAttachment2.part = chaserPart;
	(*hingeForWinchPtr)->inputAttachment1.position = VxVector3(0.0,0.0,0.0);
	(*hingeForWinchPtr)->inputAttachment2.position = part2Offset;
	(*hingeForWinchPtr)->inputAttachment1.primaryAxis = axis;
	
	// set control of the winch, either free or locked or motorized. If motorized, give the right angular velocity
	// set hinge is locked
	if (winchSpoolingFunctionControls[0].find("l") != std::string::npos)
		(*hingeForWinchPtr)->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlLocked); 
				
	// set hinge is free
	else if (winchSpoolingFunctionControls[0].find("f") != std::string::npos)
		(*hingeForWinchPtr)->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlFree); 

	// set hinge is motorized: set also its velocity
	else if (winchSpoolingFunctionControls[0].find("m") != std::string::npos)
	{
		(*hingeForWinchPtr)->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlMotorized); 					
		float winchAngularVelocity = winchSpoolingFunctionVelocities[0] / winchRadius; // (rad/s)
		(*hingeForWinchPtr)->getVxConstraint()->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, winchAngularVelocity); 
	} 

	// add winch and hinge to assembly
	assembly->addPart((*winchPartPtr));
	assembly->addConstraint((*hingeForWinchPtr));
	/*VxCollisionRule collisionRule1(chaserPart->getVxPart(), (*winchPartPtr)->getVxPart(), false); 
	assembly->getVxAssembly()->appendCollisionRule(collisionRule1);*/

}