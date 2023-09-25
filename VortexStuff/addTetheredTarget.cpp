// Created by Liam Field 03/09/22
// 
// Modified by Liam Field, March 14, 2022: added initial conditions setting for target

// To do: 
// - Clean up
// - Add .txt input functionality

#include "MyLibraryNet/addTetheredTarget.h"
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

void addTetheredTarget(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part>* tetheredTargetPartPtr)
{
	const double PI = 3.141592653589793238463;

	int targetPoint = 1;

	// read data for target
	std::stringstream targetDataFileName;
	targetDataFileName << pathData << "targetData.txt";
	VxReal targetMass = 3000.00380918; //readParameter<VxReal>("targetMass", targetDataFileName);
	VxReal targetSideLengthX = readParameter<VxReal>("targetSideLengthX", targetDataFileName);
	VxReal targetSideLengthY = readParameter<VxReal>("targetSideLengthY", targetDataFileName);
	VxReal targetSideLengthZ = readParameter<VxReal>("targetSideLengthZ", targetDataFileName);
	VxReal targetPosX = readParameter<VxReal>("targetIPosX", targetDataFileName);
	VxReal targetPosY = readParameter<VxReal>("targetIPosY", targetDataFileName);
	VxReal targetPosZ = readParameter<VxReal>("targetIPosZ", targetDataFileName);

	VxReal targetVelX = readParameter<VxReal>("targetIVelX", targetDataFileName);
	VxReal targetVelY = readParameter<VxReal>("targetIVelY", targetDataFileName);
	VxReal targetVelZ = readParameter<VxReal>("targetIVelZ", targetDataFileName);

	VxReal targetAngX = readParameter<VxReal>("targetIAngRateX", targetDataFileName);
	VxReal targetAngY = readParameter<VxReal>("targetIAngRateY", targetDataFileName);
	VxReal targetAngZ = readParameter<VxReal>("targetIAngRateZ", targetDataFileName);

	VxReal targetQuatS = readParameter<VxReal>("targetIQuatS", targetDataFileName);
	VxReal targetQuatX = readParameter<VxReal>("targetIQuatX", targetDataFileName);
	VxReal targetQuatY = readParameter<VxReal>("targetIQuatY", targetDataFileName);
	VxReal targetQuatZ = readParameter<VxReal>("targetIQuatZ", targetDataFileName);

	Vx::VxVector3 targetPos = Vx::VxVector3(targetPosX, targetPosY, targetPosZ);
	Vx::VxVector3 targetVel = Vx::VxVector3(targetVelX, targetVelY, targetVelZ);
	Vx::VxVector3 targetAngRate = Vx::VxVector3(targetAngX, targetAngY, targetAngZ);

	VxMath::Quaternion targetQuat = VxMath::Quaternion(targetQuatS, targetQuatX, targetQuatY, targetQuatZ);
	

	
	
    // Create a dynamic part that contains a box as collision geometry 
	(*tetheredTargetPartPtr)->parameterMassPropertiesContainer.mass = targetMass;
	VxSmartInterface<Box> box = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
	box->parameterDimension = VxVector3(targetSideLengthX, targetSideLengthY, targetSideLengthZ);
	VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Box::kFactoryKey); 
    cg = box;
	(*tetheredTargetPartPtr)->addCollisionGeometry(cg);
	(*tetheredTargetPartPtr)->autoComputeInertiaAndCOM();
//   	(*tetheredTargetPartPtr)->getVxPart()->setMassAndInertia(targetMass,targetInertia);


	//auto RA = VxMath::Transformation::createRotation(VxQuaternion(targetQuat[0], targetQuat[1], targetQuat[2], targetQuat[3]));
	//auto TA = VxMath::Transformation::createTranslation(VxVector3(targetPosX, targetPosY, targetPosZ));
    //targetPart->setControl(Vx::VxPart::kControlStatic);

	// can't do two local transforms at once, how to set orientation such that its scope is greater than this function???? LF
	// Attempting to use VxPart->setTransform() to solve this issue; LF NOPE

	VxTransform BB;
	BB.setRotation(targetQuat);
	BB.setTranslation(targetPos);
	VxMath::Matrix44 tmBB;
	BB.get(tmBB);
	(*tetheredTargetPartPtr)->setLocalTransform(tmBB); // ----> THIS WORKS
	//(*tetheredTargetPartPtr)->setLocalTransform(VxMath::Transformation::createRotation(targetQuat)); // <--- THIS WORKS NOT



	if (targetPoint == 1)
	{
		//(*tetheredTargetPartPtr)->getVxPart()->setOrientationQuaternion(targetQuat[0], targetQuat[1], targetQuat[2], targetQuat[3]); // center it with respect to net
	}
	else
	{
		/*VxEulerAngles::eEulerOrder order1 = VxEulerAngles::eEulerOrder(20);
		VxEulerAngles::eEulerOrder order = VxEulerAngles::getDefaultEulerOrder();
		(*tetheredTargetPartPtr)->getVxPart()->setOrientationEulerAngles(Eul, order1);*/
	}



	//(*tetheredTargetPartPtr)->getVxPart()->setOrientationEulerAngles(VxVector3(20,0,0),VxEulerAngles::getDefaultEulerOrder());
	(*tetheredTargetPartPtr)->getVxPart()->setAngularVelocity(targetAngRate); // center it with respect to net
	(*tetheredTargetPartPtr)->setLinearVelocity(targetVel); // center it with respect to net
		(*tetheredTargetPartPtr)->getVxPart()->setControl(Vx::VxPart::kControlDynamic);

    assembly->addPart(*tetheredTargetPartPtr);
	
	VxCollisionRule collisionRule1((*tetheredTargetPartPtr)->getVxPart(), false);
	assembly->getVxAssembly()->appendCollisionRule(collisionRule1);

	/*Vx::VxQuaternion TargetQ;
	(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);

	auto g = 1;*/

}

//VxReal targetJx = 15000;
//VxReal targetJy = 3000;
//VxReal targetJz = 15000;
//VxReal targetJxy = 0;
//VxReal targetJxz = 0;
//VxReal targetJyz = 0;
//VxReal33 targetInertia = { (targetJx,targetJxy,targetJxz),(targetJxy,targetJy,targetJyz),(targetJxz,targetJyz,targetJz) };