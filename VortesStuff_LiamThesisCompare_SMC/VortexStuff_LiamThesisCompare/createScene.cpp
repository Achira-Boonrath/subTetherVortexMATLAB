// Created by Eleonora Botta, September 8, 2016
// Modified by Eleonora Botta, September 15, 2016: chaser 
// Modified by Eleonora Botta, September 19, 2016: tether, winch, disabled collisions between chaser and winch
// Modified by Eleonora Botta, September 23, 2016: chaser passed as pointer to createScene for main to know it at runtime
// Modified by Eleonora Botta, September 25, 2016: pass winchPart to addTether. 
// Modified by Eleonora Botta, September 28, 2016: winch passed as pointer to createScene for main to know it at runtime
// Modified by Eleonora Botta, September 29, 2016: hinge passed as pointer to createScene for main to know it at runtime
// Modified by Eleonora Botta, October 4, 2016: save mechanism to be opened in Editor
// Modified by Eleonora Botta, October 14, 2016: modify material table with fixed values
// Modified by Eleonora Botta, October 31, 2016: contact materials computed automatically
// Modified by Eleonora Botta, January 17, 2017: read the number of tethers present and add multiple tethers
// Modified by Eleonora Botta, February 16, 2017: possibility to use different configuration for 3 rings
// Modified by Eleonora Botta, February 17, 2017: possibility to have 2 tethers configuration.
// Modified by Eleonora Botta, February 27, 2017: pass mechanism to createNet, so that I can have cables for threads. 
// Modified by Eleonora Botta, April 17, 2017:	added #include <Vx/VxCollisionRule.h>, #include <Vx/VxAssembly.h>  for Vortex Studio 2017a
// Modified by Liam Field, 3/24/22: Modified to work within the chaser-tether-target simulation
// Modified by Liam Field, 3/24/22: variation in choice of tethered connection -> distance joint or cable

// Create scene with parts linked with various constraints.

#include "MyLibraryNet/createScene.h"

#include "MyLibraryNet/createNet.h"
#include "MyLibraryNet/addCornerMasses.h"
#include "MyLibraryNet/addChaser.h"
#include "MyLibraryNet/attachDistanceN.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/addTetherToCT.h"
#include "MyLibraryNet/addWinch.h"
#include "MyLibraryNet/modifyMaterialTable.h"
#include "MyLibraryNet/addStdClosingMechanism.h"
#include "MyLibraryNet/addTetheredTarget.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxSim/VxSmartInterface.h>
#include <VxContent/Scene.h>
#include <VxContent/SceneICD.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/MechanismICD.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <VxDynamics/Hinge.h>
#include <VxSim/VxObjectSerializer.h>
#include <Vx/VxMaterialTable.h>

// for Vortex Studio 2017a
#include <Vx/VxPart.h>
#include <Vx/VxCollisionRule.h>
#include <Vx/VxAssembly.h>

#include <iostream>
#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxContent;
using namespace VxSim;

VxSim::VxSmartInterface<VxContent::Scene>  createScene(std::string pathData, VxSmartInterface<Part>* partMatrix, VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* tetheredTargetPartPtr, const int numNode, int TetherType, VxSmartInterface<DistanceJoint> allConstraintsVector[])
//VxSim::VxSmartInterface<VxContent::Scene>  createScene(std::string pathData, VxSmartInterface<Part>* partMatrix, VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* tetheredTargetPartPtr, const int numNode, int TetherType, VxSmartInterface<DistanceJoint> allConstraintsVector[])
{
	const double PI = 3.141592653589793238463;
	
	VxSmartInterface<Scene>  scene = VxSim::VxExtensionFactory::create(SceneICD::kFactoryKey);
    VxSmartInterface<Assembly>  assembly = VxSim::VxExtensionFactory::create(VxDynamics::Assembly::kFactoryKey);
	VxSmartInterface<Mechanism> mechanism = VxSim::VxExtensionFactory::create(MechanismICD::kFactoryKey);
    
	FILE* FileLog=0;
	//FileLog = fopen("Deployment_NoTether/Test_1/FileLog.txt", "w");





	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	VxReal youngModulus = readParameter<VxReal>("youngModulus", tetherDataFileName); // Pa
	VxReal tetherLength = readParameter<VxReal>("natLength", tetherDataFileName);// m
	VxReal tetherArea = readParameter<VxReal>("area", tetherDataFileName); // m
	VxReal tetherDamping = 16;
	//VxReal Test = youngModulus * tetherArea;
	VxReal segmentLength = readParameter<VxReal>("segNatLength", tetherDataFileName);
	VxReal segmentStiffness = readParameter<VxReal>("segStiffness", tetherDataFileName);
	VxReal segmentDamping = readParameter<VxReal>("segDamping", tetherDataFileName);
	VxReal partMass = readParameter<VxReal>("segMass", tetherDataFileName);
	VxReal tetherRadius = readParameter<VxReal>("tetherRadius", tetherDataFileName);
	VxReal tetherDensity = readParameter<VxReal>("tetherDensity", tetherDataFileName);
	VxReal tetherYoungModulus = readParameter<VxReal>("tetherYoungModulus", tetherDataFileName);
	
	
	
	// Create chaser
	VxSmartInterface<Part> chaserPart = (*chaserPartPtr); // defined locally, to access part
	addChaser(assembly, pathData, chaserPartPtr);

	

	// Create target
	VxSmartInterface<Part> tetheredTargetPart = (*tetheredTargetPartPtr); // defined locally, to access part
	addTetheredTarget(assembly, pathData, tetheredTargetPartPtr);

	// Dont know what to do here, therefore no collision for now
	// Enable all collisions between objects in assembly

		VxCollisionRule collisionRule(assembly->getVxAssembly(), false);
		assembly->getVxAssembly()->appendCollisionRule(collisionRule);
	
	
	//	VxCollisionRule collisionRule(assembly->getVxAssembly(), true);
	//   assembly->getVxAssembly()->appendCollisionRule(collisionRule);
		
		//modifying orientations here because I don't understand why its scope is only the add"" functions, LF
		//THIS DOESN'T WORK
		std::stringstream chaserIniDataFileName;
		chaserIniDataFileName << pathData << "chaserData.txt";
		VxReal chaserQuatS = readParameter<VxReal>("chaserIQuatS", chaserIniDataFileName);
		VxReal chaserQuatX = readParameter<VxReal>("chaserIQuatX", chaserIniDataFileName);
		VxReal chaserQuatY = readParameter<VxReal>("chaserIQuatY", chaserIniDataFileName);
		VxReal chaserQuatZ = readParameter<VxReal>("chaserIQuatZ", chaserIniDataFileName);
		chaserPart->getVxPart()->setOrientationQuaternion(chaserQuatS, chaserQuatX, chaserQuatY, chaserQuatZ);

		std::stringstream targetIniDataFileName;
		targetIniDataFileName << pathData << "targetData.txt";
		VxReal targetQuatS = readParameter<VxReal>("targetIQuatS", targetIniDataFileName);
		VxReal targetQuatX = readParameter<VxReal>("targetIQuatX", targetIniDataFileName);
		VxReal targetQuatY = readParameter<VxReal>("targetIQuatY", targetIniDataFileName);
		VxReal targetQuatZ = readParameter<VxReal>("targetIQuatZ", targetIniDataFileName);
		tetheredTargetPart->getVxPart()->setOrientationQuaternion(targetQuatS, targetQuatX, targetQuatY, targetQuatZ);

		
		std::stringstream netDataFileName;
		netDataFileName << pathData << "netData.txt";
		VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
		
	
    
	// Put the assembly in the mechanism.
	mechanism->addAssembly(assembly);

	// Create the tether and put it in the mechanism.
	if (TetherType == 1)
	{   
		// read data for chaser
		std::stringstream chaserDataFileName;
		chaserDataFileName << pathData << "chaserData.txt";
		VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
		
		if (numNode > 0)
		{//////////
		// Building code for 'n' tether node
			attachDistanceN(assembly,chaserPart,tetheredTargetPart,partMatrix,segmentLength,segmentStiffness,segmentDamping, numNode, partMass);
		//////////
		}
		else
		{

			// Implement distance joint as tether
		    segmentLength = tetherLength / (numNode + 1);
			segmentStiffness = tetherYoungModulus * PI * pow(tetherRadius, 2.00) / segmentLength;

			VxReal partMass = tetherDensity * PI * pow(tetherRadius, 2.0) * segmentLength;	// part mass of individual nodes along main tether
			//VxReal segmentDamping = segmentStiffness/10; // matching the implementation of the cable axial damping;
			VxReal segmentDamping = (2 * csiAxial * segmentStiffness) / sqrt(segmentStiffness / partMass);

			VxSmartInterface<DistanceJoint> d = VxExtensionFactory::create(DistanceJoint::kFactoryKey);
			d->inputAttachment1.part = chaserPart;
			//d->inputAttachment1.position = VxVector3(chaserSideLength / 2, 0, 0);

			//AB, target dim
			//targetSideLengthX	1.2500000000000000	double
			//targetSideLengthY	1.7500000000000000	double
			//targetSideLengthZ	1.2500000000000000	double


			d->inputAttachment2.part = tetheredTargetPart;
			d->inputAttachment2.position = VxVector3(0, -1.75 / 2, 0);
			//d->inputAttachment2.position = VxVector3(0, -1.75 / 2, -1.25 / 2); //AB, test
			d->inputDistance = 30; // keep distance smaller than Natural length?

			d->inputDistanceEquation.relaxation.stiffness = segmentStiffness/3;
			d->inputDistanceEquation.relaxation.damping = segmentDamping;
			d->inputDistanceEquation.relaxation.enable = true; // this relaxes the constraint equation and reduces computational time.
			assembly->addConstraint(d);
			mechanism->addAssembly(assembly);
			allConstraintsVector[0] = d;

		}
	}
	else if (TetherType == 2)
	{
		// read data for chaser

		// Implement distance joint as tether
		segmentLength = tetherLength / (numNode + 1);
		segmentStiffness = tetherYoungModulus * PI * pow(tetherRadius/1.0, 2.00) / segmentLength;
		segmentDamping = segmentStiffness / 10.0;

		std::stringstream chaserDataFileName;
		chaserDataFileName << pathData << "chaserData.txt";
		VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);

		// Building code for 'n' tether node
		attachDistanceN_ST(assembly, chaserPart, tetheredTargetPart, partMatrix, segmentLength, segmentStiffness, segmentDamping, numNode, partMass);
		//////////


	}
	else 
	{   addTetherToCT(pathData, mechanism, chaserPart, partMatrix, tetheredTargetPartPtr);
		mechanism->addAssembly(assembly);
	

	}
	//VxSim::VxSmartInterface<DistanceJoint> v = assembly->getConstraints()[0];
	// Vx::VxArray<Vx::ConstraintPartAttachment*> v1 = assembly->getConstraints()[0]->getVxConstraint()->getPartAttachments();
	//VxVector3 pos = v->outputAttachment1.positionRel;
	// Add the mechanism to the scene.
	scene->addMechanism(mechanism);

	
	/*Vx::VxQuaternion TargetQ;
	(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
	(*tetheredTargetPartPtr)->getVxPart()->setOrientationQuaternion(VxQuaternion(targetQuatS, targetQuatX, targetQuatY, targetQuatZ));
	Vx::VxQuaternion TargetQ1;
	tetheredTargetPart->getVxPart()->getOrientationQuaternion(TargetQ1);
	auto g = 1;*/
	// Change gravity

    VxFrame::instance()->getUniverse(0)->setGravity(0,0,0); // Set to zero, gravity is added later.
	
	//fclose(FileLog);
	std::cout << "returning scene"<< std::endl;
    return scene;
}