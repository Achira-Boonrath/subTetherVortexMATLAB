// Created by Eleonora Botta, June 23, 2017
// TO DO: choose whether to keep constraint based or force+constraint based

#include "MyLibraryNet/addStdClosingMechanism.h"
#include "MyLibraryNet/attachDistance.h"
#include "MyLibraryNet/readParameter.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>

#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// Create standard closing mechanism among the CMs
void addStdClosingMechanismCMs(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[])
{
	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);

	// add joints
	closingMechanismConstraintsVector[0] = attachDistance( assembly, cornerMasses[0], cornerMasses[1], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[1] = attachDistance( assembly, cornerMasses[1], cornerMasses[3], 0, 0.000001, 0.000001); 
	closingMechanismConstraintsVector[2] = attachDistance( assembly, cornerMasses[2], cornerMasses[3], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[3] = attachDistance( assembly, cornerMasses[2], cornerMasses[0], 0, 0.000001, 0.000001);
	for (int i=0; i<4; i++)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[i]->getProxy();
		d->inputEnable = false;
	}
}

// Create standard closing mechanism around the net perimeter
void addStdClosingMechanismNetPerimeter(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<Part> **partMatrix, VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[])
{
	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);

	// Find indexes for central part of net, and last part of the net
	int indexCenterPart, indexThirdPart, indexTwoThirdsPart; 
	if(NetSize%2 != 0) // NetSize is odd
	{
		indexCenterPart = (int)(NetSize-1)/2 ;
		indexThirdPart = (int)(NetSize-1)/3; 
		indexTwoThirdsPart = NetSize - (int)(NetSize-1)/3  ; // TODO: add (-1) to have symmetry. But not for validation against past results. 
	}
	else 
		std::cout << std::endl << "ERROR: NetSize in file netData has to be odd. Please exit simulation and correct its value. \n ";
	int indexLastPart = NetSize - 1;
	std::cout << "indexCenterPart = " << indexCenterPart << std::endl;
	std::cout << "indexLastart = " << indexLastPart << std::endl;
	std::cout << "indexThirdPart = " << indexThirdPart << std::endl;
	std::cout << "indexTwoThirdsPart = " << indexTwoThirdsPart << std::endl;

	// add joints
	closingMechanismConstraintsVector[0] = attachDistance( assembly, cornerMasses[0], partMatrix[indexThirdPart][0], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[1] = attachDistance( assembly, partMatrix[indexThirdPart][0], partMatrix[indexTwoThirdsPart][0], 0, 0.000001, 0.000001); 
	closingMechanismConstraintsVector[2] = attachDistance( assembly, partMatrix[indexTwoThirdsPart][0], cornerMasses[1], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[3] = attachDistance( assembly, cornerMasses[1], partMatrix[indexLastPart][indexThirdPart], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[4] = attachDistance( assembly, partMatrix[indexLastPart][indexThirdPart], partMatrix[indexLastPart][indexTwoThirdsPart], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[5] = attachDistance( assembly, partMatrix[indexLastPart][indexTwoThirdsPart], cornerMasses[3], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[6] = attachDistance( assembly, cornerMasses[3], partMatrix[indexTwoThirdsPart][indexLastPart], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[7] = attachDistance( assembly, partMatrix[indexTwoThirdsPart][indexLastPart], partMatrix[indexThirdPart][indexLastPart], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[8] = attachDistance( assembly, partMatrix[indexThirdPart][indexLastPart], cornerMasses[2], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[9] = attachDistance( assembly, cornerMasses[2], partMatrix[0][indexTwoThirdsPart], 0, 0.000001, 0.000001);
	closingMechanismConstraintsVector[10] = attachDistance( assembly, partMatrix[0][indexTwoThirdsPart], partMatrix[0][indexThirdPart], 0, 0.000001, 0.000001); 
	closingMechanismConstraintsVector[11] = attachDistance( assembly, partMatrix[0][indexThirdPart], cornerMasses[0], 0, 0.000001, 0.000001);
	
	for (int i=0; i<12; i++)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[i]->getProxy();
		d->inputEnable = false;
	}
}
	

void activateStdClosingMechanismCMs(int NetSize, VxReal NetSideLength, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[])
{
	// based on distancejoint constraints
	/*for (int i=0; i<4; i++)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[i]->getProxy();
		d->inputEnable = true;
	}*/
			
	// based on force
	VxReal timeForClosure = 5.0;
	VxReal cornerMassMass = cornerMasses[0]->getVxPart()->getMass();
	VxReal closingForce = 2* cornerMassMass * NetSideLength / pow(timeForClosure,2.0);
	
	VxVector3 pos0 = cornerMasses[0]->getVxPart()->getPosition();
	VxVector3 pos1 = cornerMasses[1]->getVxPart()->getPosition();
	VxVector3 fromP1toP0 = pos1 - pos0;
	VxReal distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[0]->getProxy();
		d->inputEnable = true;
	}
	else
	{
		//std::cout << "distance is " << distance << std::endl;
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		//std::cout << "force is " << force.x() << "," << force.y() << "," << force.z()  << std::endl;
		//VxVector3 force = VxVector3(50*fromP1toP0.x(), 50*fromP1toP0.y(), 50*fromP1toP0.z() );
		cornerMasses[0]->getVxPart()->addForce(force);
		cornerMasses[1]->getVxPart()->addForce(-force);
	}
				
	pos0 = cornerMasses[1]->getVxPart()->getPosition();
	pos1 = cornerMasses[3]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[1]->getProxy();
		d->inputEnable = true;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		//force = VxVector3(50*fromP1toP0.x(), 50*fromP1toP0.y(), 50*fromP1toP0.z() );
		cornerMasses[1]->getVxPart()->addForce(force);
		cornerMasses[3]->getVxPart()->addForce(-force);
	}

	pos0 = cornerMasses[3]->getVxPart()->getPosition();
	pos1 = cornerMasses[2]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[2]->getProxy();
		d->inputEnable = true;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		//force = VxVector3(50*fromP1toP0.x(), 50*fromP1toP0.y(), 50*fromP1toP0.z() );
		cornerMasses[3]->getVxPart()->addForce(force);
		cornerMasses[2]->getVxPart()->addForce(-force);
	}

	pos0 = cornerMasses[2]->getVxPart()->getPosition();
	pos1 = cornerMasses[0]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[3]->getProxy();
		d->inputEnable = true;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		//force = VxVector3(50*fromP1toP0.x(), 50*fromP1toP0.y(), 50*fromP1toP0.z() );
		cornerMasses[2]->getVxPart()->addForce(force);
		cornerMasses[0]->getVxPart()->addForce(-force);
	}

}


void activateStdClosingMechanismNetPerimeter(int NetSize, VxReal NetSideLength, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<Part> **partMatrix, VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[])
{
	/*
	// based on distancejoint constraints
	for (int i=0; i<8; i++)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[i]->getProxy();
		d->inputEnable = true;
	}
	*/
	
	// based on force
	VxReal timeForClosure = 1.0;
	VxReal cornerMassMass = cornerMasses[0]->getVxPart()->getMass();
	VxReal closingForce = 2* cornerMassMass * NetSideLength / pow(timeForClosure,2.0);

	int indexCenterPart, indexThirdPart, indexTwoThirdsPart; 
	if(NetSize%2 != 0) // NetSize is odd
	{
		indexCenterPart = (int)(NetSize-1)/2 ;
		indexThirdPart = (int)(NetSize-1)/3; 
		indexTwoThirdsPart = NetSize - (int)(NetSize-1)/3; 
	}
	else 
		std::cout << std::endl << "ERROR: NetSize in file netData has to be odd. Please exit simulation and correct its value. \n ";
	int indexLastPart = NetSize - 1;

	VxVector3 pos0 = cornerMasses[0]->getVxPart()->getPosition();
	VxVector3 pos1 = partMatrix[indexThirdPart][0]->getVxPart()->getPosition();
	VxVector3 fromP1toP0 = pos1 - pos0;
	VxReal distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[0]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		cornerMasses[0]->getVxPart()->addForce(force);
		partMatrix[indexThirdPart][0]->getVxPart()->addForce(-force);
	}

	pos0 = partMatrix[indexThirdPart][0]->getVxPart()->getPosition();
	pos1 = partMatrix[indexTwoThirdsPart][0]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[1]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[indexThirdPart][0]->getVxPart()->addForce(force);
		partMatrix[indexTwoThirdsPart][0]->getVxPart()->addForce(-force);
	}

	pos0 = partMatrix[indexTwoThirdsPart][0]->getVxPart()->getPosition();
	pos1 = cornerMasses[1]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[2]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[indexTwoThirdsPart][0]->getVxPart()->addForce(force);
		cornerMasses[1]->getVxPart()->addForce(-force);
	}

	pos0 = cornerMasses[1]->getVxPart()->getPosition();
	pos1 = partMatrix[indexLastPart][indexThirdPart]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[3]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		cornerMasses[1]->getVxPart()->addForce(force);
		partMatrix[indexLastPart][indexThirdPart]->getVxPart()->addForce(-force);
	}
	
	pos0 = partMatrix[indexLastPart][indexThirdPart]->getVxPart()->getPosition();
	pos1 = partMatrix[indexLastPart][indexTwoThirdsPart]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[4]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[indexLastPart][indexThirdPart]->getVxPart()->addForce(force);
		partMatrix[indexLastPart][indexTwoThirdsPart]->getVxPart()->addForce(-force);
	}

	pos0 = partMatrix[indexLastPart][indexTwoThirdsPart]->getVxPart()->getPosition();
	pos1 = cornerMasses[3]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[5]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[indexLastPart][indexTwoThirdsPart]->getVxPart()->addForce(force);
		cornerMasses[3]->getVxPart()->addForce(-force);
	}

	pos0 = cornerMasses[3]->getVxPart()->getPosition();
	pos1 = partMatrix[indexTwoThirdsPart][indexLastPart]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[6]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		cornerMasses[3]->getVxPart()->addForce(force);
		partMatrix[indexTwoThirdsPart][indexLastPart]->getVxPart()->addForce(-force);
	}

	pos0 = partMatrix[indexTwoThirdsPart][indexLastPart]->getVxPart()->getPosition();
	pos1 = partMatrix[indexThirdPart][indexLastPart]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[7]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[indexTwoThirdsPart][indexLastPart]->getVxPart()->addForce(force);
		partMatrix[indexThirdPart][indexLastPart]->getVxPart()->addForce(-force);
	}

	pos0 = partMatrix[indexThirdPart][indexLastPart]->getVxPart()->getPosition();
	pos1 = cornerMasses[2]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[8]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[indexThirdPart][indexLastPart]->getVxPart()->addForce(force);
		cornerMasses[2]->getVxPart()->addForce(-force);
	}

	pos0 = cornerMasses[2]->getVxPart()->getPosition();
	pos1 = partMatrix[0][indexTwoThirdsPart]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[9]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		cornerMasses[2]->getVxPart()->addForce(force);
		partMatrix[0][indexTwoThirdsPart]->getVxPart()->addForce(-force);
	}
	
	pos0 = partMatrix[0][indexTwoThirdsPart]->getVxPart()->getPosition();
	pos1 = partMatrix[0][indexThirdPart]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[10]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[0][indexTwoThirdsPart]->getVxPart()->addForce(force);
		partMatrix[0][indexThirdPart]->getVxPart()->addForce(-force);
	}
	
	pos0 = partMatrix[0][indexThirdPart]->getVxPart()->getPosition();
	pos1 = cornerMasses[0]->getVxPart()->getPosition();
	fromP1toP0 = pos1 - pos0;
	distance = fromP1toP0.normalize();
	if (distance<1.0)
	{
		VxSmartInterface<DistanceJoint> d = closingMechanismConstraintsVector[11]->getProxy();
		d->inputEnable = true;
		// std::cout << "distance joint enabled" << std::endl;
	}
	else
	{
		VxVector3 force = VxVector3(closingForce*fromP1toP0.x(), closingForce*fromP1toP0.y(), closingForce*fromP1toP0.z() );
		partMatrix[0][indexThirdPart]->getVxPart()->addForce(force);
		cornerMasses[0]->getVxPart()->addForce(-force);
	}
	
}