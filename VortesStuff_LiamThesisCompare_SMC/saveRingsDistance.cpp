// Created by Eleonora Botta, February 16, 2017
// Compute distance among rings, to have a measure of capture success.

#include "MyLibraryNet/saveRingsDistance.h"

#include <iostream>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Hinge.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxHinge.h>
#include <CableSystems/DynamicsICD.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


float saveRingsDistance3Rings(VxSmartInterface<Part> **partMatrix, int indexLastPart, int indexCenterPart, FILE* outRingsDistance)
{
	float currentRingsDistance = 0;

	// define indexes for rings
	int usedRingPartX[9]; 
	int usedRingPartY[9]; 

	usedRingPartX[0] = indexLastPart;
	usedRingPartY[0] = indexLastPart;
	
	usedRingPartX[1] = indexLastPart;
	usedRingPartY[1] = indexCenterPart;
	
	usedRingPartX[2] = indexLastPart;
	usedRingPartY[2] = 0;

	usedRingPartX[3] = indexCenterPart;
	usedRingPartY[3] = 0;

	usedRingPartX[4] = 0;
	usedRingPartY[4] = 0;

	usedRingPartX[5] = 0;
	usedRingPartY[5] = indexCenterPart;

	usedRingPartX[6] = 0;
	usedRingPartY[6] = indexLastPart;

	usedRingPartX[7] = indexCenterPart;
	usedRingPartY[7] = indexLastPart;

	usedRingPartX[8] = indexLastPart;
	usedRingPartY[8] = indexLastPart;
	
	// find overall distance among rings
	for (int i=1; i<9; i++)
	{
		VxVector3 pos2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getPosition();
		VxVector3 pos1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getPosition();
		
		VxVector3 relPos = pos2 - pos1;
		VxReal distance = relPos.normalize(); //make into unit vector and return original length
		currentRingsDistance = currentRingsDistance + distance; 
	}
		
	fprintf( outRingsDistance, "%f \n", currentRingsDistance );

	return currentRingsDistance;
}


float saveRingsDistance2Tethers(VxSmartInterface<Part> **partMatrix, int indexLastPart, int indexCenterPart, FILE* outRingsDistance)
{
	float currentRingsDistance = 0;

	// define indexes for rings
	int usedRingPartX[7]; 
	int usedRingPartY[7]; 

	usedRingPartX[0] = indexLastPart;
	usedRingPartY[0] = indexLastPart;
	
	usedRingPartX[1] = indexLastPart;
	usedRingPartY[1] = indexCenterPart;
	
	usedRingPartX[2] = indexLastPart;
	usedRingPartY[2] = 0;

	usedRingPartX[3] = 0;
	usedRingPartY[3] = 0;

	usedRingPartX[4] = 0;
	usedRingPartY[4] = indexCenterPart;

	usedRingPartX[5] = 0;
	usedRingPartY[5] = indexLastPart;

	usedRingPartX[6] = indexLastPart;
	usedRingPartY[6] = indexLastPart;
	
	// find overall distance among rings
	for (int i=1; i<7; i++)
	{
		VxVector3 pos2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getPosition();
		VxVector3 pos1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getPosition();
		
		VxVector3 relPos = pos2 - pos1;
		VxReal distance = relPos.normalize(); //make into unit vector and return original length
		currentRingsDistance = currentRingsDistance + distance; 
	}
		
	fprintf( outRingsDistance, "%f \n", currentRingsDistance );

	return currentRingsDistance;
}
