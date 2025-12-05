// Created by Eleonora Botta, February 13, 2017
// Control winch rate with a PD control based on current tether length among rings
// Modified by Eleonora Botta, February 15, 2017: use 4 tethers as metrics for control
// Modified by Eleonora Botta, February 17, 2017: possibility to have 2 tethers configuration. Needs a different allowable length, and has different ring parts

#include "MyLibraryNet/controlWinchRatePD.h"

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


//float controlWinchRatePD(VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxSmartInterface<IExtension> cableExtension)
float controlWinchRatePD(VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension )
{
	VxReal currentRingsDistance = 0;
	VxReal currentRingsRelVelocity = 0;

	//VxReal cableMaxTension = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kMaxTensionID)->toReal();
	

	// read position of parts with rings and sum them to get current distance 
	for (int i=2; i<ringsNb; i++)
	{
		// proportional part
		VxVector3 pos2 = partMatrix[ringPartX[i]][ringPartY[i]]->getVxPart()->getPosition();
		VxVector3 pos1 = partMatrix[ringPartX[i-1]][ringPartY[i-1]]->getVxPart()->getPosition();
	
		VxVector3 relPos = pos2 - pos1;
		VxReal distance = relPos.normalize(); //make into unit vector and return original length

		currentRingsDistance = currentRingsDistance + distance; 

		// derivative part
		VxVector3 vel2 = partMatrix[ringPartX[i]][ringPartY[i]]->getVxPart()->getLinearVelocity();
		VxVector3 vel1 = partMatrix[ringPartX[i-1]][ringPartY[i-1]]->getVxPart()->getLinearVelocity();

		VxVector3 relVel = vel2 - vel1;
	
		VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
		currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
	}

	// add last segment distance, from last ring to attachment point

	// proportional part
	VxVector3 pos2 = partMatrix[ringPartX[ringsNb-1]][ringPartY[ringsNb-1]]->getVxPart()->getPosition();
	VxVector3 pos1 = partMatrix[indexLastPart][0]->getVxPart()->getPosition();
	VxVector3 relPos = pos2 - pos1;
	VxReal distance = relPos.normalize(); //make into unit vector and return original length
	currentRingsDistance = currentRingsDistance + distance; 

	// derivative part
	VxVector3 vel2 = partMatrix[ringPartX[ringsNb-1]][ringPartY[ringsNb-1]]->getVxPart()->getLinearVelocity();
	VxVector3 vel1 = partMatrix[indexLastPart][0]->getVxPart()->getLinearVelocity();
	VxVector3 relVel = vel2 - vel1;
	VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
	currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;

	// apply control proportional to the current distance: winch in faster when distance is larger
	VxReal errorRingsDistance = currentRingsDistance - desiredRingsDistance;
	VxReal errorRingsVelocity = currentRingsRelVelocity;
	
	VxReal tetherLengthRateControl;
	if ( (errorRingsDistance > 0) && (cableMaxTension < 1.0) )
		  tetherLengthRateControl = proportionalGain * errorRingsDistance + derivativeGain * errorRingsVelocity;
	else
		 tetherLengthRateControl = 0.0;
		 //tetherLengthRateControl = -proportionalGain*10.0* (cableMaxTension - 2.0);

	//std::cout << "current length rate: " << tetherLengthRateControl << std::endl;
	float currentWinchSpoolingVelocity = tetherLengthRateControl;

	// transform tether length rate control to angular control 
	//float winchAngularVelocityControl = tetherLengthRateControl / winchRadius; // (rad/s)
	//(*hingeForWinchPtr)->getVxConstraint()->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, winchAngularVelocityControl);
	
	//std::cout << "errorRingsDistance: " << errorRingsDistance << std::endl;
	//std::cout << "errorRingsVelocity: " << errorRingsVelocity << std::endl;
	//std::cout << "current length rate: " << tetherLengthRateControl << std::endl;

	return currentWinchSpoolingVelocity;
}


float controlWinchRatePDon4Tethers(VxReal minAllowableLength, VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, int indexThirdPart, int indexTwoThirdsPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension, VxReal cableMinLength )
{
	float minCurrentWinchSpoolingVelocity = 1000.0;

	VxReal currentRingsDistance = 0;
	VxReal currentRingsRelVelocity = 0;

	int usedRingPartX[4]; 
	int usedRingPartY[4]; 

	// read position of parts with rings and sum them to get current distance and rel velocity
	for (int tetherCounter=0; tetherCounter<tethersNb; tetherCounter++)
	{
		if (tetherCounter == 0)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = ringPartX[1];
			usedRingPartX[2] = ringPartX[2];
			usedRingPartX[3] = ringPartX[3];
			
			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = ringPartY[1];
			usedRingPartY[2] = ringPartY[2];
			usedRingPartY[3] = ringPartY[3];
		}
		else if (tetherCounter == 1)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = indexLastPart;
			usedRingPartX[2] = indexTwoThirdsPart;
			usedRingPartX[3] = indexThirdPart;
			//*usedRingPartX= { ringPartX[0], indexLastPart, indexTwoThirdsPart, indexThirdPart };
			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = 0;
			usedRingPartY[2] = 0;
			usedRingPartY[3] = 0;
			//ringPartY = { ringPartY[0], 0, 0, 0 };
		}
		else if (tetherCounter == 2)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = 0;
			usedRingPartX[2] = 0;
			usedRingPartX[3] = 0;
			//ringPartX = { ringPartX[0], 0, 0, 0 };
			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = 0;
			usedRingPartY[2] = indexThirdPart;
			usedRingPartY[3] = indexTwoThirdsPart;
			//ringPartY = { ringPartY[0], 0, indexThirdPart, indexTwoThirdsPart };
		}
		if (tetherCounter == 3)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = 0;
			usedRingPartX[2] = indexThirdPart;
			usedRingPartX[3] = indexTwoThirdsPart;
			//ringPartX = { ringPartX[0], 0, indexThirdPart, indexTwoThirdsPart };
			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = indexLastPart;
			usedRingPartY[2] = indexLastPart;
			usedRingPartY[3] = indexLastPart;
			//ringPartY = { ringPartY[0], indexLastPart, indexLastPart, indexLastPart };
		}

		for (int i=2; i<ringsNb; i++)
		{
			VxVector3 pos2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getPosition();
			VxVector3 pos1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getPosition();
			VxVector3 vel2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getLinearVelocity();
			VxVector3 vel1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getLinearVelocity();

			VxVector3 relPos = pos2 - pos1;
			VxReal distance = relPos.normalize(); //make into unit vector and return original length
			currentRingsDistance = currentRingsDistance + distance; 

			VxVector3 relVel = vel2 - vel1;
			VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
			currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
		}
		
		// add last segment distance, from last ring to attachment point
		int attachmentPartX, attachmentPartY;
		if (tetherCounter == 0)
		{
			 attachmentPartX = indexLastPart;
			 attachmentPartY = 0;
		}
		else if (tetherCounter == 1)
		{
			 attachmentPartX = 0;
			 attachmentPartY = 0;
		}
		else if (tetherCounter == 2)
		{
			 attachmentPartX = 0;
			 attachmentPartY = indexLastPart;
		}
		else if (tetherCounter == 3)
		{
			 attachmentPartX = indexLastPart;
			 attachmentPartY = indexLastPart;
		}

		VxVector3 pos2 = partMatrix[usedRingPartX[ringsNb-1]][usedRingPartY[ringsNb-1]]->getVxPart()->getPosition();
		VxVector3 pos1 = partMatrix[attachmentPartX][attachmentPartY]->getVxPart()->getPosition();
		VxVector3 vel2 = partMatrix[usedRingPartX[ringsNb-1]][usedRingPartY[ringsNb-1]]->getVxPart()->getLinearVelocity();
		VxVector3 vel1 = partMatrix[attachmentPartX][attachmentPartY]->getVxPart()->getLinearVelocity();
	
		VxVector3 relPos = pos2 - pos1;
		VxReal distance = relPos.normalize(); //make into unit vector and return original length
		currentRingsDistance = currentRingsDistance + distance; 

		VxVector3 relVel = vel2 - vel1;
		VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
		currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
		

		// apply control proportional to the current distance: winch in faster when distance is larger
		VxReal errorRingsDistance = currentRingsDistance - desiredRingsDistance;
		VxReal errorRingsVelocity = currentRingsRelVelocity;

		VxReal tetherLengthRateControl;
		if ( (errorRingsDistance > 0) && (cableMaxTension < maxAllowableTension) && (cableMinLength > minAllowableLength) )
			tetherLengthRateControl = proportionalGain * errorRingsDistance + derivativeGain * errorRingsVelocity;
		else
			 tetherLengthRateControl = 0.0;
			//tetherLengthRateControl = -proportionalGain*10.0* (cableMaxTension - 2.0);

		//std::cout << "current length rate: " << tetherLengthRateControl << std::endl;
		float currentWinchSpoolingVelocity = tetherLengthRateControl;

		if (currentWinchSpoolingVelocity < minCurrentWinchSpoolingVelocity)
			minCurrentWinchSpoolingVelocity = currentWinchSpoolingVelocity;
	}

	
	return minCurrentWinchSpoolingVelocity;
}

float controlWinchRatePDon4TethersWith3Rings(VxReal minAllowableLength, VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension, VxReal cableMinLength )
{
	float minCurrentWinchSpoolingVelocity = 1000.0;

	VxReal currentRingsDistance = 0;
	VxReal currentRingsRelVelocity = 0;

	int usedRingPartX[3]; 
	int usedRingPartY[3]; 

	// read position of parts with rings and sum them to get current distance and rel velocity
	for (int tetherCounter=0; tetherCounter<tethersNb; tetherCounter++)
	{
		if (tetherCounter == 0)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = ringPartX[1];
			usedRingPartX[2] = ringPartX[2];
			
			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = ringPartY[1];
			usedRingPartY[2] = ringPartY[2];
		}
		else if (tetherCounter == 1)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = indexLastPart;
			usedRingPartX[2] = indexCenterPart;
		
			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = 0;
			usedRingPartY[2] = 0;
		}
		else if (tetherCounter == 2)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = 0;
			usedRingPartX[2] = 0;

			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = 0;
			usedRingPartY[2] = indexCenterPart;
		}
		if (tetherCounter == 3)
		{
			usedRingPartX[0] = ringPartX[0];
			usedRingPartX[1] = 0;
			usedRingPartX[2] = indexCenterPart;

			usedRingPartY[0] = ringPartY[0];
			usedRingPartY[1] = indexLastPart;
			usedRingPartY[2] = indexLastPart;
		}

		for (int i=2; i<ringsNb; i++)
		{
			VxVector3 pos2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getPosition();
			VxVector3 pos1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getPosition();
			VxVector3 vel2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getLinearVelocity();
			VxVector3 vel1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getLinearVelocity();

			VxVector3 relPos = pos2 - pos1;
			VxReal distance = relPos.normalize(); //make into unit vector and return original length
			currentRingsDistance = currentRingsDistance + distance; 

			VxVector3 relVel = vel2 - vel1;
			VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
			currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
		}
		
		// add last segment distance, from last ring to attachment point
		int attachmentPartX, attachmentPartY;
		if (tetherCounter == 0)
		{
			 attachmentPartX = indexLastPart;
			 attachmentPartY = 0;
		}
		else if (tetherCounter == 1)
		{
			 attachmentPartX = 0;
			 attachmentPartY = 0;
		}
		else if (tetherCounter == 2)
		{
			 attachmentPartX = 0;
			 attachmentPartY = indexLastPart;
		}
		else if (tetherCounter == 3)
		{
			 attachmentPartX = indexLastPart;
			 attachmentPartY = indexLastPart;
		}

		VxVector3 pos2 = partMatrix[usedRingPartX[ringsNb-1]][usedRingPartY[ringsNb-1]]->getVxPart()->getPosition();
		VxVector3 pos1 = partMatrix[attachmentPartX][attachmentPartY]->getVxPart()->getPosition();
		VxVector3 vel2 = partMatrix[usedRingPartX[ringsNb-1]][usedRingPartY[ringsNb-1]]->getVxPart()->getLinearVelocity();
		VxVector3 vel1 = partMatrix[attachmentPartX][attachmentPartY]->getVxPart()->getLinearVelocity();
	
		VxVector3 relPos = pos2 - pos1;
		VxReal distance = relPos.normalize(); //make into unit vector and return original length
		currentRingsDistance = currentRingsDistance + distance; 

		VxVector3 relVel = vel2 - vel1;
		VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
		currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
		

		// apply control proportional to the current distance: winch in faster when distance is larger
		VxReal errorRingsDistance = currentRingsDistance - desiredRingsDistance;
		VxReal errorRingsVelocity = currentRingsRelVelocity;

		VxReal tetherLengthRateControl;
		if ( (errorRingsDistance > 0) && (cableMaxTension < maxAllowableTension) && (cableMinLength > minAllowableLength) )
			tetherLengthRateControl = proportionalGain * errorRingsDistance + derivativeGain * errorRingsVelocity;
		else
			 tetherLengthRateControl = 0.0;
			//tetherLengthRateControl = -proportionalGain*10.0* (cableMaxTension - 2.0);

		//std::cout << "current length rate: " << tetherLengthRateControl << std::endl;
		float currentWinchSpoolingVelocity = tetherLengthRateControl;

		if (currentWinchSpoolingVelocity < minCurrentWinchSpoolingVelocity)
			minCurrentWinchSpoolingVelocity = currentWinchSpoolingVelocity;
	}

	
	return minCurrentWinchSpoolingVelocity;
}


float controlWinchRatePDon2Tethers(VxReal minAllowableLength, VxReal maxAllowableTension, VxReal tethersNb, int indexCenterPart, VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr, VxReal cableMaxTension, VxReal cableMinLength )
{
	float minCurrentWinchSpoolingVelocity = 1000.0;

	VxReal currentRingsDistance = 0;
	VxReal currentRingsRelVelocity = 0;

	int usedRingPartX[4]; 
	int usedRingPartY[4]; 

	// read position of parts with rings and sum them to get current distance and rel velocity
	for (int tetherCounter=0; tetherCounter<tethersNb; tetherCounter++)
	{
		if (tetherCounter == 0)
		{
			usedRingPartX[0] = indexCenterPart;
			usedRingPartX[1] = indexLastPart;
			usedRingPartX[2] = indexLastPart;
			usedRingPartX[3] = 0;
			
			usedRingPartY[0] = indexCenterPart;
			usedRingPartY[1] = indexCenterPart;
			usedRingPartY[2] = 0;
			usedRingPartY[3] = 0;
		}
		else if (tetherCounter == 1)
		{
			usedRingPartX[0] = indexCenterPart;
			usedRingPartX[1] = 0;
			usedRingPartX[2] = 0;
			usedRingPartX[3] = indexLastPart;
			
			usedRingPartY[0] = indexCenterPart;
			usedRingPartY[1] = indexCenterPart;
			usedRingPartY[2] = indexLastPart;
			usedRingPartY[3] = indexLastPart;
		}

		for (int i=2; i<ringsNb; i++)
		{
			VxVector3 pos2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getPosition();
			VxVector3 pos1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getPosition();
			VxVector3 vel2 = partMatrix[usedRingPartX[i]][usedRingPartY[i]]->getVxPart()->getLinearVelocity();
			VxVector3 vel1 = partMatrix[usedRingPartX[i-1]][usedRingPartY[i-1]]->getVxPart()->getLinearVelocity();

			VxVector3 relPos = pos2 - pos1;
			VxReal distance = relPos.normalize(); //make into unit vector and return original length
			currentRingsDistance = currentRingsDistance + distance; 

			VxVector3 relVel = vel2 - vel1;
			VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
			currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
		}
		
		// add last segment distance, from last ring to attachment point
		int attachmentPartX, attachmentPartY;
		if (tetherCounter == 0)
		{
			 attachmentPartX = indexLastPart;
			 attachmentPartY = 0;
		}
		else if (tetherCounter == 1)
		{
			 attachmentPartX = 0;
			 attachmentPartY = 0;
		}
		else if (tetherCounter == 2)
		{
			 attachmentPartX = 0;
			 attachmentPartY = indexLastPart;
		}
		else if (tetherCounter == 3)
		{
			 attachmentPartX = indexLastPart;
			 attachmentPartY = indexLastPart;
		}

		VxVector3 pos2 = partMatrix[usedRingPartX[ringsNb-1]][usedRingPartY[ringsNb-1]]->getVxPart()->getPosition();
		VxVector3 pos1 = partMatrix[attachmentPartX][attachmentPartY]->getVxPart()->getPosition();
		VxVector3 vel2 = partMatrix[usedRingPartX[ringsNb-1]][usedRingPartY[ringsNb-1]]->getVxPart()->getLinearVelocity();
		VxVector3 vel1 = partMatrix[attachmentPartX][attachmentPartY]->getVxPart()->getLinearVelocity();
	
		VxVector3 relPos = pos2 - pos1;
		VxReal distance = relPos.normalize(); //make into unit vector and return original length
		currentRingsDistance = currentRingsDistance + distance; 

		VxVector3 relVel = vel2 - vel1;
		VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
		currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
		

		// apply control proportional to the current distance: winch in faster when distance is larger
		VxReal errorRingsDistance = currentRingsDistance - desiredRingsDistance;
		VxReal errorRingsVelocity = currentRingsRelVelocity;

		VxReal tetherLengthRateControl;
		if ( (errorRingsDistance > 0) && (cableMaxTension < maxAllowableTension) && (cableMinLength > minAllowableLength) )
			tetherLengthRateControl = proportionalGain * errorRingsDistance + derivativeGain * errorRingsVelocity;
		else
			 tetherLengthRateControl = 0.0;
			//tetherLengthRateControl = -proportionalGain*10.0* (cableMaxTension - 2.0);

		//std::cout << "current length rate: " << tetherLengthRateControl << std::endl;
		float currentWinchSpoolingVelocity = tetherLengthRateControl;

		if (currentWinchSpoolingVelocity < minCurrentWinchSpoolingVelocity)
			minCurrentWinchSpoolingVelocity = currentWinchSpoolingVelocity;
	}

	
	return minCurrentWinchSpoolingVelocity;
}

/*
void controlWinchRatePD(VxSmartInterface<Part> **partMatrix, int ringsNb, int *ringPartX, int *ringPartY, int indexLastPart, float winchRadius, VxReal desiredRingsDistance, VxReal proportionalGain, VxReal derivativeGain, VxSmartInterface<Hinge>* hingeForWinchPtr)
{
	VxReal currentRingsDistance = 0;
	VxReal currentRingsRelVelocity = 0;

	int wantedPartsX[4] = { 0, indexLastPart , indexLastPart , 0			};
	int wantedPartsY[4] = {0 , 0			 , indexLastPart , indexLastPart }; 	 

	// read position of parts with rings and sum them to get current distance 
	for (int i=0; i<4; i++)
	{
		// proportional part
		VxVector3 pos2 = partMatrix[wantedPartsX[i]][wantedPartsY[i]]->getVxPart()->getPosition();
		VxVector3 pos1 = partMatrix[wantedPartsX[i-1]][wantedPartsY[i-1]]->getVxPart()->getPosition();
	
		VxVector3 relPos = pos2 - pos1;
		VxReal distance = relPos.normalize(); //make into unit vector and return original length

		currentRingsDistance = currentRingsDistance + distance; 

		// derivative part
		VxVector3 vel2 = partMatrix[ringPartX[i]][ringPartY[i]]->getVxPart()->getLinearVelocity();
		VxVector3 vel1 = partMatrix[ringPartX[i-1]][ringPartY[i-1]]->getVxPart()->getLinearVelocity();

		VxVector3 relVel = vel2 - vel1;
	
		VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
		currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;
	}

	// add last segment distance, from last ring to attachment point

	// proportional part
	VxVector3 pos2 = partMatrix[ringPartX[ringsNb-1]][ringPartY[ringsNb-1]]->getVxPart()->getPosition();
	VxVector3 pos1 = partMatrix[indexLastPart][0]->getVxPart()->getPosition();
	VxVector3 relPos = pos2 - pos1;
	VxReal distance = relPos.normalize(); //make into unit vector and return original length
	currentRingsDistance = currentRingsDistance + distance; 

	// derivative part
	VxVector3 vel2 = partMatrix[ringPartX[ringsNb-1]][ringPartY[ringsNb-1]]->getVxPart()->getLinearVelocity();
	VxVector3 vel1 = partMatrix[indexLastPart][0]->getVxPart()->getLinearVelocity();
	VxVector3 relVel = vel2 - vel1;
	VxReal relVelocity = relVel.x()*relPos.x() + relVel.y()*relPos.y() + relVel.z()*relPos.z();
	currentRingsRelVelocity = currentRingsRelVelocity + relVelocity;

	// apply control proportional to the current distance: winch in faster when distance is larger
	VxReal errorRingsDistance = currentRingsDistance - desiredRingsDistance;
	VxReal errorRingsVelocity = currentRingsRelVelocity;

	VxReal tetherLengthRateControl = proportionalGain * errorRingsDistance + derivativeGain * errorRingsVelocity;
	std::cout << "current length rate: " << tetherLengthRateControl << std::endl;

	// transform tether length rate control to angular control 
	float winchAngularVelocityControl = tetherLengthRateControl / winchRadius; // (rad/s)
	(*hingeForWinchPtr)->getVxConstraint()->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, winchAngularVelocityControl);
	
}
*/