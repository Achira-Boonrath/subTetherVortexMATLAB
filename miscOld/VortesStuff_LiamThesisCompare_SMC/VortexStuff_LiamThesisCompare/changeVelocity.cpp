// Created by Eleonora Botta, September 29, 2016
// Modified by Eleonora Botta, October 12, 2016: function with controls, velocities, accelerations

// Note: it could be more efficient than this, e.g., with check before this is called to see if velocity or control has to change. 

#include "MyLibraryNet/changeVelocity.h"

#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Hinge.h>
#include <Vx/VxHinge.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


float changeVelocity(VxSmartInterface<Hinge>* hingeForWinchPtr, float currentTime, float functionTimes[], std::string functionControls[], float functionVelocities[], float functionAccelerations[], int sizeFunctionTimes, float winchRadius)
{
	float currentVelocity;
	//std::cout << "I am in changeVelocity"<< std::endl;
	
	// find in what piece of the function you are
	int index;
	for (int i=1; i<sizeFunctionTimes; i++)
	{
		if ( currentTime < functionTimes[i] )
		{
			index = i-1;
			break;
		}
		else
			index = i;
	}
			
	// set hinge is locked
	if (functionControls[index].find("l") != std::string::npos)
	{
		//std::cout << "winch locked" << std::endl; 
		(*hingeForWinchPtr)->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlLocked); 
	}			
	// set hinge is free
	else if (functionControls[index].find("f") != std::string::npos)
	{
		//std::cout << "winch free" << std::endl; 
		(*hingeForWinchPtr)->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlFree); 
	}
	// set hinge is motorized: set also its velocity
	else if (functionControls[index].find("m") != std::string::npos)
	{
		(*hingeForWinchPtr)->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlMotorized); 					
		currentVelocity = functionVelocities[index] + functionAccelerations[index] * ( currentTime - functionTimes[index]);
		float winchAngularVelocity = currentVelocity / winchRadius; // (rad/s)
		(*hingeForWinchPtr)->getVxConstraint()->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, winchAngularVelocity); 
	} 

	return currentVelocity;
		//std::cout << "I have set the velocity"<< std::endl;

}

