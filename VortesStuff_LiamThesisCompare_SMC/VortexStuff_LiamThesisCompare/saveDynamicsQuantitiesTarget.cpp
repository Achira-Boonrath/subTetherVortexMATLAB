// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)
// Modified by Eleonora Botta, November 5, 2016: rearranged the file with the positions of the cable points (for all cables)
// Modified by Eleonora Botta, December 28, 2016: added a version of function without outTetherPoints

// Save dynamics of the tether and winch linear velocity

#include "MyLibraryNet/saveDynamicsQuantitiesTether.h"

#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>


#include <VxSim/IMobile.h>	// this is needed by VxSmartInterface<IExtension> cableExtension


#include <VxData/Vector.h>
#include <Vx/VxSmartPtr.h>

#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;

VxReal saveDynamicsQuantitiesChaser(VxSmartInterface<Part> chaserPart, FILE* outChaser)
{
	VxVector3 chaserpos = chaserPart->getVxPart()->getPosition();
	VxVector3 angVel = chaserPart->getVxPart()->getAngularVelocity();
	VxVector3 vel = chaserPart->getVxPart()->getLinearVelocity();
	VxVector3 force = chaserPart->getVxPart()->getForce();
	VxVector3 accel = chaserPart->getVxPart()->getLinearAcceleration();
	VxReal4 quat;
		chaserPart->getVxPart()->getOrientationQuaternion(quat);
	VxReal chaserZ = chaserpos.z();
	
	fprintf( outChaser, "%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f", chaserpos.x(), chaserpos.y(), chaserpos.z(),vel.x(),vel.y(),vel.z()
		,angVel.x(), angVel.y(), angVel.z(),force.x(),force.y(),force.z(),quat[0],quat[1],quat[2],quat[3]);
	
	fprintf( outChaser, "\n" );
	return chaserZ;
}