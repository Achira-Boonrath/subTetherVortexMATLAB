// Created by Eleonora Botta, September 6, 2016
// Modified by Eleonora Botta, September 12, 2016

#include "MyLibraryNet/saveDynamicsQuantities.h"

#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


void saveDynamicsQuantitiesBeam(VxSmartInterface<Part> partMatrix[], VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[], FILE* outX, FILE* outY, FILE* outZ, FILE* outVX, FILE* outVY, FILE* outVZ, FILE* outF, FILE* outAxes, int NetSize, int CMsPresent)
 {
	 // save dynamics quantities for net
	 for (int j=0; j<NetSize; j++)
		{
				VxVector3 pos = partMatrix[j]->getVxPart()->getPosition();
				VxVector3 vel = partMatrix[j]->getVxPart()->getLinearVelocity();
				fprintf(outX, "%g ", pos.x() ); // out x to the file
				fprintf(outY, "%g ", pos.y() ); 
				fprintf(outZ, "%g ", pos.z() ); 
				fprintf(outVX, "%g ", vel.x() ); 
				fprintf(outVY, "%g ", vel.y() ); 
				fprintf(outVZ, "%g ", vel.z() ); 
		}

	// save tensions in threads
	VxReal threadsNumber = NetSize-1;

	for (int k=0; k<threadsNumber; k++) // save tension for attachDistance
	{
		/*VxReal TensionForce = allConstraintsVector[k]->getVxConstraint()->getConstraintEquationForce(0);
		fprintf(outF, "%1.15f ", TensionForce ); */

		VxReal3 fromP2toP1 = {0,0,0};
		VxReal3 secondary = {0,0,0};
		allConstraintsVectorPrismatic[k]->getVxConstraint()->getPartAttachmentAxes (0, fromP2toP1 , secondary);
		fprintf(outAxes, "%1.15f %1.15f %1.15f    %1.15f %1.15f %1.15f " , fromP2toP1[0],fromP2toP1[1],fromP2toP1[2], secondary[0],secondary[1],secondary[2]);
		//allConstraintsVectorPrismatic[k]->getVxConstraint()->updateAttachmentFromPart(1);
	}

	fprintf(outX, "\n"); // new line to the file
	fprintf(outY, "\n");
	fprintf(outZ, "\n");
	fprintf(outVX, "\n"); 
	fprintf(outVY, "\n");
	fprintf(outVZ, "\n");
	fprintf(outF, "\n");
	fprintf(outAxes, "\n");
 }