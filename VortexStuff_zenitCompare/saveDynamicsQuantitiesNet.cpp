// Created by Eleonora Botta, September 6, 2016
// Modified by Eleonora Botta, September 12, 2016
// Modified by Eleonora Botta, September 23, 2016: pass partMatrix as VxSmartInterface<Part> **partMatrix
// Modified by Eleonora Botta, March 16, 2017: modified numbering of netModel: 0 = LP, 1 = cable-based. 
// Modified by Eleonora Botta, April 1, 2017: add function to save all masses of the net (and CMs) in a file.

#include "MyLibraryNet/saveDynamicsQuantitiesNet.h"

#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;


void saveDynamicsQuantitiesNet(int netModel, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], FILE* outX, FILE* outY, FILE* outZ, FILE* outVX, FILE* outVY, FILE* outVZ, FILE* outF, int NetSize, int CMsPresent)
 {
	 // save dynamics quantities for net
	 for (int j=0; j<NetSize; j++)
		{
			for (int i=0; i<NetSize; i++)
			{
				VxVector3 pos = partMatrix[i][j]->getVxPart()->getPosition();
				VxVector3 vel = partMatrix[i][j]->getVxPart()->getLinearVelocity();
				fprintf(outX, "%g ", pos.x() ); // out x to the file
				fprintf(outY, "%g ", pos.y() ); 
				fprintf(outZ, "%g ", pos.z() ); 
				fprintf(outVX, "%g ", vel.x() ); 
				fprintf(outVY, "%g ", vel.y() ); 
				fprintf(outVZ, "%g ", vel.z() ); 
			}
		}

	 // save dynamics quantities for corner masses - if present
	 if (CMsPresent==1)
	 {
		for (int j=0; j<4; j++)
		{
			VxVector3 pos = cornerMasses[j]->getVxPart()->getPosition();
			VxVector3 vel = cornerMasses[j]->getVxPart()->getLinearVelocity();
			fprintf(outX, "%g ", pos.x() ); // out x to the file
			fprintf(outY, "%g ", pos.y() );
			fprintf(outZ, "%g ", pos.z() );
			fprintf(outVX, "%g ", vel.x() ); 
			fprintf(outVY, "%g ", vel.y() ); 
			fprintf(outVZ, "%g ", vel.z() ); 
		}
	 }

	// save tensions in threads
	VxReal threadsNumber;
	if (CMsPresent==0)
		threadsNumber = 2*NetSize*(NetSize-1);
	else if (CMsPresent==1)
		threadsNumber = 2*NetSize*(NetSize-1)+4;

	if (netModel == 0)
	{
		for (int k=0; k<threadsNumber; k++) // save tension for attachDistance
		{
			VxReal TensionForce = allConstraintsVector[k]->getVxConstraint()->getConstraintEquationForce(0);
			fprintf(outF, "%1.15f ", TensionForce ); 
		}
	}

	fprintf(outX, "\n"); // new line to the file
	fprintf(outY, "\n");
	fprintf(outZ, "\n");
	fprintf(outVX, "\n"); 
	fprintf(outVY, "\n");
	fprintf(outVZ, "\n");
	fprintf(outF, "\n");
 }


void saveMassNet(VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], FILE* outMass, int NetSize, int CMsPresent)
{
	for (int j=0; j<NetSize; j++)
		{
			for (int i=0; i<NetSize; i++)
			{
				VxReal partMass = partMatrix[i][j]->getVxPart()->getMass();
				fprintf(outMass, "%g \n", partMass );
			}
		}
		if (CMsPresent==1)
		{
			for (int j=0; j<4; j++)
			{
				VxReal partMass = cornerMasses[j]->getVxPart()->getMass();
				fprintf(outMass, "%g \n", partMass );
			}
		}
}