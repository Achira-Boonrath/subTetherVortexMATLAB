// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)
// Modified by Eleonora Botta, November 5, 2016: rearranged the file with the positions of the cable points (for all cables)
// Modified by Eleonora Botta, December 28, 2016: added a version of function without outTetherPoints

// Save dynamics of the tether and winch linear velocity

#include "MyLibraryNet/saveDynamicsQuantitiesNodes.h"

#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxMath/Transformation.h>
#include <VxMath/VxMath.h>
#include <Vx/VxQuaternion.h>

#include <VxSim/IMobile.h>	// this is needed by VxSmartInterface<IExtension> cableExtension


#include <VxData/Vector.h>
#include <Vx/VxSmartPtr.h>

#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;

void saveDynamicsQuantitiesNodes(VxSmartInterface<Part>* partVector, FILE* outNodesPos, FILE* outNodesVel, FILE* outTension, const int numNode)
{
	/*std::vector<double> outPos(numNode*3);
	std::vector<double> outVel(numNode*3);*/

	const char* B = "%f \t %f \t %f";
	const char* BC = " \t %f \t %f \t %f";
	/*const char* C = " \t ";
	char* g = new char[strlen(B)*numNode+strlen(C)*(numNode-1)];
	int charL = 14;
	
	for (int i = 0; i < numNode; i++)
	{
		if (i == numNode-1)
		{
			strcat(g, B);
		}
		else 
		{
			strcat(g, B);
			strcat(g, C);
		}
		

	}*/



	//outVel.push_back(1);
	for (int i = 0; i < numNode; i++)
	{
		VxVector3 chaserpos = partVector[i]->getVxPart()->getPosition();
		VxVector3 vel = partVector[i]->getVxPart()->getLinearVelocity();

		if (i == 0) {
			fprintf(outNodesPos, B, chaserpos.x(), chaserpos.y(), chaserpos.z());
			fprintf(outNodesVel, B, vel.x(), vel.y(), vel.z());
		}
		else
		{
			fprintf(outNodesPos, BC, chaserpos.x(), chaserpos.y(), chaserpos.z());
			fprintf(outNodesVel, BC, vel.x(), vel.y(), vel.z());

		}
		/*if (i == 0) 
		{
			outPos[0] = chaserpos[0];
			outPos[1] = chaserpos[1];
			outPos[2] = chaserpos[2];
			outVel[0] = vel[0];
			outVel[1] = vel[1];
			outVel[2] = vel[2];
		}
		else
		{
			outPos[3*i] = chaserpos[0];
			outPos[3*i+1] = chaserpos[1];
			outPos[3*i+2] = chaserpos[2];
			outVel[3 * i] = vel[0];
			outVel[3 * i + 1] = vel[1];
			outVel[3 * i + 2] = vel[2];
		}*/
		//outPos.insert(outPos.begin(), chaserpos[0] , chaserpos[2]); <-- entirely breaks
		
		
	}
	VxSmartInterface<Assembly> Assem = partVector[0]->getParentAssembly();
	for (int i = 0; i < numNode+1; i++)
	{
		
		//VxSmartInterface<Assembly> Assem1 = Assem.getObject();
		VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[i];
		VxReal TensionForce = v->getVxConstraint()->getConstraintEquationForce(0);
		//const int count = partVector[i]->getVxPart()->getConstraintCount();
		if (i == 0)
		{
			fprintf(outTension, "%1.15f", TensionForce);
		}
		else 
		{
			fprintf(outTension, " \t %1.15f", TensionForce);
		}

	}
	fprintf(outTension, "\n");
	fprintf(outNodesPos, "\n");
	fprintf(outNodesVel, "\n");
}