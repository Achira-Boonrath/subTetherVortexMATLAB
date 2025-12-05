// Created by Liam Field April 28th,2022
// To do: Computes gravity force for 2BP, given a mu and part

#include "MyLibraryNet/OrbitalMech2BP_CoM.h"
#include "MyLibraryNet/matrixMult.h"
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

Vx::VxVector3 OrbitalMech2BP_CoM(VxSmartInterface<Part>* chaser, VxSmartInterface<Part>* target, VxSmartInterface<Part>* partVector, VxReal mu, const int numNode)
{
	double PosVec[3][2 + 4];
	std::vector < double > MassVec;


	VxVector3 CoM_Pos;
	VxVector3 CoM_Grav;
	double totalMass;






	for (int i = 0; i < 3; i++)
	{
		PosVec[i][0] = (*chaser)->getVxPart()->getPosition()[i];
		PosVec[i][1] = (*target)->getVxPart()->getPosition()[i];

	}



	MassVec.push_back((*chaser)->getVxPart()->getMass());
	MassVec.push_back((*target)->getVxPart()->getMass());

	for (int i = 0; i < numNode; i++)
	{
		MassVec.push_back(partVector[i]->getVxPart()->getMass());

		for (int j = 0; j < 3; j++)
		{
			PosVec[j][i + 2] = partVector[i]->getVxPart()->getPosition()[j];
		}

	}

	//auto h = std::sizeof(PosVec);
	auto C = matrixMult(PosVec, MassVec);
	/*VxVector3 CoM_Pos = VxVector3(C[0], C[1], C[2])*/;

	for (int j = 0; j < 6; j++)
	{
		totalMass += MassVec[j];
	}

	CoM_Pos = VxVector3(C[0], C[1], C[2]).operator/(totalMass);
	CoM_Grav = -mu * CoM_Pos / pow(CoM_Pos.norm(), 3);

	return CoM_Grav;

}