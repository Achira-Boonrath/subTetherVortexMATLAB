// Created by Liam Field April 28th,2022
// To do: Computes gravity force for 2BP, given a mu and part

#include "MyLibraryNet/OrbitalMech2BP.h"

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

Vx::VxVector3 OrbitalMech2BP(VxSmartInterface<Part>* part, VxReal mu)
{
	VxReal mass = (*part)->getVxPart()->getMass();
	Vx::VxVector3 Position = (*part)->getVxPart()->getPosition();
	Vx::VxReal Denominator = std::pow(Position.norm(), 3);
	Vx::VxVector3 gravityF = -mu * (mass)*Position / Denominator;
	return gravityF;
}

Vx::VxVector3 OrbitalMech2BP(VxPart* part, VxReal mu)
{
	VxReal mass = part->getMass();
	Vx::VxVector3 Position = part->getPosition();
	Vx::VxReal Denominator = std::pow(Position.norm(), 3);
	Vx::VxVector3 gravityF = -mu * (mass)*Position / Denominator;
	return gravityF;
}