// Created by Liam Field April 28th,2022
// To do: Computes gravity force for 2BP, given a mu and part

#ifndef ORBITALMECH2BP_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ORBITALMECH2BP_H

// Include all libraries needed by cpp files in this header

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Box.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
Vx::VxVector3 OrbitalMech2BP(VxSmartInterface<Part>* part, VxReal mu);

Vx::VxVector3 OrbitalMech2BP(VxPart* part, VxReal mu);
#endif