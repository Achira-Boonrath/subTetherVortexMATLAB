// Created by Eleonora Botta, February 16, 2017

#ifndef SAVERINGSDISTANCE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SAVERINGSDISTANCE_H

// Include all libraries needed by cpp files in this header
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

float saveRingsDistance3Rings(VxSmartInterface<Part> **partMatrix, int indexLastPart, int indexCenterPart, FILE* outRingsDistance);

float saveRingsDistance2Tethers(VxSmartInterface<Part> **partMatrix, int indexLastPart, int indexCenterPart, FILE* outRingsDistance);


#endif