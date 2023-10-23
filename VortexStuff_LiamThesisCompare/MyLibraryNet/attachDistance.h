// Created by Eleonora Botta, September 8, 2016

#ifndef ATTACHDISTANCE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ATTACHDISTANCE_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
void attachDistance(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> p1, VxSmartInterface<Part> p2, VxVector3 p1Pos, VxVector3 p2Pos, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping);

#endif