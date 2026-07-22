// Created by Eleonora Botta, September 12, 2016

#ifndef ATTACHPRISMATIC_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ATTACHPRISMATIC_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

VxSmartInterface<Prismatic> attachPrismatic(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> p1, VxSmartInterface<Part> p2, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping, VxReal threadBendingStiffness, VxReal threadBendingDamping, VxReal threadTransverseStiffness, VxReal threadTransverseDamping);

#endif