// Created by Eleonora Botta, June 23, 2017

#ifndef ADDSTDCLOSINGMECHANISM_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ADDSTDCLOSINGMECHANISM_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/addStdClosingMechanism.h"
#include "MyLibraryNet/attachDistance.h"
#include "MyLibraryNet/readParameter.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>

#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
void addStdClosingMechanismCMs(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[]);

void addStdClosingMechanismNetPerimeter(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<Part> **partMatrix, VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[]);

void activateStdClosingMechanismCMs(int NetSize, VxReal NetSideLength, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[]);

void activateStdClosingMechanismNetPerimeter(int NetSize, VxReal NetSideLength, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<Part> **partMatrix, VxSmartInterface<DistanceJoint> closingMechanismConstraintsVector[]);

#endif