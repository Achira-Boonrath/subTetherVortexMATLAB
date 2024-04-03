// Created by Eleonora Botta, September 6, 2016
// Modified by Eleonora Botta, September 23, 2016: pass partMatrix as VxSmartInterface<Part> **partMatrix
// Modified by Eleonora Botta, April 1, 2017: add function to save all masses of the net (and CMs) in a file.

#ifndef SAVEDYNAMICSQUANTITIESNET_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SAVEDYNAMICSQUANTITIESNET_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <stdio.h>
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

// define headers of cpp functions
void saveDynamicsQuantitiesNet(int netModel, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], FILE* outX, FILE* outY, FILE* outZ, FILE* outVX, FILE* outVY, FILE* outVZ, FILE* outF, int NetSize, int CMsPresent);

void saveDynamicsQuantitiesBeam(VxSmartInterface<Part> partMatrix[], VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[], FILE* outX, FILE* outY, FILE* outZ, FILE* outVX, FILE* outVY, FILE* outVZ, FILE* outF, FILE* outAxes, int NetSize, int CMsPresent);

void saveMassNet(VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], FILE* outMass, int NetSize, int CMsPresent);

#endif