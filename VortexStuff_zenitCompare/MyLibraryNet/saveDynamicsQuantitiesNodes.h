// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)

#ifndef SAVEDYNAMICSQUANTITIESNODES_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SAVEDYNAMICSQUANTITIESNODES_H

// Include all libraries needed by cpp files in this header
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

#include <VxSim/IMobile.h>
#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;

void saveDynamicsQuantitiesNodes(VxSmartInterface<Part>* partVector, FILE* outNodes, FILE* outNodesVel, FILE* outTension, const int numNode, int TetherType);
//void saveDynamicsQuantitiesNodes(VxSmartInterface<Part>* partVector, FILE* outNodes, FILE* outNodesVel, FILE* outTension, const int numNode);

#endif