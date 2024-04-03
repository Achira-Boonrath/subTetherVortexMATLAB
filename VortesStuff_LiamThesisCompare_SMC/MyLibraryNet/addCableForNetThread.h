// Created by Eleonora Botta, February 27, 2017
// Modified by Eleonora Botta, March 16, 2017: pass some parameters for cables, previously read from netData.txt

#ifndef ADDCABLEFORNETTHREAD_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ADDCABLEFORNETTHREAD_H

// Include all libraries needed by cpp files in this header
#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxFactoryKey.h>
#include <VxSim/VxSmartInterface.h>
#include <CableSystems/CableSystemsICD.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/Part.h>
#include <VxData/Container.h>
#include <VxData/FieldBase.h>
//#include <Vx/VxConnectionFactory.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

#define MAXGRIDSIZE 100

// define headers of cpp functions

void addCableForNetThread(VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> part0, VxSmartInterface<Part> part1, VxReal threadStiffness, VxReal threadDamping, VxReal threadBendingStiffness, VxReal threadBendingDamping, VxReal MeshLength, VxReal netRadius, VxReal linearNetDensity, int wayPointsNb, int bendingPresentInNet, int netCGtype );

#endif