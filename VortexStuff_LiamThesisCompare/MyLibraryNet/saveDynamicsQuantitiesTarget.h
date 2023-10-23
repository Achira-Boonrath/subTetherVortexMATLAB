// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)

#ifndef SAVEDYNAMICSQUANTITIESCHASER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SAVEDYNAMICSQUANTITIESCHASER_H

// Include all libraries needed by cpp files in this header
#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>

#include <VxSim/IMobile.h>
#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;

VxReal saveDynamicsQuantitiesChaser(VxSmartInterface<Part> chaserPart, FILE* outChaser);


#endif