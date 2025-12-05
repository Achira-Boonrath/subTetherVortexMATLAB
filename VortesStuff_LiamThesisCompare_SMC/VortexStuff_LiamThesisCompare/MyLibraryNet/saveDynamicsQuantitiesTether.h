// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)

#ifndef SAVEDYNAMICSQUANTITIESTETHER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SAVEDYNAMICSQUANTITIESTETHER_H

// Include all libraries needed by cpp files in this header
#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Hinge.h>

#include <VxSim/IMobile.h>	// this is needed by VxSmartInterface<IExtension> cableExtension

#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;

void saveDynamicsQuantitiesTether(VxSmartInterface<IExtension> cableExtension, FILE* outTether);

void saveDynamicsQuantitiesTether(VxSmartInterface<IExtension> cableExtension, FILE* outTether, VxSmartInterface<Hinge>* hingeForWinchPtr, float currentWinchSpoolingVelocity);

//void saveDynamicsQuantitiesTether(VxSmartInterface<IExtension> cableExtension, FILE* outTether, FILE* outTetherPoints, VxSmartInterface<Hinge>* hingeForWinchPtr, float currentWinchSpoolingVelocity);

//void saveDynamicsQuantitiesTether(VxSmartInterface<IExtension> cableExtension, FILE* outTether, FILE* outTetherPoints, float timestep);


#endif