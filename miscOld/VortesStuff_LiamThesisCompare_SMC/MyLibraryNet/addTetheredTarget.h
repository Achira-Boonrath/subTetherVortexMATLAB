// Created by Liam Field 03/09/22
// Designed to simulate a lower order model, where the tether is directly attached to the target.

#ifndef ADDTETETHEREDTARGET_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ADDTETETHEREDTARGET_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"

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
void addTetheredTarget(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part>* tetheredTargetPartPtr);

#endif