// Created by Eleonora Botta, September 15, 2016
// Modified by Eleonora Botta, September 23, 2016: chaser passed as pointer, for the main to know it

#ifndef ADDCHASER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ADDCHASER_H

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
void addChaser(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part>* chaserPartPtr);

#endif