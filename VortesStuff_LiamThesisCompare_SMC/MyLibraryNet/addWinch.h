// Created by Eleonora Botta, September 19, 2016
// Modified by Eleonora Botta, September 28, 2016: winch passed as pointer to createScene for main to know it at runtime
// Modified by Eleonora Botta, September 29, 2016: hinge passed as pointer to createScene for main to know it at runtime

#ifndef ADDWINCH_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ADDWINCH_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Box.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>
#include <Vx/VxCylinder.h>
#include <Vx/VxTransform.h>
#include <VxDynamics/Hinge.h>
#include <Vx/VxHinge.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/Cylinder.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
void addWinch(VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part>* winchPartPtr, VxSmartInterface<Hinge>* hingeForWinchPtr);

#endif