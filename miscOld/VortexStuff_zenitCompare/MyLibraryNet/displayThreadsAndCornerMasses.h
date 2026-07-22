// Created by Eleonora Botta, September 6, 2016

#ifndef DISPLAYTHREADSANDCORNERMASSES_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define DISPLAYTHREADSANDCORNERMASSES_H

// Include all libraries needed by cpp files in this header
#include <VxMath/Transformation.h>
#include <VxGraphics/ShapeGenerator.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/CollisionGeometry.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

#define MAXGRIDSIZE 100

// define headers of cpp functions
void displayThreadsAndCornerMasses(VxGraphics::ShapeGenerator *mShapeGenerator, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], int gNetSize, int CMsPresent);

#endif