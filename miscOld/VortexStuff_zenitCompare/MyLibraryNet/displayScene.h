// Created by Eleonora Botta, September 21, 2016
// Modified by Eleonora Botta, September 23, 2016: graphics for chaser
// Modified by Eleonora Botta, September 29, 2016: graphics for winch.
// Modified by Eleonora Botta, March 3, 2017: pass netModel
// Modified by Eleonora Botta, May 14, 2020: added visualization of std closing mechanism

#ifndef DISPLAYSCENE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define DISPLAYECENE_H

// Include all libraries needed by cpp files in this header
#include <VxMath/Transformation.h>
#include <VxGraphics/ShapeGenerator.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/CollisionGeometry.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
//void displayScene(VxGraphics::ShapeGenerator *mShapeGenerator, VxSmartInterface<Part>** partMatrix, VxSmartInterface<Part> cornerMasses[], int gNetSize, int CMsPresent, int chaserPresent, VxSmartInterface<Part>* chaserPartPtr, float chaserSideLength, VxSim::VxSmartInterface<VxDynamics::Part>* winchPartPtr, float winchRadius, int netModel);

//void displayScene(VxGraphics::ShapeGenerator* mShapeGenerator, VxSmartInterface<Part>* partMatrix, VxSmartInterface<Part>* chaserPartPtr, float chaserSideLength, VxSmartInterface<Part>* tetheredTargetPartPtr);
void displayScene(VxGraphics::ShapeGenerator* mShapeGenerator, VxSmartInterface<Part>* partMatrix, VxSmartInterface<Part>* chaserPartPtr, float chaserSideLength, VxSmartInterface<Part>* tetheredTargetPartPtr, int numNode,int TetherType);//AB, 


#endif