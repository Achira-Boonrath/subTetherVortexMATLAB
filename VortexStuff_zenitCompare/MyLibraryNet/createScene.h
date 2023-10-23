// Created by Eleonora Botta, September 6, 2016
// Modified by Eleonora Botta, September 15, 2016: chaser 
// Modified by Eleonora Botta, September 29, 2016: hinge passed as pointer to createScene for main to know it at runtime

#ifndef CREATESCENE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CREATESCENE_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/createNet.h"
#include "MyLibraryNet/addCornerMasses.h"
#include "MyLibraryNet/addChaser.h"
#include "MyLibraryNet/readParameter.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxSim/VxSmartInterface.h>
#include <VxContent/Scene.h>
#include <VxContent/SceneICD.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/MechanismICD.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <VxDynamics/Hinge.h>

#include <iostream>
#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxContent;
using namespace VxSim;

// define headers of cpp functions
//VxSim::VxSmartInterface<VxContent::Scene>  createScene(std::string pathData, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<Part> chaserPart, VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[]); // for net
//VxSim::VxSmartInterface<VxContent::Scene>  createScene(std::string pathData, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* winchPartPtr, VxSmartInterface<Hinge>* hingeForWinchPtr, VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[]); // for net

VxSim::VxSmartInterface<VxContent::Scene>  createScene(std::string pathData, VxSmartInterface<Part>* partMatrix, VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* tetheredTargetPartPtr, const int numNode, int TetherType, VxSmartInterface<DistanceJoint> allConstraintsVector[]);



#endif