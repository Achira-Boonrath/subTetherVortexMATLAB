// Created by Eleonora Botta, September 9, 2016

#ifndef CREATENET_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define CREATENET_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/attachDistance.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Sphere.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>

#include <iostream>
#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
float createNet(VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Assembly> assembly, std::string pathData, VxSmartInterface<Part> **partMatrix, VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[], VxReal totalMass, VxMaterial* myNetMaterial00, VxMaterial* myNetMaterial01, VxMaterial* myNetMaterial11, FILE* FileLog);

#endif