// Created by Eleonora Botta, September 8, 2016

#ifndef ATTACHDISTANCEN_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ATTACHDISTANCEN_H

#include "MyLibraryNet/attachDistance.h"

#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Sphere.h>
#include <Vx/VxPart.h>
#include <VxMath/VxMath.h>
#include <VxMath/Transformation.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxSim/VxSmartInterface.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

// define headers of cpp functions
void attachDistanceN(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> chaser, VxSmartInterface<Part> target, VxSmartInterface<Part>* partMatrix, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping, int numNode, float partMass);

void attachDistanceN_ST(VxSmartInterface<Assembly> assembly, VxSmartInterface<Part> chaser, VxSmartInterface<Part> target, VxSmartInterface<Part>* partMatrix, VxReal MaxDistance, VxReal threadStiffness, VxReal threadDamping, int numNode, float partMass);
#endif