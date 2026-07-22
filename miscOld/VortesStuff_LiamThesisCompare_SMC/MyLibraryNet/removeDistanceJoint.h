#ifndef REMOVEDISTANCVEJOINT_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define REMOVEDISTANCVEJOINT_H 

#include <iostream>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Constraint.h>
#include <Vx/VxPart.h>
#include <VxSim/VxSmartInterface.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;

VxDynamics::DistanceJoint removeDistanceJoint(VxDynamics::DistanceJoint d);
#endif