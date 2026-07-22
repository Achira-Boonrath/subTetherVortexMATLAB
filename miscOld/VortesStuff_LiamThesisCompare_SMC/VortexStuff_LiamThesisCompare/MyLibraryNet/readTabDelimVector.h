#ifndef READTABDELIMVECTOR_H   // To make sure you don't declare the function more than once by including the header multiple times.
#define READTABDELIMVECTOR_H

// Include all libraries needed by cpp files in this header
#include <iostream>
#include <sstream>
#include <fstream>
#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxMath/Transformation.h>
#include <VxMath/VxMath.h>
#include <Vx/VxQuaternion.h>

// define headers of cpp functions

Vx::VxVector3 readTabDelimVector(std::stringstream& fileName, int lineNumber);

#endif