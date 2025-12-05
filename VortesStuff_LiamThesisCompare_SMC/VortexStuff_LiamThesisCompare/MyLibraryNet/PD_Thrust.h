#ifndef PDTHRUST_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define PDTHRUST_H

//#include "MyLibraryNet/PD_Thrust.h"
#include "MyLibraryNet/QuatMat.h"


#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <Vx/VxCylindrical.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxMath/Transformation.h>
#include <VxMath/VxMath.h>
#include <Vx/VxQuaternion.h>

#include <VxSim/IMobile.h>	// this is needed by VxSmartInterface<IExtension> cableExtension


#include <VxData/Vector.h>
#include <Vx/VxSmartPtr.h>


#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;


std::vector<double> PD_Thrust(VxSmartInterface<Part>* p1, VxSmartInterface<Part>* chaserPartPtr, std::array<double, 4> PD_States, VxSmartInterface<DistanceJoint> tetherJoints[], int Direction, const int numnodes);
#endif

