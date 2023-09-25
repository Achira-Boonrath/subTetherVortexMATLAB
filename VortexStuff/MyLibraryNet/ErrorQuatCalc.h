// Created by Liam Field May 10, 2023
// Calculates error quaternion between two quaternions
#ifndef ERRORQUATCALC_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ERRORQUATCALC_H


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

#include <VxSim/IMobile.h>	// this is needed by VxSmartInterface<IExtension> cableExtension


#include <VxData/Vector.h>
#include <Vx/VxSmartPtr.h>


#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;

VxVector3 ErrorCalcQuat(VxQuaternion DesQ, VxQuaternion ChaserQ);


#endif