// Created by Liam Field 3/21/22
// Sliding mode controller for chaser attitude

#include "MyLibraryNet/ErrorQuatCalc.h"


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

// Matrix for quaternion integration/multiplication. k = 0 -> scalar last, other - > scalar first
VxVector3 ErrorCalcQuat(VxQuaternion DesQ, VxQuaternion ChaserQ)
{
	VxVector3 Equat;

	auto w = DesQ.w();
	auto x = DesQ.x();
	auto y = DesQ.y();
	auto z = DesQ.z();

	auto w2 = ChaserQ.w();
	auto x2 = ChaserQ.x();
	auto y2 = ChaserQ.y();
	auto z2 = ChaserQ.z();
	
	Equat[0] = w* x2 - w2 * x - y * z2 + y2 * z;
	Equat[1] = w* y2 - w2 * y + x * z2 - x2 * z;
	Equat[2] = w* z2 - w2 * z - x * y2 + x2 * y;

	return Equat;
}