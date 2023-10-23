// Created by Liam Field 3/21/22
// Sliding mode controller for chaser attitude

#include "MyLibraryNet/QuatMat.h"


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
VxMath::Matrix44 QuatMat(VxQuaternion DesQ, int k)
{
	// Column 4 is zeros

	VxMath::Matrix44 QMatD;
	if (k == 0)
	{
		// Scalar Last
		QMatD.operator()(0, 0) = DesQ.w();
		QMatD.operator()(1, 1) = DesQ.w();
		QMatD.operator()(2, 2) = DesQ.w();
		QMatD.operator()(0, 1) = -DesQ.z();
		QMatD.operator()(1, 0) = DesQ.z();
		QMatD.operator()(0, 2) = DesQ.y();
		QMatD.operator()(2, 0) = -DesQ.y();
		QMatD.operator()(1, 2) = -DesQ.x();
		QMatD.operator()(2, 1) = DesQ.x();

		QMatD.operator()(3, 0) = -DesQ.x();
		QMatD.operator()(3, 1) = -DesQ.y();
		QMatD.operator()(3, 2) = -DesQ.z();
		QMatD.operator()(3, 3) = 0;
		//////////////
	}
	else
	{
		// scalar first
		QMatD.operator()(1, 0) = DesQ.w();
		QMatD.operator()(2, 1) = DesQ.w();
		QMatD.operator()(3, 2) = DesQ.w();
		QMatD.operator()(1, 1) = -DesQ.z();
		QMatD.operator()(2, 0) = DesQ.z();
		QMatD.operator()(1, 2) = DesQ.y();
		QMatD.operator()(3, 0) = -DesQ.y();
		QMatD.operator()(2, 2) = -DesQ.x();
		QMatD.operator()(3, 1) = DesQ.x();

		QMatD.operator()(0, 0) = -DesQ.x();
		QMatD.operator()(0, 1) = -DesQ.y();
		QMatD.operator()(0, 2) = -DesQ.z();
		QMatD.operator()(3, 3) = 0;

	}

	return QMatD;
}