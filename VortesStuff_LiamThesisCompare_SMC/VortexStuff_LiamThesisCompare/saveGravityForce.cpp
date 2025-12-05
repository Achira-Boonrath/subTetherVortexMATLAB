// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)
// Modified by Eleonora Botta, November 5, 2016: rearranged the file with the positions of the cable points (for all cables)
// Modified by Eleonora Botta, December 28, 2016: added a version of function without outTetherPoints

// Save dynamics of the tether and winch linear velocity

#include "MyLibraryNet/saveGravityForce.h" //-> HOW DOES THIS WORK

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

void saveGravityForce(VxVector3 chs, VxVector3 tgt, FILE* outGravity)
{

	
	fprintf( outGravity, "%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f", chs.x(),chs.y(),chs.z(), tgt.x(), tgt.y(), tgt.z());
	
	fprintf(outGravity, "\n" );

}