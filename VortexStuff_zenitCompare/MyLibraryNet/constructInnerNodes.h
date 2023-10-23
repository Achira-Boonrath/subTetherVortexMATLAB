
#ifndef CONSTRUCTINNERNODES_H   // To make sure you don't declare the function more than once by including the header multiple times.
#define CONSTRUCTINNERNODES_H

// Include all libraries needed by cpp files in this header
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/attachDistance.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxContent/Scene.h>
#include <VxContent/SceneICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Part.h>
#include <VxDynamics/Sphere.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxSim/VxSmartInterface.h>
#include <VxMath/Transformation.h>

#include <iostream>
#include <fstream>

// third party libraries
#include <Eigen/Eigen/Core> // for computing eigenvalues
#include <Eigen/Eigen/Dense>
#include <Eigen/Eigen/Eigenvalues>
#define MAXGRIDSIZE 301

using namespace Vx;
using namespace VxDynamics;
using namespace VxContent;
using namespace VxSim;

// define headers of cpp functions
//int constructInnerNodes(VxSmartInterface<Assembly> assembly, std::string pathData, std::string pathResults, VxSmartInterface<Part>** partMatrix, VxReal** masstMatrixBeforeSwitch, VxReal** masstMatrixAfterSwitch ,VxSmartInterface<Part>** InnerNodeMatrix, VxSmartInterface<Part>** CornerMassMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], int numNodes, int NumNodesToAdd);
//int constructInnerNodes(VxSmartInterface<Assembly> assembly, std::string pathData, std::string pathResults, VxSmartInterface<Part>** partMatrix, VxReal** masstMatrixBeforeSwitch, VxReal** masstMatrixAfterSwitch, VxSmartInterface<Part>** InnerNodeMatrix, VxSmartInterface<Part>** CornerMassMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxVector3** sumFExtBeforeSwitch, VxReal** sumFdotVBeforeSwitch, int numNodes, int NumNodesToAdd);
int constructInnerNodes(VxSmartInterface<Assembly> assembly, std::string pathData, std::string pathResults, VxSmartInterface<Part>** partMatrix, VxReal** masstMatrixBeforeSwitch, VxReal** masstMatrixAfterSwitch, VxSmartInterface<Part>** InnerNodeMatrix, VxSmartInterface<Part>** CornerMassMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxVector3** sumFExtBeforeSwitch, std::vector <double> sumFdotVBeforeSwitch, int numNodes, int NumNodesToAdd);

int constructInnerNodes_partialSwitch(VxSmartInterface<Assembly> assembly, std::string pathData, std::string pathResults, VxSmartInterface<Part>** partMatrix, VxReal** masstMatrixBeforeSwitch, VxReal** masstMatrixAfterSwitch, VxSmartInterface<Part>** InnerNodeMatrix, VxSmartInterface<Part>** CornerMassMatrix, VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxVector3** sumFExtBeforeSwitch, std::vector <double> sumFdotVBeforeSwitch, int numNodes, int NumNodesToAdd);

int changeDistanceJoint(VxSmartInterface<Assembly> assembly, std::string pathData, std::string pathResults, VxSmartInterface<Part>** partMatrix, VxSmartInterface<DistanceJoint> allConstraintsVector[], int numNodes);

#endif