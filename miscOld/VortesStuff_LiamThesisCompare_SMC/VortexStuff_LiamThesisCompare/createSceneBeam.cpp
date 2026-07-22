// Created by Eleonora Botta, September 12, 2016
// Create scene for cantilever beam with parts linked with various constraints.

#include "MyLibraryNet/createScene.h"

#include "MyLibraryNet/createBeam.h"
#include "MyLibraryNet/addCornerMasses.h"
#include "MyLibraryNet/readParameter.h"

#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxSim/VxSmartInterface.h>
#include <VxContent/Scene.h>
#include <VxContent/SceneICD.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/MechanismICD.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>

#include <iostream>
#include <fstream>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxDynamics;
using namespace VxContent;
using namespace VxSim;


VxSim::VxSmartInterface<VxContent::Scene>  createSceneBeam(std::string pathData, VxSmartInterface<Part> partMatrix[], VxSmartInterface<Part> cornerMasses[], VxSmartInterface<DistanceJoint> allConstraintsVector[], VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[])
{
	VxSmartInterface<Scene>  scene = VxSim::VxExtensionFactory::create(SceneICD::kFactoryKey);
    VxSmartInterface<Assembly>  assembly = VxSim::VxExtensionFactory::create(VxDynamics::Assembly::kFactoryKey);

	FILE* FileLog=0;
	FileLog = fopen("FileLog.txt", "w");

	// read data for simulation (gravity)
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt";
	VxReal gravity = readParameter<VxReal>("gravity", simulationDataFileName);
	int CMsPresent = readParameter<int>("CMsPresent", simulationDataFileName);

	float totalMass = 0;
	
	// Create beam
	totalMass = createBeam(assembly, pathData, partMatrix, allConstraintsVector, allConstraintsVectorPrismatic, totalMass, FileLog);

    //VxPart* base = addBase(assembly);
	
	// Disable collision, except with base.
    VxCollisionRule collisionRule(assembly->getVxAssembly(), false);
    //VxCollisionRule collisionRule2(assembly, base, true); 
    assembly->getVxAssembly()->appendCollisionRule(collisionRule);
   // assembly->appendCollisionRule(collisionRule2);

    // Create a mechanism and put the assembly in it.
	VxSmartInterface<Mechanism> mechanism = VxSim::VxExtensionFactory::create(MechanismICD::kFactoryKey);
    mechanism->addAssembly(assembly);

    // Add the mechanism to the scene.
    scene->addMechanism(mechanism);

	// Change gravity
    VxFrame::instance()->getUniverse(0)->setGravity(0,0,gravity); // by default we have gravity.

	fclose(FileLog);

    return scene;
}