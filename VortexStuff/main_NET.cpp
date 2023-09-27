// Created by Eleonora Botta, September 9, 2016
// Modified by Eleonora Botta, September 15, 2016: chaser 
// Modified by Eleonora Botta, September 17, 2016: target
// Modified by Eleonora Botta, September 19, 2016: checks of data
// Modified by Andreas Enzenhöfer, September 21, 2016: extensions for handling graphics
// Modified by Eleonora Botta, September 23, 2016: chaser passed as pointer to createScene to know it at runtime. Cable graphics.
// Modified by Eleonora Botta, September 28, 2016: winch passed as pointer to createScene for main to know it at runtime
// Modified by Eleonora Botta, September 29, 2016: graphics for winch, hinge passed as pointer to createScene for main to know it at runtime, read winch velocity from file.
// Modified by Eleonora Botta, September 30, 2016: change winch velocity, save tether quantities. 
// Modified by Eleonora Botta, October 10, 2016: retrieve generic cable and the dynamics extension associated. 
// Modified by Eleonora Botta, October 12, 2016: function changeVelocity with controls, velocities, accelerations. Take initial conditions from these too. 
// Modified by Eleonora Botta, October27, 2016: Choose if OSG needs to be used, save computational time, save dynamics data just each tsave period. 
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)
// Modified by Eleonora Botta, January 19, 2017: possibility to use a batch file to launch, and give different tether controls
//												 put results in folder Results/ and in folder that depends in the input file, in case.
// Modified by Eleonora Botta, February 13, 2017: read info on rings in main, closed PD control over length of tether
// Modified by Eleonora Botta, February 14, 2017: save tether dynamics info on all tethers
// Modified by Eleonora Botta, February 15, 2017: tethers with 3 rings with CL control
// Modified by Eleonora Botta, February 16, 2017: read from input data for CL control, save distance among rings
// Modified by Eleonora Botta, February 17, 2017: possibility to have 2 tethers configuration, with different initial time for CL control.
// Modified by Eleonora Botta, March 16, 2017: call modifyMaterialTableToAllowForCableCollisions() at the wanted time if netModel == 1 (cable model for net)
// Modified by Eleonora Botta, March 31, 2017:  modified point of view for some cases, to have expected axes in OSG.
//												added code that should change threads EI at a certain time, but does not work. 
// Modified by Eleonora Botta, April 1, 2017: add function to save all masses of the net (and CMs) in a file.
// Modified by Eleonora Botta, April 17, 2017: commented KeyboardHelper, VxSimulatorModuleFactory, FindInterface for Vortex Studio 2017a
// Modified by Eleonora Botta, April 18, 2017:  commented application->beginMainLoop() and application->endMainLoop() for Vortex Studio 2017a
//												substituted obj->findExtensions(...) withobj->findExtensionsByKey(...) for Vortex Studio 2017a
// Modified by Eleonora Botta, June 15, 2017: selected different cameras for different simulations
// Modified by Eleonora Botta, June 16, 2017: change gravity, if target is present and is envisat mockup
// Modified by Eleonora Botta, June 29, 2017: closingMechanismType=1 is among CMs, closingMechanismType=2 goes around net perimeter 

// Modified by Eleonora Botta, April 3 2020: ported to 2019c, added saveDynamicsQuantitiesChaser
// Modified by Eleonora Botta, May 12 2020: if target is drone, set gravity to 9.81 along x-dir (to be changed) and freeze chaser and target - preliminary simulation


#include <VxSim/KeyboardHelpDisplayICD.h>
#include <VxSim/KeyboardNotificationDisplayICD.h>

#include <Vx/VxSmartPtr.h>
#include <VxSim/VxApplication.h>
#include <VxSim/VxDynamicsModuleICD.h>
#include <VxSim/VxSimulationFileManager.h>
#include <VxSim/VxSimulatorModule.h> //was <VxSim/VxSimulatorModuleFactory.h> before
#include <VxSim/VxExtensionFactory.h>
#include <Vx/Find.h> //#include <VxSim/FindInterface.h> // not needed in the new version
#include <VxSim/IKeyboard.h>

#include <VxGraphics/Context.h>
#include <VxGraphics/ICamera.h>
#include <VxGraphics/GraphicsModule.h>
#include <VxGraphics/Viewport.h>
#include <VxGraphicsPlugins/PerspectiveICD.h>
#include <VxGraphicsPlugins/DynamicsVisualizerICD.h>
#include <VxGraphicsPlugins/GraphicsModuleICD_OSG.h>

// EB 24 jan 2020 #include <VxGraphics/SceneGraph.h>
#include <VxGraphics/Services.h>
#include <VxGraphics/ShapeGenerator.h>
#include <VxGraphics/Window.h>
#include <VxGraphics/WindowingSystem.h>
#include <VxGraphics/Window.h>
#include <VxGraphics/Viewport.h>

#include <VxPluginSystem/VxPluginManager.h>

#include <Vx/VxAssembly.h>
#include <Vx/VxBox.h>
#include <Vx/VxSphere.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxPart.h>
#include <Vx/VxTransform.h>
#include <VxSim/VxApplication.h>
#include <VxDynamics/Mechanism.h> // Changed it from Vx/VxMechanism.h, (didn't exist in 2017)

// Include the used constraints
#include <Vx/VxCylindrical.h>
#include <Vx/VxGearRatio.h>
#include <Vx/VxHinge.h>
#include <Vx/VxHomokinetic.h>
#include <Vx/VxPrismatic.h>
#include <Vx/VxScrewJoint.h>
#include <Vx/VxSpring.h>
#include <Vx/VxDistanceJoint.h>
#include <Vx/VxAngular2Position3.h>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxSolverParameters.h>
#include <Vx/VxMaterialTable.h>
#include <VxMath/BoundingSphere.h>

#include <assert.h>
#include <stdio.h>
#include <fstream>
#include <numeric>
#include <vector>
#include <valarray>
#include <ctime>
#include <string>
#include <Windows.h>
#include <Eigen/Eigen/core>
//#include <Shlwapi.h>

// for 6.7
#include <VxMath/Transformation.h>
#include <Vx/VxEulerAngles.h>
#include <Vx/VxMessage.h>
#include <iostream>
#include <VxContent/Scene.h>
#include <VxContent/SceneICD.h>
#include <VxContent/ConnectionContainerExtension.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/MechanismICD.h>
#include <VxDynamics/Assembly.h>
#include <VxDynamics/AssemblyICD.h>
#include <VxDynamics/DistanceJoint.h>
#include <VxDynamics/DistanceJointICD.h>
#include <VxDynamics/Prismatic.h>
#include <VxDynamics/Constraint.h>
#include <VxDynamics/CollisionGeometry.h>
#include <VxDynamics/Sphere.h>
#include <VxDynamics/MassPropertiesContainer.h>
#include <VxDynamics/ConstraintEquationRelaxationContainer.h>
#include <VxDynamics/Hinge.h>

// for cable
#include <CableSystems/DynamicsICD.h>

// for using my own functions
#include "MyLibraryNet/saveDynamicsQuantitiesNet.h"
#include "MyLibraryNet/saveDynamicsQuantitiesTarget.h"
#include "MyLibraryNet/saveDynamicsQuantitiesNodes.h"
#include "MyLibraryNet/saveDynamicsQuantitiesTether.h"
#include "MyLibraryNet/saveDynamicsQuantitiesChaser.h"
#include "MyLibraryNet/SlidingModeChaserAttControl.h"
#include "MyLibraryNet/displayThreadsAndCornerMasses.h"
#include "MyLibraryNet/displayScene.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/createScene.h"
//#include "MyLibraryNet/addTarget.h"
#include "MyLibraryNet/checkData.h"
#include "MyLibraryNet/readArray.h"
#include "MyLibraryNet/changeVelocity.h"
#include "MyLibraryNet/saveContactInformation.h"
#include "MyLibraryNet/controlWinchRatePD.h"
#include "MyLibraryNet/saveRingsDistance.h"
#include "MyLibraryNet/modifyMaterialTable.h"
#include "MyLibraryNet/addStdClosingMechanism.h"
#include "MyLibraryNet/TetherPIDThrustControl.h"
#include "MyLibraryNet/NodeInitialConditions.h"
#include "MyLibraryNet/NodeInitialConditionsFromInput.h"
#include "MyLibraryNet/readTabDelimVector.h"
#include "MyLibraryNet/OrbitalMech2BP.h"
#include "MyLibraryNet/OrbitalMech2BP_Part.h"
#include "MyLibraryNet/TestFunction.h"
#include "MyLibraryNet/matrixMult.h"
#include "MyLibraryNet/OrbitalMech2BP_CoM.h"
#include "MyLibraryNet/PD_Thrust.h"
#include "MyLibraryNet/saveGravityForce.h"



// display 
#include "MyLibraryNet/AccessoryExtension.h"
#include "MyLibraryNet/AccessoryModule.h"

// screenshots
//#include "MyLibraryNet/ExportScreenshots.h"

#include <exception>

#define MAXGRIDSIZE 100

using namespace Vx;
using namespace VxContent;
using namespace VxDynamics;
using namespace VxSim;
using namespace CableSystems::DynamicsICD;

// LF 2/27/23
//#define GRAPHICS // Turn1s graphics on or off
#define SCREENSHOT // Turns screenshots on or off (requires graphics to be defined)
#define GRAPHICS_COM // used for camera in the system center of mass while on orbit

//
//#define GRAVITY // -> Gravitational force applied to each body individually

//#define NodeGravity // -> Needed for the two macros below, have to change matrixMult if numNode != 4
//#define GRAVITYCOM // -> applies the gravitational force associated with the system center of mass to VxUniverse 
//#define GRAVITYCOM2 // -> applies the gravitational force associated with the system center of mass to VxUniverse, then adds the gravitational force of the chaser and target to each other individually 

// This is where the Framework Application is created and the required modules are inserted, and where I call creation of scene, display, save quantities.
int main(int argc, const char* argv[])
{
	int returnValue = 0;

	std::string pathData = "Inputs/MultiTests/Vbar_Correct/";
	//std::string pathData = "Inputs/";
	// check data
	//checkData(pathData);

	// Removed: read data for net 

	// Removed: Find indexes for central part of net, and last part of the net (used in reading ring positions) 

	Vx::VxReal mu = 3.986000 * pow(10,14);
	// read data for chaser
	// Need to add initial Conditions
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt";
	float chaserSideLength = readParameter<float>("chaserSideLength", chaserDataFileName);


	

	// read data for simulation
	std::stringstream simulationDataFileName;
	simulationDataFileName << pathData << "simulationData.txt";
	VxReal finalTime = readParameter<VxReal>("finalTime", simulationDataFileName);
	VxReal tstep = readParameter<VxReal>("tstep", simulationDataFileName);
	VxReal tsave = readParameter<VxReal>("tsave", simulationDataFileName);
	int CMsPresent = readParameter<int>("CMsPresent", simulationDataFileName);
	int chaserPresent = readParameter<int>("chaserPresent", simulationDataFileName);
	int targetPresent = readParameter<int>("targetPresent", simulationDataFileName);
	int tetherPresent = readParameter<int>("tetherPresent", simulationDataFileName);
	int OSGview = readParameter<int>("OSGview", simulationDataFileName);
	

	// Initial Quaternion Data
	std::vector<double> chaserQuat = { 0.170800000000000,
											  -0.556400000000000,
											   0.663700000000000,
											   0.469900000000000 };
	std::vector<double> targetQuat = { 0.296199984506851 ,
									   0.321394676402429,
									  -0.383021490232830,
									   0.813797007353886 };

	
	// read PD control data
	std::array<VxReal, 4> PD_States;
	std::stringstream PDControlDataFilename;
	PDControlDataFilename << pathData << "PDControlData.txt";
	int PDControl = readParameter<int>("PDControl", PDControlDataFilename);
	int Direction = readParameter<int>("Direction", PDControlDataFilename);
	VxReal ThrustConstant = readParameter<VxReal>("ThrustConstant", PDControlDataFilename);
	PD_States[0] = readParameter<VxReal>("DesiredL", PDControlDataFilename);
	PD_States[1] = readParameter<VxReal>("Kp", PDControlDataFilename);
	PD_States[2] = readParameter<VxReal>("Kd", PDControlDataFilename);
	
	// initialize output files. If an input is given, put them in dedicated folders
	std::string pathResults("Results/");
	if (argc > 1)
	{
		std::string tempStr = argv[1];
		pathResults += (tempStr.substr(0, tempStr.size() - 4) + "/");
		std::cout << pathResults << std::endl;
	}
	FILE* outMass = 0;
	FILE* outX = 0;
	FILE* outY = 0;
	FILE* outZ = 0;
	FILE* outVX = 0;
	FILE* outVY = 0;
	FILE* outVZ = 0;
	FILE* outF = 0;
	FILE* outTime = 0;
	FILE* outSolver = 0;
	FILE* outTarget = 0;
	FILE* outTether = 0;
	FILE* outChaser = 0;
	FILE* outTetherPoints = 0;
	FILE* outPID = 0;
	FILE* outSMC = 0;
	FILE* outNodesPos = 0;
	FILE* outNodesVel = 0;
	FILE* outTension = 0;
	FILE* outCenterofMassConditions = 0;
	FILE* outPD = 0;
	FILE* outGravity = 0;

	outMass = fopen((pathResults + "MassVector.txt").c_str(), "w");
	outX = fopen((pathResults + "NodesX.txt").c_str(), "w");
	outY = fopen((pathResults + "NodesY.txt").c_str(), "w");
	outZ = fopen((pathResults + "NodesZ.txt").c_str(), "w");
	outVX = fopen((pathResults + "NodesVX.txt").c_str(), "w");
	outVY = fopen((pathResults + "NodesVY.txt").c_str(), "w");
	outVZ = fopen((pathResults + "NodesVZ.txt").c_str(), "w");
	outF = fopen((pathResults + "ThreadsF.txt").c_str(), "w");
	outTime = fopen((pathResults + "IntegrationTime.txt").c_str(), "w");
	outSolver = fopen((pathResults + "SolverLog.txt").c_str(), "w");
	outTarget = fopen((pathResults + "TargetDynamics.txt").c_str(), "w");
	outTether = fopen((pathResults + "TetherDynamics.txt").c_str(), "w");
	outChaser = fopen((pathResults + "ChaserDynamics.txt").c_str(), "w");
	outTetherPoints = fopen((pathResults + "TetherPoints.txt").c_str(), "w");
	outPID = fopen((pathResults + "PID.txt").c_str(), "w");
	outSMC = fopen((pathResults + "SMC.txt").c_str(), "w");
	outNodesPos = fopen((pathResults + "NodesPos.txt").c_str(), "w");
	outNodesVel = fopen((pathResults + "NodesVel.txt").c_str(), "w");
	outTension = fopen((pathResults + "Tension.txt").c_str(), "w");
	outCenterofMassConditions = fopen((pathResults + "CoMCond.txt").c_str(), "w");
	outPD = fopen((pathResults + "outPD.txt").c_str(), "w");
	outGravity = fopen((pathResults + "GravityForce.txt").c_str(), "w");

	try
	{
		// Instantiate the Vortex application.
		Vx::VxSmartPtr<VxSim::VxApplication> application = new VxSim::VxApplication;
		//Instantiate a Framework dynamics module add it to the application.
		VxSim::VxSmartInterface<VxSim::ISimulatorModule> dynamicsModule = VxSim::VxExtensionFactory::create(VxSim::VxDynamicsModuleICD::kFactoryKey);
		application->insertModule(dynamicsModule);

		VxPluginSystem::VxPluginManager::instance()->load("VxCycloneOSG");
		VxSim::VxSmartInterface<VxGraphics::GraphicsModule> graphicModule = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::GraphicsModuleICD::kModuleFactoryKey);
		application->insertModule(graphicModule);

		// Create a default camera for the Graphic module
		VxSim::VxSmartInterface<VxGraphics::ICamera> freeCamera = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::PerspectiveICD::kExtensionFactoryKey);

		// Removed: if else statement


		//freeCamera->lookAt(Vx::VxVector3(15, 50, 0.0), Vx::VxVector3(15, 0.0, 0.0), Vx::VxVector3(0.0, 0.0, 1.0));

		//freeCamera->lookAt(Vx::VxVector3(30, 0, 10), Vx::VxVector3(31, 0.0, 0.0), Vx::VxVector3(0.0, 0.0, 1.0));
		freeCamera->lookAt(Vx::VxVector3(16.25, 45, 5), Vx::VxVector3(17, 0.0, 0.0), Vx::VxVector3(0.0, 0.0, 1.0));
		application->add(freeCamera);


#ifdef GRAPHICS
		// Set the free camera as the active camera using the interface specialised for the VxGraphicModule
		VxGraphics::Setup windowSetup = graphicModule->createDefaultWindow();
		// EB 24 jan 2020 - For 2017c:
		/*windowSetup.mContext->setPlacement(700, 100, 900, 900);
		windowSetup.mViewport->setClearColor(Vx::VxColor(0.0, 0.0, 0.0, 1.0));
		windowSetup.mViewport->setPolygonMode(VxGraphics::PolygonMode_Solid);
		windowSetup.mViewport->setPlacement(0, 0, 900, 900);*/
		windowSetup.mViewport->setCamera(freeCamera);

		//get utility class for creating graphic shapes
		/**/
		VxGraphics::ShapeGenerator* mShapeGenerator;
		// EB 24 jan 2020 mShapeGenerator = &graphicModule->getSceneGraph()->getServices().getShapeGenerator();
		mShapeGenerator->FAST_SHAPE;
		mShapeGenerator->WIREFRAME_PASS;
		// Instantiate the DynamicsVisualizer if you want to view the physics associated with various parts that have no graphics.
		/* EB 24 jan 2020
		VxPluginSystem::VxPluginManager::instance()->load("DynamicsVisualizer");
		VxSim::VxExtension* dynamicsVisualizer = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DynamicsVisualizerICD::kExtensionFactoryKey);
		//dynamicsVisualizer->getInput(VxGraphic sPlugins::DynamicsVisualizerICD::kDisplayCollisionGeometry)->setValue(true);
		//dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayConstraint)->setValue(true);
		application->add(dynamicsVisualizer);
		*/

		// Instantiate the DynamicsVisualizer if you want to view the physics associated with various parts that have no graphics.   
		VxPluginSystem::VxPluginManager::instance()->load("DynamicsVisualizer");
		auto dynamicsVisualizer = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DynamicsVisualizerICD::kExtensionFactoryKey);
		dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayCollisionGeometry)->setValue(true);
		application->add(dynamicsVisualizer);
#endif

	

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// Removed: Add scene with target - if present (check done inside) (added into display scene) 03/14/22
		const int numNode = 1;

		int numNodeDraw = numNode;
		int TetherType = 2; //type 2: sub tether

		// Instantiate the CableSystemDynamics to create cable
		VxPluginSystem::VxPluginManager::instance()->load("CableSystemsDynamics");

		// Create all the parts and the constraint that need to be accessed or displayed graphically at runtime
		VxSmartInterface<Part>* partVector = new VxSmartInterface<Part>[numNode+2];
		/*for (int i = 0; i < numNode+1; i++)
		{
			partVector[i][i] = new VxSmartInterface<Part>;
		}*/
		VxSmartInterface<DistanceJoint> allConstraintsVector[numNode + 1];
		VxSmartInterface<Part> chaserPart = VxExtensionFactory::create(PartICD::kFactoryKey);
		VxSmartInterface<Part>* chaserPartPtr = &chaserPart;
		VxSmartInterface<Part> tetheredTargetPart = VxExtensionFactory::create(PartICD::kFactoryKey);
		VxSmartInterface<Part>* tetheredTargetPartPtr = &tetheredTargetPart;

		
		//std::cout << "The position of the sphere of index " << i << "," << j << " is " << pos.x() << ", " << pos.y() << ", " << pos.z() << " m" << '\n'; 
		

		

		//VxSmartInterface<DistanceJoint> allConstraintsVector[2 * MAXGRIDSIZE * (MAXGRIDSIZE - 1) + 4]; // all distance joints
		//VxSmartInterface<Prismatic> allConstraintsVectorPrismatic[2 * MAXGRIDSIZE * (MAXGRIDSIZE - 1) + 4]; // all prismatic joints

		// create the scene with net, corner masses, chaser, tether - then add it to the application
		VxSmartInterface<Scene>  scene = createScene(pathData, partVector, chaserPartPtr, tetheredTargetPartPtr, numNode, TetherType, allConstraintsVector);
		application->add(scene);
		std::cout << "added scene to application" << std::endl;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// For display of graphics: this will call directly displayScene at every time step
		if (OSGview == 1)
		{
			// register module: add accessories to simulation at every time step (nothing dynamic): create and register key(for extensions and accessories)
			VxSim::VxFactoryKey kAccessoryModuleKey(VxSim::VxUuid("{97cdb6b5-619e-4ed2-b418-ea26785c2172}"), "Custom", "AccessoryModule"); // random UID
			VxSim::VxExtensionFactory::registerType<AccessoryModule>(kAccessoryModuleKey);

			// create and insert module: in module, functions called at every time step
			VxSim::VxSmartInterface<VxSim::ISimulatorModule> module = VxSim::VxExtensionFactory::create(kAccessoryModuleKey);
			application->insertModule(module);

			// register extension: to draw at every time step
			VxSim::VxFactoryKey kAccessoryExtensionKey(VxSim::VxUuid("{b6d7b392-d8e1-4fa6-b08b-e7ab8638a7b3}"), "Custom", "AccessoryExtension");
			VxSim::VxExtensionFactory::registerType<AccessoryExtension>(kAccessoryExtensionKey);

			// create and insert extension
			VxSim::VxSmartInterface<AccessoryExtension> accessoryExtension = VxSim::VxExtensionFactory::create(kAccessoryExtensionKey);
			application->add(accessoryExtension);
			//accessoryExtension->setConfiguration(partMatrix, cornerMasses, &gNetSize, &CMsPresent); // Andreas: if you don't use pointers it doesn't update
			//accessoryExtension->setConfiguration(partMatrix, cornerMasses, &gNetSize, &CMsPresent, &chaserPresent, chaserPartPtr, &chaserSideLength, winchPartPtr, &winchRadius, &netModel);
			//accessoryExtension->setConfiguration(partVector, chaserPartPtr, &chaserSideLength, tetheredTargetPartPtr);//AB
			accessoryExtension->setConfiguration(partVector, chaserPartPtr, &chaserSideLength, tetheredTargetPartPtr, &numNodeDraw, &TetherType);
			std::cout << "Initialized OSG" << std::endl;

			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			// Find the first CableSystem dynamics extension, to retrieve its properties later on
		/*	VxSmartInterface<IExtension> dynamicsCableExtension;
			size_t mechanismCount = scene->getMechanisms().size();
			std::cout << "\n \n the size of scene->getMechanisms() is " << mechanismCount << std::endl << std::endl;
			for (size_t mechanismIndex = 0; mechanismIndex < mechanismCount; ++mechanismIndex)
			{
				VxSmartInterface<Mechanism>  mechanism = scene->getMechanisms()[mechanismIndex];
				if (mechanism.valid())
				{
					size_t extCount = mechanism->getExtensions().size();
					std::cout << "\n \n the size of mechanism->getExtensions() is " << extCount << std::endl << std::endl;
					for (size_t extIndex = 0; extIndex < extCount; ++extIndex)
					{
						VxSmartInterface<IExtension> ext = mechanism->getExtensions()[extIndex];
						if (ext.valid())
						{
							if (ext->getProxy()->getCreationKey() == CableSystemsICD::kGenericCableKey)
							{
								std::cout << "\n \n I found the generic cable " << std::endl << std::endl;
								Vx::VxSmartPtr<VxObject> obj = ext->getProxy()->toObject();
								Vx::VxArray<VxExtension*> childExtensions = obj->findExtensionsByKey(CableSystemsICD::kDynamicsGenericCableKey);
								if (childExtensions.size() == 1)
								{
									dynamicsCableExtension = childExtensions[0];
								}
							}
						}
					}
				}
			}*/

			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			VxSolverParameters* solverPar = VxFrame::instance()->getUniverse(0)->getSolverParameters(0);
			unsigned int solverType = solverPar->getConstraintSolver();

			// Decide solver
			fprintf(outSolver, "The solver type is %d \n", solverType);
			/*solverPar->setConstraintSolver(0);
			solverPar->setConstraintStdSolverTolerance(0.01);
			bool partEnabled = solverPar->isPartitionSplittingEnabled();*/
			fprintf(outSolver, "The solver tolerance is %g \n", solverPar->getConstraintStdSolverTolerance());
			fprintf(outSolver, "The solver max iteration is %g \n", solverPar->getConstraintStdSolverMaxIteration());
			//Vx::VxSolverParameters::setConstraintSolver(solverPar);
			fprintf(outSolver, "The integrator time step is %g \n", VxFrame::instance()->getTimeStep());
			//VxFrame::instance()->setTimeStep(tstep); 
			application->setSimulationFrameRate(1 / tstep);
			fprintf(outSolver, "The integrator time step is %g \n", VxFrame::instance()->getTimeStep());
			std::cout << "Solver setup done" << std::endl;

			// Set initial conditions
			int countSteps = 0;
			VxReal currentTime = 0;
			fprintf(outTime, "%f \n", currentTime);

			//Removed: Save initial conditions

			auto targetInertia = (*tetheredTargetPartPtr)->getVxPart()->getMassProperties();
			//std::cout << targetInertia() << std::endl;
			//Removed: Save masses of net and corner masses

			
			VxFrame::instance()->getUniverse(0)->setGravity(0, 0, 0);
			/*Vx::VxVector3 chaserPos = Vx::VxVector3(0, 0, 0);
			Vx::VxVector3 targetPos = (0, 0, 0);
			Vx::VxVector3 chaserVel = (0, 0, 0);
			Vx::VxVector3 targetVel = (0, 0, 0);
			std::vector<double> chaserQuat = { 1, 0.0, 0.0, 0.0 };
			std::vector<double> targetQuat = { 1, 0.0, 0.0, 0.0 };
			Vx::VxVector3 chaserAngRate = (0, 0, 0);
			Vx::VxVector3 targetAngRate = (0, 0, 0);*/

//
			//VxSmartInterface<Mechanism> mech = scene->getMechanisms()[0];

			//VxSmartInterface<Assembly> ass = mech->getAssemblies()[0];
			//Vx::VxAssembly* Assem = ass->getVxAssembly();
			//Vx::VxConstraintSet Constrs = Assem->getConstraints()[0];
			//Vx::VxAssembly* ass = scene->getMechanisms()[0]->getAssemblies()[0];
			
			//VxSim::VxSmartInterface<DistanceJoint> v = ass->getConstraints()[0];

			

			// Initial Quaternions
			//std::vector<double> chaserQuat = { 1,0,0,0};
			

			Vx::VxQuaternion TargetQ;    
			(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);

			const double PI = 3.141592653589793238463;
			

			// Extracting attachment point information for later use, LF
			std::stringstream chaserDataFileName;
			chaserDataFileName << pathData << "chaserData.txt";
			VxReal chaserAttX = readParameter<VxReal>("chaserAttX", chaserDataFileName);
			VxReal chaserAttY = readParameter<VxReal>("chaserAttY", chaserDataFileName);
			VxReal chaserAttZ = readParameter<VxReal>("chaserAttZ", chaserDataFileName);

			VxVector3 chaserAtt = VxVector3(chaserAttX, chaserAttY, chaserAttZ);
			
			std::stringstream targetDataFileName;
			targetDataFileName << pathData << "targetData.txt";
			VxReal targetAttX = readParameter<VxReal>("targetAttX", targetDataFileName);
			VxReal targetAttY = readParameter<VxReal>("targetAttY", targetDataFileName);
			VxReal targetAttZ = readParameter<VxReal>("targetAttZ", targetDataFileName);

			VxVector3 targetAtt = VxVector3(targetAttX, targetAttY, targetAttZ);

			//Vx::VxQuaternion TargetQ1;
			//(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ1);

			//VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
			//const Vx::VxArray<VxSim::VxSmartInterface<VxDynamics::Constraint>> v = Assem->getConstraints();
			//VxSim::VxSmartInterface<VxDynamics::DistanceJoint> vG = v[0];
			
			//AB turn off
			//// Node Initial Conditions
			//if (numNode > 0)
			//{
			//	NodeInitialConditionsFromInput(chaserPartPtr, tetheredTargetPartPtr, partVector, numNode,pathData);
			//}

			//Vx::VxVector3 chaserAng = (*chaserPartPtr)->getVxPart()->getAngularVelocity();

			/*Vx::VxMassProperties MC;
			MC = (*tetheredTargetPartPtr)->getVxPart()->getMassProperties();
			Vx::VxInertiaTensor H;
			MC.getInertiaTensorLocal(H);*/
			/*double L = H(0, 0);*/
			//Vx::VxArray<VxSim::VxSmartInterface<Constraint>> Vfg = ass->getConstraints();
			//Vx::VxQuaternion quat;
			//(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(quat);
			//chaserPart->getVxPart()->getOrientationQuaternion(quat);

					// Run the simulation
			Vx::VxVector3 chaserP = (*chaserPartPtr)->getVxPart()->getPosition();
			Vx::VxVector3 targetP = (*tetheredTargetPartPtr)->getVxPart()->getPosition();

			// float theta = acos(chaserP.dot(targetP) / (chaserP.norm() * targetP.norm()));
			//VxVector3 NodeP = partVector[0]->getVxPart()->getPosition();

			// Saving Intitial Conditions
			saveDynamicsQuantitiesChaser(chaserPart, outChaser);
			saveDynamicsQuantitiesChaser(tetheredTargetPart, outTarget);
			if (numNode > 0)
			{
				saveDynamicsQuantitiesNodes(partVector, outNodesPos, outNodesVel, outTension, numNode);
			}




			/*VxQuaternion ChaserQ;
			(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
			VxVector3 HHH = ChaserQ.rotate(chaserAtt);

			Vx::VxQuaternion TargetQ1;
			(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ1);
			VxVector3 HHHT = TargetQ1.rotate(targetAtt);
			VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
			VxVector3 targetAttPos = TargetQ1.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();

			VxVector3 L = targetAttPos - chaserAttPos;
			double Lnorm = L.norm();

			auto targetAttRot = TargetQ1.rotate(targetAtt);
			auto AlignAngle = acos(L.dot(targetAttRot) / (L.norm() * targetAttRot.norm()));

			auto g = 1;*/






			double intError = 0;
			double Error = 0;
			double* errorPtr = &Error;
			//VxMath::VxVector4 ThrustPlusError = TetherPIDThrustControl(chaserPartPtr, tetheredTargetPartPtr, VxVector3(300, 2000, 300), double(.01), intError, numNode, outPID);


			int nStepsSave = (int)(tsave / tstep);
			clock_t simulationTime;
			simulationTime = clock();
			std::cout << "I am beginning to run... " << std::endl;
			//VxVector3 Mass1 = partMatrix[0][0]->getVxPart()->getPosition();
			// (application->update()); //<- Breaking due to this


			// LF changes first gravity step based on CoM position 2/27/23

				//AB turn off for ST test
			//(*chaserPartPtr)->getVxPart()->setControl(Vx::VxPart::kControlStatic);
			//(*tetheredTargetPartPtr)->getVxPart()->setControl(Vx::VxPart::kControlStatic);

#ifdef NodeGravity
			VxFrame::instance()->getUniverse(0)->setGravity(OrbitalMech2BP_CoM(chaserPartPtr,tetheredTargetPartPtr,partVector,mu,numNode));
			VxVector3 CoM_Pos;
			VxVector3 CoM_Grav;
#endif
			while ((application->update()) && (currentTime < finalTime))
			{
				//// change tstep after capture
				//if ( (currentTime > 5.0) && (changeStepFlag == 0) )
				//{
				//	changeStepFlag == 1;
				//	tstep = 0.0001;
				//	nStepsSave = (int) (tsave/tstep) ;
				//	countSteps == 0;
				//}



				// update time
				countSteps++;
				currentTime = currentTime + tstep;
				//std::cout << currentTime << std::endl;



				
				VxSmartInterface<IExtension> dynamicsCableExtension;
				if (targetPresent == 1)
					//saveDynamicsQuantitiesTarget(tetheredTargetPart, outTarget);
					if (tetherPresent == 1)
					{
						
						size_t mechanismCount = scene->getMechanisms().size();
						for (size_t mechanismIndex = 0; mechanismIndex < mechanismCount; ++mechanismIndex)
						{
							VxSmartInterface<Mechanism>  mechanism = scene->getMechanisms()[mechanismIndex];
							if (mechanism.valid())
							{
								size_t extCount = mechanism->getExtensions().size();
								for (size_t extIndex = 0; extIndex < extCount; ++extIndex)
								{
									VxSmartInterface<IExtension> ext = mechanism->getExtensions()[extIndex];
									if (ext.valid())
									{
										if (ext->getProxy()->getCreationKey() == CableSystemsICD::kGenericCableKey)
										{
											Vx::VxSmartPtr<VxObject> obj = ext->getProxy()->toObject();
											Vx::VxArray<VxExtension*> childExtensions = obj->findExtensionsByKey(CableSystemsICD::kDynamicsGenericCableKey);
											if (childExtensions.size() == 1)
											{
												dynamicsCableExtension = childExtensions[0];
												VxReal cableLength = dynamicsCableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kTotalLengthID)->toReal();
												//VxReal Elong = dynamicsCableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kElongationID)->toReal();
												
											}
										}
									}
									fprintf(outTether, "%d \t", extIndex);
									saveDynamicsQuantitiesTether(dynamicsCableExtension, outTether);

								}
							}
						}
					}

				/*VxQuaternion ChaserQ;
				(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
				VxVector3 HHH = ChaserQ.rotate(chaserAtt);
				
				Vx::VxQuaternion TargetQ;
				(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);
				VxVector3 HHHT = TargetQ.rotate(targetAtt);
				VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
				VxVector3 targetAttPos = TargetQ.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();

				VxVector3 L = targetAttPos - chaserAttPos;
				double Lnorm = L.norm();
				auto g = 1;*/

				/*VxVector3 chaserAttPos2 = ChaserQ.getInverse().rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
				VxVector3 targetAttPos2 = TargetQ.getInverse().rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();
				VxVector3 L2 = targetAttPos - chaserAttPos;
				double Lnorm2 = L2.norm();
				auto g2 = 1;*/

				//Attempts to extract attachment points from cable extension (Failure), LF
				//auto Out = dynamicsCableExtension->getProxy()->getOutput(PointDefinitionContainerID::kOffsetID);
				//auto Out1 = dynamicsCableExtension->getProxy()->getOutput(PointDefinitionContainerID::kOffsetID)->toReal();
				
	

				//Testing different ways to acquire assembly constraints, LF
				//VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
				//auto v = Assem->getConstraints();
				//int S = v.size();
				//auto Mech = Assem->getParentMechanism();
	
				//auto Assems = VxFrame::instance()->getUniverse(0)->getAssemblies();
				////auto Mech = scene->getMechanisms();
				////auto S = Mech.;
				//auto Ss = Assems.size();
				//auto assemblyiter = Assems.begin();
				//auto assemblyiter1 = Assems.end();
				//VxAssembly* someassembly = *assemblyiter1;
				//const VxConstraintSet& constraintsinassembly = someassembly->getConstraints();
				//int sSs = constraintsinassembly.size();

			




				if (countSteps % nStepsSave == 0)
				{
					if (tetherPresent == 1) {
						//	saveDynamicsQuantitiesTether(dynamicsCableExtension, outTether, hingeForWinchPtr, currentWinchSpoolingVelocity);
					}
					// saveDynamicsQuantitiesNet(netModel, partMatrix, cornerMasses, allConstraintsVector, outX, outY, outZ, outVX, outVY, outVZ, outF, NetSize, CMsPresent);

					fprintf(outTime, "%f \n", currentTime);
				}

				if (countSteps % (int(0.1 / tstep)) == 0)
				{
					std::cout << currentTime << std::endl;
				}

				//VxSmartInterface<Assembly> ass = scene->getMechanisms()[1]->getAssemblies()[1];


				//partVector[0]->getVxPart()->getPosition();

				// LF: Here down is for application of gravity comparison
				double chaserMass = (*chaserPartPtr)->getVxPart()->getMass();
				double targetMass = (*tetheredTargetPartPtr)->getVxPart()->getMass();

				//AB test
				Vx::VxVector3 chaserP = (*chaserPartPtr)->getVxPart()->getPosition();
				//std::cout << chaserP[0] << std::endl;
				Vx::VxVector3 targetP = (*tetheredTargetPartPtr)->getVxPart()->getPosition();
				//std::cout << targetP[0] << std::endl;
				VxReal NORMPOSITIONS = (chaserP - targetP).norm();


 			//	VxPartSet PartsUniverse = VxFrame::instance()->getUniverse(0)->getParts();
				//int iterCount = 0;
				//for (auto iter = PartsUniverse.begin(), end = PartsUniverse.end(); iter != end; ++iter)
				//{
				//	size_t indexOut = 0;
				//	VxPart* part1 = *iter;
				//	VxPart* part2 = dynamic_cast<VxPart*>(part1);
				//	std::string needName = part2->getName();
				//	indexOut = needName.find("Dynamics Generic Cable_SegmentFlex", 0);
				//	if (indexOut == 0)
				//	{
				//		//std::cout << "\n \n The part name is " << needName << std::endl << std::endl;
				//		if (Vx::VxPart::kControlDynamic == part2->getControl())
				//		{
				//			if (abs(part2->getPosition().norm() - targetP.norm()) < 1)
				//			{


				//				//std::cout << "\n  The part name is " << needName  << std::endl;
				//				//std::cout << "\n  The position difference is " << part2->getPosition().norm() - targetP.norm() << std::endl << std::endl;
				//
				//			
				//			}
				//			iterCount += 1;
				//			//std::cout << "\n YESSSSSSSS" << std::endl;
				//		}


				//	}
				//	//if ((index_string = needName.) == 0)

				//	//"Dynamics Generic Cable_SegmentFlex"


				//}


				// For adding Gravity using VxUniverse of the center of mass of the system
#ifdef NodeGravity				
				double PosVec[3][2 + numNode];
				std::vector < double > MassVec;


				
				double totalMass = 0.0;

				if (numNode > 0) 
				{

					


					for (int i = 0; i < 3; i++)
					{
						PosVec[i][0] = chaserP[i];
						PosVec[i][1] = targetP[i];
						
					}



					MassVec.push_back(chaserMass);
					MassVec.push_back(targetMass);
				
					for (int i = 0; i < numNode; i++)
					{ 
						MassVec.push_back(partVector[i]->getVxPart()->getMass());
						
						for (int j = 0; j < 3; j++)
						{
							PosVec[j][i+2] = partVector[i]->getVxPart()->getPosition()[j];
						}
					
					}

					//auto h = std::sizeof(PosVec);
					auto C = matrixMult(PosVec,MassVec);
					/*VxVector3 CoM_Pos = VxVector3(C[0], C[1], C[2])*/;
					
					for (int j = 0; j < 6; j++)
					{
						totalMass += MassVec[j];
					}
					
					CoM_Pos = VxVector3(C[0], C[1], C[2]).operator/(totalMass);
					CoM_Grav = -mu *CoM_Pos / pow(CoM_Pos.norm(),3);
//#endif
					

#ifdef GRAVITYCOM
					//auto GravCheck = VxFrame::instance()->getUniverse(0)->getGravity();
					VxFrame::instance()->getUniverse(0)->setGravity(CoM_Grav);
#endif



#ifdef GRAVITYCOM2
					VxFrame::instance()->getUniverse(0)->setGravity(CoM_Grav);

					(*chaserPartPtr)->getVxPart()->addForce(OrbitalMech2BP(chaserPartPtr, mu)-CoM_Grav*chaserMass);

					// target
					(*tetheredTargetPartPtr)->getVxPart()->addForce(OrbitalMech2BP(tetheredTargetPartPtr, mu)-CoM_Grav*targetMass);

					// nodes/tether
					if (TetherType == 1)
					{
						if (numNode > 0)
						{
							for (int i = 0; i < numNode - 1; i++)
							{
								VxSmartInterface<Part> p = partVector[i];
								VxSmartInterface<Part>* pstar = &p;
								partVector[i]->getVxPart()->addForce(CoM_Grav*partVector[i]->getVxPart()->getMass());


							}
						}
					}
#endif
				}	
#endif

					 




				// To add: 
				// - cable gravity
				
				
				//////////////// Orbital Mechanics -> Newton Gravity 2-body
#ifdef GRAVITY



				// chaser
				VxVector3 chsGrav = OrbitalMech2BP(chaserPartPtr, mu);
				(*chaserPartPtr)->getVxPart()->addForce(chsGrav);

				// target
				VxVector3 tgtGrav = OrbitalMech2BP(tetheredTargetPartPtr, mu);
				(*tetheredTargetPartPtr)->getVxPart()->addForce(tgtGrav);
				

				
				// nodes/tether
				if (TetherType == 1)
				{
					if (numNode > 0)
					{
						for (int i = 0; i < numNode - 1; i++)
						{
							VxSmartInterface<Part> p = partVector[i];
							VxSmartInterface<Part>* pstar = &p;
							
							partVector[i]->getVxPart()->addForce(OrbitalMech2BP(pstar, mu));
							partVector[i]->getVxPart()->addForce(chsGrav);


						}
					}
				}
				else
				{

					VxPartSet PartsUniverse = VxFrame::instance()->getUniverse(0)->getParts();

					for (auto iter = PartsUniverse.begin(), end = PartsUniverse.end(); iter != end; ++iter)
					{
						size_t indexOut = 0;
						VxPart* part1 = *iter;
						VxPart* part2 = dynamic_cast<VxPart*>(part1);
						std::string needName = part2->getName();
						indexOut = needName.find("Dynamics Generic Cable_SegmentFlex", 0);
						//std::cout << "\n \n The part name is " << needName << std::endl << std::endl;
						if (indexOut == 0)
						{
							//std::cout << "\n \n The part name is " << needName << std::endl << std::endl;
							if (Vx::VxPart::kControlDynamic == part2->getControl())
							{
								part2->addForce(OrbitalMech2BP(part2, mu));
							}


						}
						//if ((index_string = needName.) == 0)

						//"Dynamics Generic Cable_SegmentFlex"


					}


				//	TestFunction(dynamicsCableExtension,mu);
				}
				/// ////////////
#endif 
				
				/// //////////// Camera
#ifdef GRAPHICS
#ifdef GRAPHICS_COM
				Vx::VxVector3 CenterOfMass_System = (chaserMass * chaserP + targetMass * targetP) / (targetMass + chaserMass);
				Vx::VxReal NCenterOfMass_System = CenterOfMass_System.norm();
				Vx::VxVector3 CoMUV = CenterOfMass_System / NCenterOfMass_System;
#endif
#endif

				// CLOSED LOOP PD CONTROL
				VxVector3 InputThrust;
				std::vector<double> outputPD;
				Vx::VxVector3 chaserVel = (*chaserPartPtr)->getVxPart()->getLinearVelocity();
				if (PDControl == 1) {
					outputPD = PD_Thrust(tetheredTargetPartPtr, chaserPartPtr, PD_States, allConstraintsVector, Direction, numNode);
					InputThrust = VxVector3(outputPD[2], outputPD[3], outputPD[4]);
					
				}
				else
				{
					// OPEN LOOP CONTROL 
					////VxQuaternion ChaserQ;
					////(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
					////VxVector3 HHH = ChaserQ.rotate(chaserAtt);

					////Vx::VxQuaternion TargetQ1;
					////(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ1);
					////VxVector3 HHHT = TargetQ1.rotate(targetAtt);
					////VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
					////VxVector3 targetAttPos = TargetQ1.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();

					////VxVector3 L = targetAttPos - chaserAttPos;
					////double Lnorm = L.norm();

					////InputThrust = -ThrustConstant * L / (L.norm());
					//////(*chaserPartPtr)->getVxPart()->addForce(-ThrustConstant * L / (L.norm()));

					//////InputThrust = -5 * L / (L.norm());

					/////*(*chaserPartPtr)->getVxPart()->addForce(-5*chaserVel/(chaserVel.norm()));

					////InputThrust = -5 * chaserVel / (chaserVel.norm());*/

				}

				//AB, for Sys ID
				InputThrust = VxVector3(-500.0, 0.0, 0.0);
				(*chaserPartPtr)->getVxPart()->addForce(InputThrust);
				//(*tetheredTargetPartPtr)->getVxPart()->addForce(InputThrust);

				//VxFrame::instance()->getUniverse(0)->setGravity(VxVector3(0.0, 0.0,-9.810));
				//Vx::VxVector3 Eye = CenterOfMass_System + 60 * CoMUV;
				/*Vx::VxVector3 Eye = chaserP + 60 * CoMUV;
				Vx::VxVector3 Center = CenterOfMass_System;
				Vx::VxVector3 Up = chaserP.cross(chaserVel);
				freeCamera->lookAt(Eye, Center, Up);*/

				//freeCamera->lookAt(chaserP, chaserP, VxVector3(0, 0, 1));
				//freeCamera->lookAt(VxVector3(0, 50, 1), VxVector3(0, 0, 0), VxVector3(0, 0, 1));
				//VxVector3 CoMDist = targetP - chaserP;

				//	fprintf(outTetherPoints,"%f \t", CoMDist.norm());


				/// //////////// Controls
				// chaser attitude, output commented out for now

				//AB, comment out for test
				//VxVector3 ChaserTorqueInput = SlidingModeChaserAttControl(chaserPartPtr, tetheredTargetPartPtr, VxVector3(1, 1, 1), VxVector3(4, 4, 4), VxVector3(0, 0, 0), double(.001), outSMC);
				//(*chaserPartPtr)->addTorque(ChaserTorqueInput);

				// tether length control
				/*if (currentTime == tstep)
				{
					VxMath::VxVector4 ThrustPlusError = TetherPIDThrustControl(chaserPartPtr, tetheredTargetPartPtr, VxVector3(300, 2000, 300), double(.01), intError, numNode,outPID);
					(*chaserPartPtr)->getVxPart()->addForce(ThrustPlusError.xyz());
					(*errorPtr) = intError + ThrustPlusError.w() * tstep;
				}
				else 
				{
					VxMath::VxVector4 ThrustPlusError = TetherPIDThrustControl(chaserPartPtr, tetheredTargetPartPtr, VxVector3(300, 2000, 300), double(.01), Error, numNode, outPID);
					(*chaserPartPtr)->getVxPart()->addForce(ThrustPlusError.xyz());
					(*errorPtr) = Error + ThrustPlusError.w() * tstep;

				
				}*/
				/// ////////////

#ifdef GRAPHICS
#ifdef GRAPHICS_COM

				//Vx::VxVector3 Eye = CenterOfMass_System + 60 * CoMUV;//
				//Vx::VxVector3 Eye = CenterOfMass_System + 60 * CoMUV;//AB sub tet
				//Eye = VxVector3(Eye[0], Eye[2], Eye[1]);//AB sub tet
				//Vx::VxVector3 Center = CenterOfMass_System- VxVector3(10, 0.0, 0.0);
				//Vx::VxVector3 Up = chaserP.cross(chaserVel);
				//freeCamera->lookAt(Eye, Center, Up);

				freeCamera->lookAt(Vx::VxVector3(0.0, -120, -8), Vx::VxVector3(0.0, 0.0, -12), Vx::VxVector3(0.0, 0.0, 1.0));//AB, for Sens

#endif
#endif
				//VxReal Angle123 = std::acos((Center-Eye).dot(chaserP-Eye)/((Center - Eye).norm() * (chaserP - Eye).norm()));


				//freeCamera->setTargetLocked(chaserPartPtr);

				// save dynamics quantities and contact information at every saving time step
				if (countSteps % nStepsSave == 0)
				{
					if (PDControl == 1) {
					
						fprintf(outPD, "%1.15f \t %1.15f \t %1.15f \t %1.15f \t %1.15f", InputThrust.x(), InputThrust.y(), InputThrust.z(), outputPD[0], outputPD[1]);
						fprintf(outPD, "\n");
					}
					else
					{
						fprintf(outPD, "%1.15f \t %1.15f \t %1.15f", InputThrust.x(), InputThrust.y(), InputThrust.z());
						fprintf(outPD, "\n");
					}

					if (numNode > 0)
					{
						saveDynamicsQuantitiesNodes(partVector, outNodesPos, outNodesVel, outTension, numNode);
					}
#ifdef Gravity

						saveGravityForce(chsGrav, tgtGrav, outGravity);
#endif

						saveDynamicsQuantitiesChaser(chaserPart, outChaser);
						saveDynamicsQuantitiesChaser(tetheredTargetPart, outTarget);
						if (numNode > 0)
						{
						/*	saveDynamicsQuantitiesNodes(partVector, outNodesPos, outNodesVel, outTension,numNode);*/
							
							//Printing out system center of mass coordinates and gravity acceleration LF 2/27/23 
							
#ifdef NodeGravity
							fprintf(outCenterofMassConditions, "%f \t %f \t %f \t %f \t %f \t %f \n", CoM_Pos.x(), CoM_Pos.y(), CoM_Pos.z(), totalMass*CoM_Grav.x(), totalMass * CoM_Grav.y(), totalMass * CoM_Grav.z());
#endif

						}
						else 
						{
							(*allConstraintsVector)[0].getVxConstraint()->getConstraintEquationForce(0);
							fprintf(outTension, "%f \n", (*allConstraintsVector)[0].getVxConstraint()->getConstraintEquationForce(0));
						}
						
					

#ifdef GRAPHICS					
					std::stringstream st;
					st << currentTime;
					//Vx::VxFilename screenshot_name = pathResults + "Screenshots/" + "t=" + st.str() + "s.png"; // EB 24 jan 2020 : this was for 2017c
					// windowSetup.mContext->takeScreenshot(screenshot_name); // EB 24 jan 2020 : this was for 2017c
					std::string screenshot_name = pathResults + "Screenshots/" + "t=" + st.str() + "s.png";

					VxGraphics::Window* window = VxGraphics::findFirstWindowForViewport(windowSetup.mViewport.get());
#ifdef SCREENSHOT					
					if (window)
					{

						window->takeScreenshot(screenshot_name);
						window->swapBuffers();
						//++mFrameCount;
					}
#endif

#endif

				}

			}
			//application->endMainLoop();



			// save computational time
			simulationTime = clock() - simulationTime;
			fprintf(outSolver, "\nIt took %g seconds to integrate. \n", ((float)simulationTime) / CLOCKS_PER_SEC);
			//fprintf(outTime,)

		}
	}
		catch (const std::exception& ex)
		{
			std::cout << "Error : " << ex.what() << std::endl;
			returnValue = 1;
		}
		catch (...)
		{
			Vx::LogWarn("Got an unhandled exception, application will exit!\n");
			returnValue = 1;
		}


		fclose(outMass);
		fclose(outX);
		fclose(outY);
		fclose(outZ);
		fclose(outVX);
		fclose(outVY);
		fclose(outVZ);
		fclose(outSolver);
		fclose(outF);
		fclose(outTarget);
		fclose(outTether);
		fclose(outChaser);
		fclose(outTetherPoints);
		fclose(outPID);
		fclose(outTime);
		fclose(outSMC);
		fclose(outNodesPos);
		fclose(outNodesVel);
		fclose(outTension);

		return returnValue;
	

}
