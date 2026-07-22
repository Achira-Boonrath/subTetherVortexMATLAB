// Created by Eleonora Botta, March 31, 2017
// Modifying main_simpleTetherAxialWinch1Ring.cpp, so that the cable can do a 90deg turn in the second part, thanks to 2 rings 
// on the same part, but with different offsets, and a rigid fixed-length segment between them. 

#include <VxSim/KeyboardHelpDisplayICD.h>
#include <VxSim/KeyboardNotificationDisplayICD.h>

//#include <Vx/VxSmartPtr.h>
#include <VxSim/VxApplication.h>
#include <VxSim/VxDynamicsModuleICD.h>
//#include <VxSim/VxSimulationFileManager.h>
#include <VxSim/VxSimulatorModule.h> //was <VxSim/VxSimulatorModuleFactory.h> before
#include <VxSim/VxExtensionFactory.h>
//#include <Vx/Find.h> //#include <VxSim/FindInterface.h> // not in the VxSim folder
#include <VxSim/IKeyboard.h>

#include <VxGraphics/Context.h>
#include <VxGraphics/ICamera.h>
#include <VxGraphics/GraphicsModule.h>
#include <VxGraphics/Viewport.h>
#include <VxGraphicsPlugins/PerspectiveICD.h>
#include <VxGraphicsPlugins/DynamicsVisualizerICD.h>
#include <VxGraphicsPlugins/GraphicsModuleICD_OSG.h>

#include <VxGraphics/SceneGraph.h>
#include <VxGraphics/Services.h>
#include <VxGraphics/ShapeGenerator.h>

#include <VxPluginSystem/VxPluginManager.h>

#include <Vx/VxAssembly.h>
#include <Vx/VxBox.h>
#include <Vx/VxSphere.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxPart.h>
#include <Vx/VxTransform.h>
#include <VxSim/VxApplication.h>
#include <VxDynamics/Mechanism.h> // Changed it from Vx/VxMechanism.h, (didn't exist)

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
#include <Vx/VxBoundingSphere.h>

#include <assert.h>
#include <stdio.h>
#include <fstream>
#include <ctime>

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
#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxFactoryKey.h>
#include <VxSim/VxSmartInterface.h>
#include <CableSystems/CableSystemsICD.h>
#include <CableSystems/DynamicsICD.h>
#include <CableSystems/DynamicsPropertiesICD.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/Part.h>
#include <VxData/Container.h>
#include <VxData/FieldBase.h>
#include <VxDynamics/Capsule.h>
#include <VxContent/ConnectionContainerExtension.h>
#include <VxGraphicsPlugins/SplineICD.h>
#include <CableSystems/GraphicsICD.h>
#include <Vx/VxColor.h>
#include <VxDynamics/Cylinder.h>

// for cable
#include <CableSystems/DynamicsICD.h>

// for using my own functions
#include "MyLibraryNet/saveDynamicsQuantitiesNet.h"
#include "MyLibraryNet/saveDynamicsQuantitiesTarget.h"
#include "MyLibraryNet/saveDynamicsQuantitiesTether.h"
#include "MyLibraryNet/displayThreadsAndCornerMasses.h"
#include "MyLibraryNet/displayScene.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/createScene.h"
#include "MyLibraryNet/addTarget.h"
#include "MyLibraryNet/checkData.h"
#include "MyLibraryNet/readArray.h"
#include "MyLibraryNet/changeVelocity.h"
#include "MyLibraryNet/saveContactInformation.h"
#include <Vx/VxCylinder.h>

// display 
#include "MyLibraryNet/AccessoryExtension.h"
#include "MyLibraryNet/AccessoryModule.h"


using namespace Vx;
using namespace VxContent;
using namespace VxDynamics;
using namespace VxSim;
using namespace CableSystems;
using namespace CableSystems::DynamicsICD;

// This is where the Framework Application is created and the required modules are inserted, and where I call creation of scene, display, save quantities.
int main(int argc, const char *argv[])
{
    int returnValue = 0;

	// data for simulation
	VxReal finalTime = 10;
	VxReal tstep = 0.001;
	VxReal tsave = 0.001;
	int OSGview = 1;

	// initialize output files
	FILE* outX=0;
	FILE* outY=0;
	FILE* outZ=0;
	FILE* outVX=0;
	FILE* outVY=0;
	FILE* outVZ=0; 
	FILE* outF=0;
	FILE* outTime=0;
	FILE* outSolver=0;
	FILE* outTarget=0;
	FILE* outTether=0;
	FILE* outTetherPoints=0;
	FILE* outContact=0;
	outX = fopen("NodesX.txt", "w");
	outY = fopen("NodesY.txt", "w");
	outZ = fopen("NodesZ.txt", "w");
	outVX = fopen("NodesVX.txt", "w");
	outVY = fopen("NodesVY.txt", "w");
	outVZ = fopen("NodesVZ.txt", "w");
	outF = fopen("ThreadsF.txt", "w");
	outTime = fopen("IntegrationTime.txt", "w");
	outSolver = fopen("SolverLog.txt", "w");
	outTarget = fopen("TargetDynamics.txt","w");
	outTether = fopen("TetherDynamics_1.txt","w");
	outTetherPoints = fopen("TetherPoints.txt","w");
	outContact = fopen("ContactInfo.txt","w");

    try 
    {
		// Instantiate the Vortex application.
		Vx::VxSmartPtr<VxSim::VxApplication> application = new VxSim::VxApplication;

		VxSim::VxSmartInterface<VxSim::ISimulatorModule> dynamicsModule = VxSim::VxExtensionFactory::create(VxSim::VxDynamicsModuleICD::kFactoryKey);
		application->insertModule(dynamicsModule);

		/*// Instantiate a Graphic module using OSG and add it to the application.   
		VxPluginSystem::VxPluginManager::instance()->load("VxCycloneOSG");
		VxSim::VxSmartInterface<VxGraphics::GraphicsModule> graphicModule = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::GraphicsModuleICD::kModuleFactoryKey);
		application->insertModule(graphicModule);

		// Create a default camera for the Graphic module
		VxSim::VxSmartInterface<VxGraphics::ICamera> freeCamera = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::PerspectiveICD::kExtensionFactoryKey);
		freeCamera->lookAt(Vx::VxVector3(-5, 0, -1), Vx::VxVector3(0.0, 0.0, -0.5), Vx::VxVector3(0.0, 0.0, 1.0));  //  look from, focal point , z vector
		application->add(freeCamera);

		// Set the free camera as the active camera using the interface specialised for the VxGraphicModule
		VxGraphics::Setup windowSetup = graphicModule->createDefaultWindow();
		windowSetup.mViewport->setCamera(freeCamera);

		// Instantiate a Framework dynamics module add it to the application.   
		

		// get utility class for creating graphic shapes
		VxGraphics::ShapeGenerator *mShapeGenerator;
		mShapeGenerator = &graphicModule->getSceneGraph()->getServices().getShapeGenerator();

		// Instantiate the DynamicsVisualizer if you want to view the physics associated with various parts that have no graphics.   
		VxPluginSystem::VxPluginManager::instance()->load("DynamicsVisualizer");
		VxSim::VxExtension* dynamicsVisualizer = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::DynamicsVisualizerICD::kExtensionFactoryKey);
		dynamicsVisualizer->getInput(VxGraphicsPlugins::DynamicsVisualizerICD::kDisplayCollisionGeometry)->setValue(true);
		application->add(dynamicsVisualizer);


		// Add keyboard help display
		VxSim::VxExtension* helpDisplay = VxSim::VxExtensionFactory::create(VxSim::KeyboardHelpDisplayICD::kFactoryKey);
		application->add(helpDisplay);

		// Add keyboard command display
		VxSim::VxExtension* keyboardNotificationDisplay = VxSim::VxExtensionFactory::create(VxSim::KeyboardNotificationDisplayICD::kFactoryKey);
		application->add(keyboardNotificationDisplay);*/

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		// Instantiate the CableSystemDynamics to create cable
		VxPluginSystem::VxPluginManager::instance()->load("CableSystemsDynamics");

        // Create 2 parts and the tether
		VxSmartInterface<Scene>  scene = VxSim::VxExtensionFactory::create(SceneICD::kFactoryKey);
		VxSmartInterface<Assembly>  assembly = VxSim::VxExtensionFactory::create(VxDynamics::Assembly::kFactoryKey);
		
		VxSmartInterface<Part> part1 = VxExtensionFactory::create(PartICD::kFactoryKey);
        VxSmartInterface<Sphere> sphere = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		VxReal sphereRadius = 0.01; 
        sphere->parameterRadius = sphereRadius ;
		part1->parameterMassPropertiesContainer.mass = 1;
        VxSmartInterface<CollisionGeometry> cg = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		cg = sphere;
        part1->addCollisionGeometry(cg);
		part1->autoComputeInertiaAndCOM();
        VxVector3 pos1 = VxVector3(0, 0, 0);
		part1->setLocalTransform(VxMath::Transformation::createTranslation(pos1));
		part1->setLinearVelocity( Vx::VxVector3( 0.0, 0.0, 0.0) );
		part1->getVxPart()->freeze(true);
		assembly->addPart(part1);

		VxReal L0 = 2.0;
		VxReal winchRadius = 0.1;
		VxSmartInterface<Part> part2 = VxExtensionFactory::create(PartICD::kFactoryKey);
		VxSmartInterface<Sphere> sphere2 = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		VxReal sphereRadius2 = 0.1; 
        sphere2->parameterRadius = sphereRadius2 ;
		part2->parameterMassPropertiesContainer.mass = 1.0;
        VxSmartInterface<CollisionGeometry> cg2 = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		cg2 = sphere2;
        part2->addCollisionGeometry(cg2);
		part2->autoComputeInertiaAndCOM();
        VxVector3 pos2 = VxVector3(0, winchRadius, -L0);
		part2->setLocalTransform(VxMath::Transformation::createTranslation(pos2));
		part2->setLinearVelocity( Vx::VxVector3( 0.0, 0.0, 0.0) );
		part2->getVxPart()->freeze(true);
		assembly->addPart(part2);

		VxSmartInterface<Part> part3 = VxExtensionFactory::create(PartICD::kFactoryKey);
		VxSmartInterface<Sphere> sphere3 = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		VxReal sphereRadius3 = 0.1; 
        sphere3->parameterRadius = sphereRadius3 ;
		part3->parameterMassPropertiesContainer.mass = 1.0;
        VxSmartInterface<CollisionGeometry> cg3 = VxExtensionFactory::create(VxDynamics::Sphere::kFactoryKey); 
		cg3 = sphere3;
        part3->addCollisionGeometry(cg3);
		part3->autoComputeInertiaAndCOM();
        VxVector3 pos3 = VxVector3(0, winchRadius+L0, -L0);
		part3->setLocalTransform(VxMath::Transformation::createTranslation(pos3));
		part3->setLinearVelocity( Vx::VxVector3( 0.0, 0.0, 0.0) );
		assembly->addPart(part3);

		//VxSmartInterface<DistanceJoint> Dconstraint = attachDistance(assembly, part1, part2, 1.0, 1.0, 0.1);

		// Create a mechanism and put the assembly in it.
		VxSmartInterface<Mechanism> mechanism = VxSim::VxExtensionFactory::create(MechanismICD::kFactoryKey);
		mechanism->addAssembly(assembly);


		// create winch part
		VxSmartInterface<Part> winchPart = VxExtensionFactory::create(PartICD::kFactoryKey);
		winchPart->getVxPart()->setControl(Vx::VxPart::kControlDynamic);   
		Vx::VxVector3 winchPosition = VxVector3(0,0,0);
		winchPart->setLocalTransform(VxMath::Transformation::createTranslation(winchPosition));
	
		// add collision geometry to winch
		VxSmartInterface<Cylinder> cylinderWinch = VxExtensionFactory::create(VxDynamics::Cylinder::kFactoryKey); 
		cylinderWinch->parameterRadius = winchRadius;
		cylinderWinch->parameterHeight = 0.02;
		winchPart->parameterMassPropertiesContainer.mass = 1;
        VxSmartInterface<CollisionGeometry> cgWinch = VxExtensionFactory::create(VxDynamics::Cylinder::kFactoryKey); 
		cgWinch = cylinderWinch;
		VxTransform tm;
		tm.makeRotation(Vx::VxEulerAngles(0.0, VX_HALF_PI, 0.0));
		cgWinch->getVxCollisionGeometry()->setTransformRelative(tm);
        winchPart->addCollisionGeometry(cgWinch);
		winchPart->autoComputeInertiaAndCOM();

		// create hinge between chaser and winch, centered in center of the winch
		VxVector3 axis(1.0, 0.0, 0.0);
		const VxVector3 part2Offset = VxMath::Transformation::getTranslation(winchPart->getLocalTransform()) - VxMath::Transformation::getTranslation(part1->getLocalTransform());
		VxSmartInterface<Hinge> hingeForWinch = VxExtensionFactory::create(Hinge::kFactoryKey);
		hingeForWinch->inputAttachment1.part = winchPart;
		hingeForWinch->inputAttachment2.part = part1;
		hingeForWinch->inputAttachment1.position = VxVector3(0.0,0.0,0.0);
		hingeForWinch->inputAttachment2.position = part2Offset;
		hingeForWinch->inputAttachment1.primaryAxis = axis;
	
		// set control of the winch, either free or locked or motorized. If motorized, give the right angular velocity
		//// set hinge is locked
		//hingeForWinch->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlLocked); 
		//// set hinge is free
		//hingeForWinch->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlFree); 
		// set hinge is motorized: set also its velocity
		hingeForWinch->getVxConstraint()->setControl(VxHinge::kAngularCoordinate, VxConstraint::kControlMotorized); 					
		float winchAngularVelocity = 0.1/winchRadius; // (rad/s)
		hingeForWinch->getVxConstraint()->setMotorDesiredVelocity(VxHinge::kAngularCoordinate, winchAngularVelocity); 

		// add winch and hinge to assembly
		assembly->addPart(winchPart);
		assembly->addConstraint(hingeForWinch);
		VxCollisionRule collisionRule1(part1->getVxPart(), winchPart->getVxPart(), false); 
		assembly->getVxAssembly()->appendCollisionRule(collisionRule1);
		VxCollisionRule collisionRule2(part2->getVxPart(), assembly, false); 
		assembly->getVxAssembly()->appendCollisionRule(collisionRule2);
		
		VxCollisionRule collisionRule3(part3->getVxPart(), assembly, false); 
		assembly->getVxAssembly()->appendCollisionRule(collisionRule3);
		

		// Create the tether and put it in the mechanism.
		Vx::VxSmartPtr<VxSim::VxExtension> dynamicsExtension;
		Vx::VxSmartPtr<VxSim::VxExtension> graphicsExtension;
		Vx::VxSmartPtr<VxSim::VxObject> genericCable = VxSim::VxExtensionFactory::create(CableSystemsICD::kGenericCableKey)->toObject();   	
		if (nullptr != genericCable)
		{
			// Creates the Connections container
			Vx::VxSmartPtr<VxSim::VxExtension> connectionExtension =  VxSim::VxExtensionFactory::create(VxContent::ConnectionContainerExtensionICD::kFactoryKey);
			if( nullptr != connectionExtension )
			{
				connectionExtension->setName("Connections");
				genericCable->add(connectionExtension.get());
				// Creates the Dynamics extension
				dynamicsExtension = VxSim::VxExtensionFactory::create(CableSystemsICD::kDynamicsGenericCableKey);
				if ( nullptr != dynamicsExtension )
				{
						std::cout << "cableDynamicsExtension exists" << std::endl;
						dynamicsExtension->setName(CableSystemsICD::kDynamicsGenericCableName);
						genericCable->add(dynamicsExtension.get());
				}
				// Creates the Graphics Extension
				graphicsExtension = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::SplineICD::kFactoryKey);
				if ( nullptr != graphicsExtension )
				{
					std::cout << "graphicsExtension exists" << std::endl;
					graphicsExtension->setName(CableSystemsICD::kGraphicsCableName);
					genericCable->add(graphicsExtension.get());
				}
				// Creates connection between dynamicsExtension and graphicsExtension
				VxData::Container* cablesContainer = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
				auto connectionContainer = VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension>( connectionExtension );
				connectionContainer->create(&(*cablesContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID], graphicsExtension->getInput(VxGraphicsPlugins::SplineICD::kSplineControlPoints));
			}
		}
		// initialize containers to define cable properties
		VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
		VxData::FieldBase& defFieldBase = container[kDefinitionID];
		VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
		VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
		if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(4)) ) // We will set TOT items below    
		{
			//VxFatalError(0, "Cannot resize the List of PointDefinition\n");    
		}
		VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
		VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

		// Set attachments.  
		// set first attachment attached to winch
		VxData::Container* attachmentPoint1 = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));    
		(*attachmentPoint1)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kWinch));    
		(*attachmentPoint1)[PointDefinitionContainerID::kPartID].setValue(winchPart.getExtension());
		(*attachmentPoint1)[WinchDefinitionContainerID::kInverseWrappingID].setValue(true);

		VxData::Container* ring1 = dynamic_cast<VxData::Container*>(&(fieldBasePoints["1"]));    
		(*ring1)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
		(*ring1)[PointDefinitionContainerID::kPartID].setValue(part2.getExtension());   
		(*ring1)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, 0.0, sphereRadius2));
		(*ring1)[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
		// ring axis directed as cable flow

		VxData::Container* ring2 = dynamic_cast<VxData::Container*>(&(fieldBasePoints["2"]));    
		(*ring2)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
		(*ring2)[PointDefinitionContainerID::kPartID].setValue(part2.getExtension());   
		(*ring2)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, sphereRadius2, 0.0));
		(*ring2)[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));

		VxData::Container* attachmentPoint2 = dynamic_cast<VxData::Container*>(&(fieldBasePoints["3"]));    
		(*attachmentPoint2)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPoint2)[PointDefinitionContainerID::kPartID].setValue(part3.getExtension());   
		//(*attachmentPoint2)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, 0.0, -chaserSideLength/2.0));
	
		// Set properties of cable. 
		int nSections = 5;
		VxReal EAtrue = 100.0;
		//VxReal EItrue = 0.1;
		//VxReal EAused = EAtrue; // used EA
		//VxReal EIused = EItrue; // used EA
		//VxReal EIused = EItrue*(nSections-1)/nSections; // used EI
		VxReal EAused = EAtrue*(nSections-1)/nSections; // used EA
		
		VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
		params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(0.00001); 
		//params[CableSystemParamDefinitionContainerID::kGeometricRadiusID].setValue(0.00001); 
		params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(0); // 0: None, 1: spheres, 2: capsule
		params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(0.0000001); // kg/m
		//params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.0);
		//params[CableSystemParamDefinitionContainerID::kInternalDampingMaxStableLoadID].setValue(1.0);
		params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
		//params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
		params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(EAused/10.0);
		//params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(EIused/10.0);    
		//params[CableSystems::DynamicsPropertiesICD::kYoungsModulusID].setValue(tetherYoungModulus/100000000);

		VxData::Container& segmentDefinition00 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
		segmentDefinition00[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition00[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition00[SegmentDefinitionContainerID::kFlexibleID].setValue(true);

		VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
		segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(0);
		segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
		segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
		segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
		//segmentDefinition0[SegmentDefinitionContainerID::kMaxNumberOfSectionsID].setValue(1000);
		//segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(L0/nSections);
		//segmentDefinition0[SegmentDefinitionContainerID::kMaxElongationPercentageID].setValue(200.0);

		VxData::Container& segmentDefinition0b = dynamic_cast<VxData::Container&>(fieldBaseSegments["2"]); 
		segmentDefinition0b[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
		segmentDefinition0b[SegmentDefinitionContainerID::kFixedLengthID].setValue(true);    
		segmentDefinition0b[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
		segmentDefinition0b[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
		
		VxData::Container& segmentDefinition1 = dynamic_cast<VxData::Container&>(fieldBaseSegments["3"]); 
		segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(0);
		segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(0);
		//segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
		//segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
		segmentDefinition1[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition1[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition1[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
		segmentDefinition1[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
		segmentDefinition1[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
		//segmentDefinition0[SegmentDefinitionContainerID::kMaxNumberOfSectionsID].setValue(1000);
		segmentDefinition1[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(L0/nSections);
		//segmentDefinition0[SegmentDefinitionContainerID::kMaxElongationPercentageID].setValue(200.0);

		// Adds the cable to the mechanism
		if (mechanism.valid())            
		{               
			mechanism->addExtension(genericCable);
		}

		// Add the mechanism to the scene.
		scene->addMechanism(mechanism);

		// Change gravity
		VxReal gravityValue = 0.0;
		VxFrame::instance()->getUniverse(0)->setGravity(0,0,gravityValue); // by default we have gravity.

		application->add(scene); 
		
	    
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		// Find the first CableSystem dynamics extension, to retrieve its properties later on
		VxSmartInterface<IExtension> dynamicsCableExtension;
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
						if ( ext->getProxy()->getCreationKey() == CableSystemsICD::kGenericCableKey) 
						{
							std::cout << "\n \n I found the generic cable " <<  std::endl << std::endl;
							Vx::VxSmartPtr<VxObject> obj = ext->getProxy()->toObject();
							Vx::VxArray<VxExtension*> childExtensions = obj->findExtensions(CableSystemsICD::kDynamicsGenericCableKey);
							if ( childExtensions.size() == 1 )
							{
								dynamicsCableExtension = childExtensions[0];
							}
						}
					}                                
				}            
			}        
		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// Decide solver
		VxSolverParameters *solverPar =  VxFrame::instance()->getUniverse(0)->getSolverParameters(0);
		unsigned int solverType = solverPar->getConstraintSolver();   
		fprintf(outSolver, "The solver type is %d \n", solverType ); 
		fprintf(outSolver, "The solver tolerance is %g \n", solverPar->getConstraintStdSolverTolerance() ); 
		fprintf(outSolver, "The solver max iteration is %g \n", solverPar->getConstraintStdSolverMaxIteration() ); 
		// Vx::VxSolverParameters::setConstraintSolver(eConstraintSolver solver); 
		fprintf(outSolver, "The integrator time step is %g \n", VxFrame::instance()->getTimeStep() );  
		application->setSimulationFrameRate(1 / tstep);
		fprintf(outSolver, "The integrator time step is %g \n", VxFrame::instance()->getTimeStep() ); 

		fprintf(outSolver, "At static equilibrium, elongation should be: %g \n", -part2->parameterMassPropertiesContainer.mass * gravityValue / EAtrue  ); 
		fprintf(outSolver, "At static equilibrium, z-coord should be: %g \n", part2->getVxPart()->getPosition().z() + part2->parameterMassPropertiesContainer.mass * gravityValue / EAtrue  ); 

		
		// Set initial conditions
		int countSteps = 0; 
		VxReal currentTime = 0;
		fprintf(outTime, "%f \n", currentTime); 
		//saveDynamicsQuantitiesTether(dynamicsCableExtension, outTether, outTetherPoints, currentTime);
		//saveDynamicsQuantitiesTether(dynamicsCableExtension, outTether, outTetherPoints, hingeForWinchPtr, currentWinchSpoolingVelocity);

        // Run the simulation
		
		int nStepsSave = (int) (tsave/tstep) ;
		clock_t simulationTime;
		simulationTime = clock();

		while((application->update()) && (currentTime<finalTime))
        {    
			// update time
			countSteps++;
			currentTime = currentTime + tstep;
			if ( countSteps % nStepsSave == 0)
				fprintf(outTime, "%f \n", currentTime); 
			// save dynamics quantities and contact information at every saving time step
			std::cout << currentTime << std::endl;
			saveDynamicsQuantitiesTether(dynamicsCableExtension, outTether, outTetherPoints, currentTime);
			

		}
        
		
		// save computational time
		simulationTime = clock() - simulationTime;
		fprintf(outSolver, "\nIt took %g seconds to integrate. \n", ((float)simulationTime)/CLOCKS_PER_SEC); 
	}
    catch(const std::exception& ex )
    {
        std::cout << "Error : " << ex.what() << std::endl;
        returnValue = 1;
    }
    catch( ... )
    {
        Vx::LogWarn("Got an unhandled exception, application will exit!\n");
        returnValue = 1;
    }

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
	fclose(outTetherPoints);
	fclose(outContact);
	fclose(outTime);

    return returnValue;
}