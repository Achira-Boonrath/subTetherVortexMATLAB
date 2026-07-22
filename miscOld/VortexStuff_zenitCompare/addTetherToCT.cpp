// Create tether between chaser and target 

// Created by Liam Field 03/13/22
// To add:
// - input file functionality
// - total length input
// 
// 
// kAttachmentTypeEndID
//Vx::VxID kPointsID("Points"); (only for flexible segments)

#include "MyLibraryNet/addTethertoCT.h"
#include "MyLibraryNet/readParameter.h"
#include "MyLibraryNet/readArray.h"
//#include "MyLibraryNet/addTarget.h"

#include <VxSim/VxExtensionFactory.h>
#include <VxSim/VxFactoryKey.h>
#include <VxSim/VxSmartInterface.h>
#include <CableSystems/CableSystemsICD.h>
#include <CableSystems/DynamicsICD.h>
#include <CableSystems/DynamicsPropertiesICD.h>
#include <VxDynamics/Mechanism.h>
#include <VxDynamics/Part.h>
#include <Vx/VxPart.h>
#include <VxData/Container.h>
#include <VxData/FieldBase.h>
#include <VxDynamics/Capsule.h>
#include <VxContent/ConnectionContainerExtension.h>
#include <VxGraphicsPlugins/SplineICD.h>
#include <CableSystems/GraphicsICD.h>
#include <Vx/VxColor.h>
#include <Vx/VxFrame.h>

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;
using namespace CableSystems;
using namespace CableSystems::DynamicsICD;

#define MAXGRIDSIZE 100

void addTetherToCT(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> *partMatrix, VxSmartInterface<Part>* tetheredTargetPart)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);

	

	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	int tetherFlexible = readParameter<int>("tetherFlexible", tetherDataFileName);
	VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);
	VxReal tetherRadius = readParameter<VxReal>("diameter", tetherDataFileName)/2;
	VxReal tetherDensity = readParameter<VxReal>("tetherDensity", tetherDataFileName);
	VxReal tetherYoungModulus = readParameter<VxReal>("youngModulus", tetherDataFileName);
	VxReal tetherDamping = readParameter<VxReal>("segDamping", tetherDataFileName);
	VxReal natLength = readParameter<VxReal>("natLength", tetherDataFileName);
	VxReal ringsNb = readParameter<VxReal>("ringsNb", tetherDataFileName);
	VxReal tethersNb = readParameter<VxReal>("tethersNb", tetherDataFileName);
	std::cout  << std::endl << "Read " << std::endl;


	// read data for chaser
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	//VxReal chaserSideLengthX = readParameter<VxReal>("chaserSideLengthX", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);
	VxReal chaserAttX = readParameter<VxReal>("chaserAttX", chaserDataFileName);
	VxReal chaserAttY = readParameter<VxReal>("chaserAttY", chaserDataFileName);
	VxReal chaserAttZ = readParameter<VxReal>("chaserAttZ", chaserDataFileName);
	
	// read data for target
	std::stringstream targetDataFileName;
	targetDataFileName << pathData << "targetData.txt";
	VxReal targetSideLengthX = readParameter<VxReal>("targetSideLengthX", targetDataFileName);
	VxReal targetSideLengthY = readParameter<VxReal>("targetSideLengthY", targetDataFileName);
	VxReal targetSideLengthZ = readParameter<VxReal>("targetSideLengthZ", targetDataFileName);
	VxReal targetAttX = readParameter<VxReal>("targetAttX", targetDataFileName);
	VxReal targetAttY = readParameter<VxReal>("targetAttY", targetDataFileName);
	VxReal targetAttZ = readParameter<VxReal>("targetAttZ", targetDataFileName);
	VxReal targetQuatS = readParameter<VxReal>("targetIQuatS", targetDataFileName);
	VxReal targetQuatX = readParameter<VxReal>("targetIQuatX", targetDataFileName);
	VxReal targetQuatY = readParameter<VxReal>("targetIQuatY", targetDataFileName);
	VxReal targetQuatZ = readParameter<VxReal>("targetIQuatZ", targetDataFileName);

	//Attempting to adress cable orientation by applying target and chaser roation here, LF

	Vx::VxQuaternion TargetQi;
	(*tetheredTargetPart)->getVxPart()->getOrientationQuaternion(TargetQi);
	VxMath::Quaternion targetQuat = VxMath::Quaternion(targetQuatS, targetQuatX, targetQuatY, targetQuatZ);
	VxVector3 targAtt_World = targetQuat.rotate(Vx::VxVector3(targetAttX, targetAttY, targetAttZ));


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	VxVector3 Target1Q;
	(*tetheredTargetPart)->getVxPart()->getOrientationEulerAngles(Target1Q);
	auto g = 1;
	// Create generic cable extension, including dynamic cable, graphic cable, connection of these

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
			////Creates the Dynamic Cable Properties
			Vx::VxSmartPtr<VxSim::VxExtension> createDynamicsProperties = VxSim::VxExtensionFactory::create(CableSystemsICD::kDynamicsPropertiesPluginKey);
			if ( nullptr != createDynamicsProperties )
			{
				std::cout << "createDynamicsProperties exists" << std::endl;
				createDynamicsProperties->setName(CableSystemsICD::kDynamicsPropertiesName);
				genericCable->add(createDynamicsProperties.get());
			}
			// Creates connection between dynamicsExtension and graphicsExtension
			VxData::Container* cablesContainer = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
			auto connectionContainer = VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension>( connectionExtension );
			connectionContainer->create(&(*cablesContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID], graphicsExtension->getInput(VxGraphicsPlugins::SplineICD::kSplineControlPoints));
		}
	}
	// Adds the cable to the mechanism
	if (mechanism.valid())            
	{               
		mechanism->addExtension(genericCable);
	}
	//dynamicsExtension->getOutput(CableSystems::DynamicsICD::kTotalLengthID);
	//(*dynamicsExtension)[CableSystems::DynamicsICD::kTotalLengthID].setvalue(30);;
	VxData::Container* cablesContainer2 = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kTotalLengthID));
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Find the number of points to be specified for cable. It is: 1 for each ring + 2 attachments
	int pointsNb = 2; 
	std::cout << "pointsNb is " << pointsNb << std::endl;

	// initialize containers to define cable properties
	VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
	VxData::FieldBase& defFieldBase = container[kDefinitionID];
	VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
	VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
	if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(pointsNb)) ) // We will set TOT items below    
	{
		Vx::LogError(0, "Cannot resize the List of PointDefinition\n");    
	}
	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

	// Attempt at accessing DynamicsICD namespace functions (kTotalLengthID etc.) LF
	
	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//(*tetheredTargetPart)->getVxPart()->setOrientationQuaternion(targetQuat[0], targetQuat[1], targetQuat[2], targetQuat[3]);

	Vx::VxQuaternion TargetQ;
	(*tetheredTargetPart)->getVxPart()->getOrientationQuaternion(TargetQ);
	
	// Set attachments.  
	// set first attachment 
	VxData::Container* attachmentPointChaser = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"])); 
	VxData::Container* attachmentPointTarget = dynamic_cast<VxData::Container*>(&(fieldBasePoints["1"]));

	
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(chaserPart.getExtension());
		(*attachmentPointChaser)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3( chaserAttX, chaserAttY, chaserAttZ));

		VxVector3 Target1Qi;
		(*tetheredTargetPart)->getVxPart()->getOrientationEulerAngles(Target1Qi);

		(*attachmentPointTarget)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPointTarget)[PointDefinitionContainerID::kPartID].setValue((*tetheredTargetPart).getExtension());
		(*attachmentPointTarget)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(targetAttX, targetAttY, targetAttZ));
		//(*attachmentPointTarget)[PointDefinitionContainerID::kOffsetID].setValue(targAtt_World);
		//targAtt_World
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set properties of cable. 
	int nSections = 5;
	VxReal EAtrue = tetherYoungModulus * PI * pow(tetherRadius,2.00);	 // assuming tether stiffness is defined as EA
	VxReal EItrue = tetherYoungModulus / 4.0 * PI * pow(tetherRadius,4.00);
	//std::cout << EItrue << std::endl;
	VxReal densityTrue = tetherDensity * PI * pow(tetherRadius,2.0);	// linear density

	VxReal EAused;
	VxReal EIused;
	VxReal densityUsed;
	if (tetherFlexible == 1)
	{
		EAused = EAtrue*(nSections-1)/nSections; // used EA
		EIused = EItrue*(nSections-1)/nSections; // used EI
		densityUsed = densityTrue;
	}
	else
	{
		EAused = EAtrue; // used EA
		EIused = EItrue; // used EI
		densityUsed = densityTrue;
	}
	
	VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
	params[kTotalLengthID].setValue(natLength);
	params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(tetherRadius); 
	params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(densityUsed); // kg/m
	params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
	params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
	params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(tetherDamping);
	params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(EIused/10.0);  
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(0); // 0: None, 1: spheres, 2: capsule
	params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.25);
	//params[CableSystemParamDefinitionContainerID::kInternalDampingMaxStableLoadID].setValue(10.0);
	//params[CableSystemParamDefinitionContainerID::kInternalDampingEnabledID].setValue(1);
	//params[CableSystemParamDefinitionContainerID::kInternalDampingScaleID].setValue(1);
	params[CableSystemParamDefinitionContainerID::kMaxTensionID].setValue(1000.0);
	  
	////wn1AxialCable = PI/2.0/cableLength * pow( cableYoungModulus/cableDensity, 0.5);
	////VxReal tetherDamping = 2 * csiAxial / wn1AxialCable * cableStiffness;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Is this necessary? Setting parameters of the segments? Is this to turn turn parts of the cable dynamics off/on ? Specifically related
	// to the segments?

	VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]);

	if (tetherFlexible == 1)	// long segment of the cable is flexible
	{
		if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
		{
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(true);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(1.0);

			// Waypoints for placing cable appropriately, doesn't work for the position problem; LF
			//VxData::FieldBase& fb = segmentDefinition0[SegmentDefinitionContainerID::kPointsID];
			//VxData::FieldArrayBase& pointsFAB = dynamic_cast<VxData::FieldArrayBase&>(fb);
			//VxData::FieldArray<VxData::Field<VxVector3> >& pointsFA = dynamic_cast<VxData::FieldArray<VxData::Field<VxVector3> >&>(fb);
			//pointsFA.resize(29);

			//VxQuaternion ChaserQ;
			//chaserPart->getVxPart()->getOrientationQuaternion(ChaserQ);
			////VxVector3 HHH = ChaserQ.rotate(Vx::VxVector3(chaserAttX, chaserAttY, chaserAttZ));

			//Vx::VxQuaternion TargetQ;
			//(*tetheredTargetPart)->getVxPart()->getOrientationQuaternion(TargetQ);
			////VxVector3 HHHT = TargetQ.rotate(targetAtt);
			//VxVector3 chaserAttPos = ChaserQ.rotate(Vx::VxVector3(chaserAttX, chaserAttY, chaserAttZ)) + chaserPart->getVxPart()->getPosition();
			//VxVector3 targetAttPos = TargetQ.rotate(Vx::VxVector3(targetAttX, targetAttY, targetAttZ)) + (*tetheredTargetPart)->getVxPart()->getPosition();

			//VxVector3 L = targetAttPos - chaserAttPos;

			//for (int i = 0; i < 29; i++)
			//{
			//	pointsFA[i].setValue(chaserAttPos + L / L.norm() * (i + 1));
			//}

		}
		else
		{
			VxReal initialCableLength = chaserDistanceFromNet;
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(1);

			





		}
	}

	// Set properties of segments
	//int defSegmentIndex = 1;
	//for (int i=0; i<pointsNb; i++)
	//{
		//std::ostringstream indexSegmentSS;
		//indexPointSS << defSegmentIndex;
		//std::string indexSegmentString = indexSegmentSS.str();
		//std::cout << indexSegmentString << std::endl;

		/*VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    */

	//}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//VxReal cableLength = genericCable->getOutput(CableSystems::DynamicsICD::kTotalLengthID)->toReal(); // Cable length is 0?
	//VxReal CableLength = genericCable->getOutput(CableSystems::DynamicsICD::kTotalLengthID)->toReal();


	// Modify properties of graphics extension
	static Vx::VxColor threadColor = Vx::VxColor(); // (red, green, blue, transparency)
	VxData::Container& parameters = graphicsExtension->getParameterContainer();
	(parameters)[CableSystems::GraphicsICD::kRadiusID].setValue(tetherRadius * 4);
	(parameters)[CableSystems::GraphicsICD::kTechniqueID].setValue(0);
	(parameters)[CableSystems::GraphicsICD::kColorID].setValue(threadColor);
       // (parameters)[CableSystems::GraphicsICD::kNumberOfSidesID].setValue(10);
        //(parameters)[CableSystems::GraphicsICD::kRoundnessID].setValue(1.0);
       // (parameters)[CableSystems::GraphicsICD::kInterpolationID].setValue(false);
       // (parameters)[CableSystems::GraphicsICD::kSubdivisionPerSectionID].setValue(0);
}



// with possibility of multiple tethers

// 4 rings
void addTethersWith4Rings(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);

	// Find indexes for central part of net, and last part of the net (used in reading ring positions)
	int indexCenterPart, indexThirdPart, indexTwoThirdsPart; 
	if(NetSize%2 != 0) // NetSize is odd
	{
		indexCenterPart = (int)(NetSize-1)/2 ;
		//indexQuarterPart = (int)(NetSize-1)/4; 
		indexThirdPart = (int)(NetSize-1)/3; 
		indexTwoThirdsPart = NetSize - (int)(NetSize-1)/3; 
	}
	else 
		std::cout << std::endl << "ERROR: NetSize in file netData has to be odd. Please exit simulation and correct its value. \n ";
	int indexLastPart = NetSize - 1;

	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	int tetherFlexible = readParameter<int>("tetherFlexible", tetherDataFileName);
	VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);
	VxReal tetherRadius = readParameter<VxReal>("tetherRadius", tetherDataFileName);
	VxReal tetherDensity = readParameter<VxReal>("tetherDensity", tetherDataFileName);
	VxReal tetherYoungModulus = readParameter<VxReal>("tetherYoungModulus", tetherDataFileName);
	VxReal ringsNb = readParameter<VxReal>("ringsNb", tetherDataFileName);
	VxReal tethersNb = readParameter<VxReal>("tethersNb", tetherDataFileName);
	VxReal EImultiplier = readParameter<VxReal>("EImultiplier", tetherDataFileName);
	std::cout  << std::endl << "Read " << std::endl;

	// read rings and put indexes of parts in arrays
	int ringPartX[MAXGRIDSIZE];
	int ringPartY[MAXGRIDSIZE];
	int sizeRingPartX = 0;
	int sizeRingPartY = 0;
	if (ringsNb > 0)
	{
		std::cout  << std::endl << "Read " << std::endl;
		sizeRingPartX = readArray("ringsPartX", tetherDataFileName, ringPartX, indexCenterPart, indexLastPart, indexThirdPart, indexTwoThirdsPart);
		sizeRingPartY = readArray("ringsPartY", tetherDataFileName, ringPartY, indexCenterPart, indexLastPart, indexThirdPart, indexTwoThirdsPart);
	}

	// read data for chaser
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Create generic cable extension, including dynamic cable, graphic cable, connection of these

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
			////Creates the Dynamic Cable Properties
			//Vx::VxSmartPtr<VxSim::VxExtension> createDynamicsProperties = VxSim::VxExtensionFactory::create(CableSystemsICD::kDynamicsPropertiesPluginKey);
			//if ( nullptr != createDynamicsProperties )
			//{
			//	std::cout << "createDynamicsProperties exists" << std::endl;
			//	createDynamicsProperties->setName(CableSystemsICD::kDynamicsPropertiesName);
			//	genericCable->add(createDynamicsProperties.get());
			//}
			// Creates connection between dynamicsExtension and graphicsExtension
			VxData::Container* cablesContainer = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
			auto connectionContainer = VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension>( connectionExtension );
			connectionContainer->create(&(*cablesContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID], graphicsExtension->getInput(VxGraphicsPlugins::SplineICD::kSplineControlPoints));
		}
	}
	// Adds the cable to the mechanism
	if (mechanism.valid())            
	{               
		mechanism->addExtension(genericCable);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Find the number of points to be specified for cable. It is: 1 for each ring + 2 attachments
	ringsNb = sizeRingPartY;
	int pointsNb = ringsNb + 2; 
	std::cout << "pointsNb is " << pointsNb << std::endl;

	// initialize containers to define cable properties
	VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
	VxData::FieldBase& defFieldBase = container[kDefinitionID];
	VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
	VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
	if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(pointsNb)) ) // We will set TOT items below    
	{
		Vx::LogError(0, "Cannot resize the List of PointDefinition\n");    
	}
	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set attachments.  
	// set first attachment 
	VxData::Container* attachmentPointChaser = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));    
	if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(chaserPart.getExtension());   
		(*attachmentPointChaser)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, 0.0, -chaserSideLength/2.0));
	}
	else  // tether attached to winch
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kWinch));    
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(winchPart.getExtension());
		(*attachmentPointChaser)[WinchDefinitionContainerID::kInverseWrappingID].setValue(true);
	}

	// create ring points
	int defPointIndex = 1;
	for (int i=0; i<ringsNb; i++)
	{
		std::ostringstream indexPointSS;
		indexPointSS << defPointIndex;
		std::string indexPointString = indexPointSS.str();
		std::cout << indexPointString << std::endl;

		VxData::Container& ringPointNet_1 = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]);  // ring in center of net
		ringPointNet_1[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
		if (tetherCounter == 0)
		{	
			ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
			//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			if (i==0)
			{
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			}
			else if (i==1)
			{
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==3)
			{
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
		}
		else if (tetherCounter == 1)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexTwoThirdsPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));		
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexThirdPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));		
			}
		}
		else if (tetherCounter == 2)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension());   
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexThirdPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));		
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexTwoThirdsPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));		
			}
		}
		else if (tetherCounter == 3)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension());
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexThirdPart][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, 0.0));		
			}
			else if (i==3)	
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexTwoThirdsPart][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, 0.0));	
			}
		}
		defPointIndex = defPointIndex + 1;
	}
	
	// set last attachment point 
	std::ostringstream indexPointSS;
	indexPointSS << defPointIndex;
	std::string indexPointString = indexPointSS.str();
	std::cout << indexPointString << std::endl;
	VxData::Container& attachmentPointNet = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]); 
	attachmentPointNet[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
	if (ringsNb == 0)   // on central part of the net if there are no rings
		attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
	else				// on part [0][indexCenterPart] if there are rings
	{	
		if (tetherCounter == 0)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension());   
		else if (tetherCounter == 1)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension());   
		else if (tetherCounter == 2)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension());   
		else if (tetherCounter == 3)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexLastPart].getExtension());   
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set properties of cable. 
	int nSections = 20;
	VxReal EAtrue = tetherYoungModulus * PI * pow(tetherRadius,2.00);	 // assuming tether stiffness is defined as EA
	VxReal EItrue = tetherYoungModulus / 4.0 * PI * pow(tetherRadius,4.00);
	//std::cout << EItrue << std::endl;
	VxReal densityTrue = tetherDensity * PI * pow(tetherRadius,2.0);	// linear density

	VxReal EAused;
	VxReal EIused;
	VxReal densityUsed;
	if (tetherFlexible == 1)
	{
		EAused = EAtrue*(nSections-1)/nSections; // used EA
		EIused = EItrue*(nSections-1)/nSections; // used EI
		densityUsed = densityTrue;
	}
	else
	{
		EAused = EAtrue; // used EA
		EIused = EItrue; // used EI
		densityUsed = densityTrue;
	}
	EIused = EIused*EImultiplier;
	std::cout << " The value of EI used is " << EIused << std::endl << std::endl;

	VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
	params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(tetherRadius); 
	params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(densityUsed); // kg/m
	params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
	params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
	params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(EAused/10.0);
	params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(EIused/10.0);  
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(0); // 0: None, 1: spheres, 2: capsule
	params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.0);


	//params[CableSystemParamDefinitionContainerID::kInternalDampingMaxStableLoadID].setValue(1.0);
	  
	////wn1AxialCable = PI/2.0/cableLength * pow( cableYoungModulus/cableDensity, 0.5);
	////VxReal tetherDamping = 2 * csiAxial / wn1AxialCable * cableStiffness;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (tetherFlexible == 1)	// long segment of the cable is flexible
	{
		if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
		{
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(true);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(1.0);
		}
		else
		{
			VxReal initialCableLength = chaserDistanceFromNet;
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
			/*segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, 2.00, -1.0));
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, -2.00, -2.0));

			if (pointsNb > 2)
			{
				VxData::Container& segmentDefinition1 = dynamic_cast<VxData::Container&>(fieldBaseSegments["2"]); 
				/*segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
				segmentDefinition1[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
				segmentDefinition1[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
				segmentDefinition1[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
				segmentDefinition1[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
				segmentDefinition1[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
				segmentDefinition1[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			
				if (pointsNb > 3)
				{
					VxData::Container& segmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments["3"]); 
					/*segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
					segmentDefinition2[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
					segmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
					segmentDefinition2[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
					segmentDefinition2[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
					segmentDefinition2[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

					if (pointsNb > 4)
					{
						VxData::Container& segmentDefinition3 = dynamic_cast<VxData::Container&>(fieldBaseSegments["4"]); 
						/*segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
						segmentDefinition3[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
						segmentDefinition3[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
						segmentDefinition3[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
						segmentDefinition3[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
						segmentDefinition3[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
						segmentDefinition3[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

						if (pointsNb > 5)
						{
							VxData::Container& segmentDefinition4 = dynamic_cast<VxData::Container&>(fieldBaseSegments["5"]); 
							///*segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
							segmentDefinition4[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
							segmentDefinition4[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
							segmentDefinition4[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
							segmentDefinition4[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
							segmentDefinition4[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
						
							if (pointsNb > 6)
							{
								VxData::Container& segmentDefinition5 = dynamic_cast<VxData::Container&>(fieldBaseSegments["6"]); 
								///*segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
								segmentDefinition5[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
								segmentDefinition5[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
								segmentDefinition5[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
								segmentDefinition5[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
								segmentDefinition5[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
							}
						}
					}
				}
			}
		}
	}

	// Set properties of segments
	//int defSegmentIndex = 1;
	//for (int i=0; i<pointsNb; i++)
	//{
		//std::ostringstream indexSegmentSS;
		//indexPointSS << defSegmentIndex;
		//std::string indexSegmentString = indexSegmentSS.str();
		//std::cout << indexSegmentString << std::endl;

		/*VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    */

	//}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Modify properties of graphics extension
    static Vx::VxColor threadColor = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
	VxData::Container& parameters = graphicsExtension->getParameterContainer();
    (parameters)[CableSystems::GraphicsICD::kRadiusID].setValue(tetherRadius*60);
    (parameters)[CableSystems::GraphicsICD::kColorID].setValue(threadColor);
       // (parameters)[CableSystems::GraphicsICD::kNumberOfSidesID].setValue(10);
        //(parameters)[CableSystems::GraphicsICD::kRoundnessID].setValue(1.0);
       // (parameters)[CableSystems::GraphicsICD::kInterpolationID].setValue(false);
       // (parameters)[CableSystems::GraphicsICD::kSubdivisionPerSectionID].setValue(0);
}

// 3 rings
void addTethersWith3Rings(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);

	// Find indexes for central part of net, and last part of the net (used in reading ring positions)
	int indexCenterPart, indexQuarterPart; 
	if(NetSize%2 != 0) // NetSize is odd
	{
		indexCenterPart = (int)(NetSize-1)/2 ;
		indexQuarterPart = (int)(NetSize-1)/4; 
	}
	else 
		std::cout << std::endl << "ERROR: NetSize in file netData has to be odd. Please exit simulation and correct its value. \n ";
	int indexLastPart = NetSize - 1;

	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	int tetherFlexible = readParameter<int>("tetherFlexible", tetherDataFileName);
	VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);
	VxReal tetherRadius = readParameter<VxReal>("tetherRadius", tetherDataFileName);
	VxReal tetherDensity = readParameter<VxReal>("tetherDensity", tetherDataFileName);
	VxReal tetherYoungModulus = readParameter<VxReal>("tetherYoungModulus", tetherDataFileName);
	VxReal ringsNb = readParameter<VxReal>("ringsNb", tetherDataFileName);
	VxReal tethersNb = readParameter<VxReal>("tethersNb", tetherDataFileName);
	VxReal EImultiplier = readParameter<VxReal>("EImultiplier", tetherDataFileName);
	std::cout  << std::endl << "Read " << std::endl;

	// read rings and put indexes of parts in arrays
	int ringPartX[MAXGRIDSIZE];
	int ringPartY[MAXGRIDSIZE];
	int sizeRingPartX = 0;
	int sizeRingPartY = 0;
	if (ringsNb > 0)
	{
		std::cout  << std::endl << "Read " << std::endl;
		sizeRingPartX = readArray("ringsPartX", tetherDataFileName, ringPartX, indexCenterPart, indexLastPart);
		sizeRingPartY = readArray("ringsPartY", tetherDataFileName, ringPartY, indexCenterPart, indexLastPart);
	}

	// read data for chaser
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Create generic cable extension, including dynamic cable, graphic cable, connection of these

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
			////Creates the Dynamic Cable Properties
			//Vx::VxSmartPtr<VxSim::VxExtension> createDynamicsProperties = VxSim::VxExtensionFactory::create(CableSystemsICD::kDynamicsPropertiesPluginKey);
			//if ( nullptr != createDynamicsProperties )
			//{
			//	std::cout << "createDynamicsProperties exists" << std::endl;
			//	createDynamicsProperties->setName(CableSystemsICD::kDynamicsPropertiesName);
			//	genericCable->add(createDynamicsProperties.get());
			//}
			// Creates connection between dynamicsExtension and graphicsExtension
			VxData::Container* cablesContainer = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
			auto connectionContainer = VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension>( connectionExtension );
			connectionContainer->create(&(*cablesContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID], graphicsExtension->getInput(VxGraphicsPlugins::SplineICD::kSplineControlPoints));
		}
	}
	// Adds the cable to the mechanism
	if (mechanism.valid())            
	{               
		mechanism->addExtension(genericCable);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Find the number of points to be specified for cable. It is: 1 for each ring + 2 attachments
	ringsNb = sizeRingPartY;
	int pointsNb = ringsNb + 2; 
	std::cout << "pointsNb is " << pointsNb << std::endl;

	// initialize containers to define cable properties
	VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
	VxData::FieldBase& defFieldBase = container[kDefinitionID];
	VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
	VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
	if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(pointsNb)) ) // We will set TOT items below    
	{
		Vx::LogError(0, "Cannot resize the List of PointDefinition\n");    
	}
	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set attachments.  
	// set first attachment 
	VxData::Container* attachmentPointChaser = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));    
	if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(chaserPart.getExtension());   
		(*attachmentPointChaser)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, 0.0, -chaserSideLength/2.0));
	}
	else  // tether attached to winch
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kWinch));    
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(winchPart.getExtension());
		(*attachmentPointChaser)[WinchDefinitionContainerID::kInverseWrappingID].setValue(true);
	}

	// create ring points
	int defPointIndex = 1;
	for (int i=0; i<ringsNb; i++)
	{
		std::ostringstream indexPointSS;
		indexPointSS << defPointIndex;
		std::string indexPointString = indexPointSS.str();
		std::cout << indexPointString << std::endl;

		VxData::Container& ringPointNet_1 = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]);  // ring in center of net
		ringPointNet_1[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
		if (tetherCounter == 0)
		{	
			ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
			//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			if (i==0)
			{
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));*/
			}
		}
		else if (tetherCounter == 1)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, 0.0));		
			}
		}
		else if (tetherCounter == 2)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension());   
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexCenterPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));		
			}
		}
		else if (tetherCounter == 3)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension());
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));	
			}
		}
		defPointIndex = defPointIndex + 1;
	}
	
	// set last attachment point 
	std::ostringstream indexPointSS;
	indexPointSS << defPointIndex;
	std::string indexPointString = indexPointSS.str();
	std::cout << indexPointString << std::endl;
	VxData::Container& attachmentPointNet = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]); 
	attachmentPointNet[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
	if (ringsNb == 0)   // on central part of the net if there are no rings
		attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
	else				// on part [0][indexCenterPart] if there are rings
	{	
		if (tetherCounter == 0)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension());   
		else if (tetherCounter == 1)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension());   
		else if (tetherCounter == 2)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension());   
		else if (tetherCounter == 3)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexLastPart].getExtension());   
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set properties of cable. 
	int nSections = 20;
	VxReal EAtrue = tetherYoungModulus * PI * pow(tetherRadius,2.00);	 // assuming tether stiffness is defined as EA
	VxReal EItrue = tetherYoungModulus / 4.0 * PI * pow(tetherRadius,4.00);
	//std::cout << EItrue << std::endl;
	VxReal densityTrue = tetherDensity * PI * pow(tetherRadius,2.0);	// linear density

	VxReal EAused;
	VxReal EIused;
	VxReal densityUsed;
	if (tetherFlexible == 1)
	{
		EAused = EAtrue*(nSections-1)/nSections; // used EA
		EIused = EItrue*(nSections-1)/nSections; // used EI
		densityUsed = densityTrue;
	}
	else
	{
		EAused = EAtrue; // used EA
		EIused = EItrue; // used EI
		densityUsed = densityTrue;
	}
	EIused = EIused*EImultiplier;
	std::cout << " The value of EI used is " << EIused << std::endl << std::endl;

	VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
	params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(tetherRadius); 
	params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(densityUsed); // kg/m
	params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
	params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
	params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(EAused/10.0);
	params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(EIused/10.0);  
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(0); // 0: None, 1: spheres, 2: capsule
	params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.0);
	//params[CableSystemParamDefinitionContainerID::kInternalDampingMaxStableLoadID].setValue(1.0);
	  
	////wn1AxialCable = PI/2.0/cableLength * pow( cableYoungModulus/cableDensity, 0.5);
	////VxReal tetherDamping = 2 * csiAxial / wn1AxialCable * cableStiffness;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (tetherFlexible == 1)	// long segment of the cable is flexible
	{
		if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
		{
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(true);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(1.0);
		}
		else
		{
			VxReal initialCableLength = chaserDistanceFromNet;
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
			/*segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, 2.00, -1.0));
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, -2.00, -2.0));

			if (pointsNb > 2)
			{
				VxData::Container& segmentDefinition1 = dynamic_cast<VxData::Container&>(fieldBaseSegments["2"]); 
				/*segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
				segmentDefinition1[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
				segmentDefinition1[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
				segmentDefinition1[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
				segmentDefinition1[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
				segmentDefinition1[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
				segmentDefinition1[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			
				if (pointsNb > 3)
				{
					VxData::Container& segmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments["3"]); 
					/*segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
					segmentDefinition2[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
					segmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
					segmentDefinition2[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
					segmentDefinition2[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

					if (pointsNb > 4)
					{
						VxData::Container& segmentDefinition3 = dynamic_cast<VxData::Container&>(fieldBaseSegments["4"]); 
						/*segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
						segmentDefinition3[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
						segmentDefinition3[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
						segmentDefinition3[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
						segmentDefinition3[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
						segmentDefinition3[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
						segmentDefinition3[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

						if (pointsNb > 5)
						{
							VxData::Container& segmentDefinition4 = dynamic_cast<VxData::Container&>(fieldBaseSegments["5"]); 
							///*segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
							segmentDefinition4[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
							segmentDefinition4[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
							segmentDefinition4[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
							segmentDefinition4[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
						
							if (pointsNb > 6)
							{
								VxData::Container& segmentDefinition5 = dynamic_cast<VxData::Container&>(fieldBaseSegments["6"]); 
								///*segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
								segmentDefinition5[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
								segmentDefinition5[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
								segmentDefinition5[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
								segmentDefinition5[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
							}
						}
					}
				}
			}
		}
	}

	// Set properties of segments
	//int defSegmentIndex = 1;
	//for (int i=0; i<pointsNb; i++)
	//{
		//std::ostringstream indexSegmentSS;
		//indexPointSS << defSegmentIndex;
		//std::string indexSegmentString = indexSegmentSS.str();
		//std::cout << indexSegmentString << std::endl;

		/*VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    */

	//}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Modify properties of graphics extension
    static Vx::VxColor threadColor = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
	VxData::Container& parameters = graphicsExtension->getParameterContainer();
    (parameters)[CableSystems::GraphicsICD::kRadiusID].setValue(tetherRadius*60);
    (parameters)[CableSystems::GraphicsICD::kColorID].setValue(threadColor);
       // (parameters)[CableSystems::GraphicsICD::kNumberOfSidesID].setValue(10);
        //(parameters)[CableSystems::GraphicsICD::kRoundnessID].setValue(1.0);
       // (parameters)[CableSystems::GraphicsICD::kInterpolationID].setValue(false);
       // (parameters)[CableSystems::GraphicsICD::kSubdivisionPerSectionID].setValue(0);
}


void addTethersWith3RingsNew(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);

	// Find indexes for central part of net, and last part of the net (used in reading ring positions)
	int indexCenterPart, indexQuarterPart; 
	if(NetSize%2 != 0) // NetSize is odd
	{
		indexCenterPart = (int)(NetSize-1)/2 ;
		indexQuarterPart = (int)(NetSize-1)/4; 
	}
	else 
		std::cout << std::endl << "ERROR: NetSize in file netData has to be odd. Please exit simulation and correct its value. \n ";
	int indexLastPart = NetSize - 1;

	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	int tetherFlexible = readParameter<int>("tetherFlexible", tetherDataFileName);
	VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);
	VxReal tetherRadius = readParameter<VxReal>("tetherRadius", tetherDataFileName);
	VxReal tetherDensity = readParameter<VxReal>("tetherDensity", tetherDataFileName);
	VxReal tetherYoungModulus = readParameter<VxReal>("tetherYoungModulus", tetherDataFileName);
	VxReal ringsNb = readParameter<VxReal>("ringsNb", tetherDataFileName);
	VxReal tethersNb = readParameter<VxReal>("tethersNb", tetherDataFileName);
	VxReal EImultiplier = readParameter<VxReal>("EImultiplier", tetherDataFileName);
	std::cout  << std::endl << "Read " << std::endl;

	// read rings and put indexes of parts in arrays
	int ringPartX[MAXGRIDSIZE];
	int ringPartY[MAXGRIDSIZE];
	int sizeRingPartX = 0;
	int sizeRingPartY = 0;
	if (ringsNb > 0)
	{
		std::cout  << std::endl << "Read " << std::endl;
		sizeRingPartX = readArray("ringsPartX", tetherDataFileName, ringPartX, indexCenterPart, indexLastPart);
		sizeRingPartY = readArray("ringsPartY", tetherDataFileName, ringPartY, indexCenterPart, indexLastPart);
	}

	// read data for chaser
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Create generic cable extension, including dynamic cable, graphic cable, connection of these

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
			////Creates the Dynamic Cable Properties
			//Vx::VxSmartPtr<VxSim::VxExtension> createDynamicsProperties = VxSim::VxExtensionFactory::create(CableSystemsICD::kDynamicsPropertiesPluginKey);
			//if ( nullptr != createDynamicsProperties )
			//{
			//	std::cout << "createDynamicsProperties exists" << std::endl;
			//	createDynamicsProperties->setName(CableSystemsICD::kDynamicsPropertiesName);
			//	genericCable->add(createDynamicsProperties.get());
			//}
			// Creates connection between dynamicsExtension and graphicsExtension
			VxData::Container* cablesContainer = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
			auto connectionContainer = VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension>( connectionExtension );
			connectionContainer->create(&(*cablesContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID], graphicsExtension->getInput(VxGraphicsPlugins::SplineICD::kSplineControlPoints));
		}
	}
	// Adds the cable to the mechanism
	if (mechanism.valid())            
	{               
		mechanism->addExtension(genericCable);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Find the number of points to be specified for cable. It is: 1 for each ring + 2 attachments
	ringsNb = sizeRingPartY*2 - 1; // *2 - 1 to have right direction of rings
	int pointsNb = ringsNb + 2; 
	std::cout << "pointsNb is " << pointsNb << std::endl;

	// initialize containers to define cable properties
	VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
	VxData::FieldBase& defFieldBase = container[kDefinitionID];
	VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
	VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
	if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(pointsNb)) ) // We will set TOT items below    
	{
		Vx::LogError(0, "Cannot resize the List of PointDefinition\n");    
	}
	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set attachments.  
	// set first attachment 
	VxData::Container* attachmentPointChaser = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));    
	if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(chaserPart.getExtension());   
		(*attachmentPointChaser)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, 0.0, -chaserSideLength/2.0));
	}
	else  // tether attached to winch
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kWinch));    
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(winchPart.getExtension());
		(*attachmentPointChaser)[WinchDefinitionContainerID::kInverseWrappingID].setValue(true);
	}

	// create ring points
	int defPointIndex = 1;
	for (int i=0; i<ringsNb; i++)
	{
		std::ostringstream indexPointSS;
		indexPointSS << defPointIndex;
		std::string indexPointString = indexPointSS.str();
		std::cout << indexPointString << std::endl;

		VxData::Container& ringPointNet_1 = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]);  // ring in center of net
		ringPointNet_1[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
		if (tetherCounter == 0)
		{	
			//ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[ringPartX[i]][ringPartY[i]].getExtension());   
			//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 1.0, .0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexLastPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 1.0, .0));
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexLastPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, .0));
			}
			else if (i==4)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, .0));
			}
		}
		else if (tetherCounter == 1)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension()); 
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, -1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension()); 
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, -1.0, 0.0));
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, .0));
			}
			else if (i==4)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][0].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, .0));
			}
		}
		else if (tetherCounter == 2)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension()); 
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, -1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, -1.0, 0.0));
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==4)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexCenterPart].getExtension()); 
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));		
			}
		}
		else if (tetherCounter == 3)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension()); 
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension());
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 1.0, 0.0));
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension());
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));
			}
			else if (i==4)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexLastPart].getExtension()); 
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));	
			}
		}
		defPointIndex = defPointIndex + 1;
	}
	
	// set last attachment point 
	std::ostringstream indexPointSS;
	indexPointSS << defPointIndex;
	std::string indexPointString = indexPointSS.str();
	std::cout << indexPointString << std::endl;
	VxData::Container& attachmentPointNet = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]); 
	attachmentPointNet[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
	if (ringsNb == 0)   // on central part of the net if there are no rings
		attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
	else				// on part [0][indexCenterPart] if there are rings
	{	
		if (tetherCounter == 0)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension());   
		else if (tetherCounter == 1)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension());   
		else if (tetherCounter == 2)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension());   
		else if (tetherCounter == 3)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexLastPart].getExtension());   
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set properties of cable. 
	int nSections = 20;
	VxReal EAtrue = tetherYoungModulus * PI * pow(tetherRadius,2.00);	 // assuming tether stiffness is defined as EA
	VxReal EItrue = tetherYoungModulus / 4.0 * PI * pow(tetherRadius,4.00);
	//std::cout << EItrue << std::endl;
	VxReal densityTrue = tetherDensity * PI * pow(tetherRadius,2.0);	// linear density

	VxReal EAused;
	VxReal EIused;
	VxReal densityUsed;
	if (tetherFlexible == 1)
	{
		EAused = EAtrue*(nSections-1)/nSections; // used EA
		EIused = EItrue*(nSections-1)/nSections; // used EI
		densityUsed = densityTrue;
	}
	else
	{
		EAused = EAtrue; // used EA
		EIused = EItrue; // used EI
		densityUsed = densityTrue;
	}
	EIused = EIused*EImultiplier;
	std::cout << " The value of EI used is " << EIused << std::endl << std::endl;

	VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
	params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(tetherRadius); 
	params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(densityUsed); // kg/m
	params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
	params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
	params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(EAused/10.0);
	params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(EIused/10.0);  
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(0); // 0: None, 1: spheres, 2: capsule
	params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.0);
	//params[CableSystemParamDefinitionContainerID::kInternalDampingMaxStableLoadID].setValue(1.0);
	  
	////wn1AxialCable = PI/2.0/cableLength * pow( cableYoungModulus/cableDensity, 0.5);
	////VxReal tetherDamping = 2 * csiAxial / wn1AxialCable * cableStiffness;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (tetherFlexible == 1)	// long segment of the cable is flexible
	{
		if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
		{
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(true);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(1.0);
		}
		else
		{
			VxReal initialCableLength = chaserDistanceFromNet;
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
			/*segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, 2.00, -1.0));
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, -2.00, -2.0));

			if (pointsNb > 2)
			{
				VxData::Container& segmentDefinition1 = dynamic_cast<VxData::Container&>(fieldBaseSegments["2"]); 
				/*segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
				segmentDefinition1[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
				segmentDefinition1[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
				segmentDefinition1[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
				segmentDefinition1[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
				segmentDefinition1[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
				segmentDefinition1[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			
				if (pointsNb > 3)
				{
					VxData::Container& segmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments["3"]); 
					/*segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
					segmentDefinition2[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
					segmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
					segmentDefinition2[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
					segmentDefinition2[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

					if (pointsNb > 4)
					{
						VxData::Container& segmentDefinition3 = dynamic_cast<VxData::Container&>(fieldBaseSegments["4"]); 
						/*segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
						segmentDefinition3[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
						segmentDefinition3[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
						segmentDefinition3[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
						segmentDefinition3[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
						segmentDefinition3[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
						segmentDefinition3[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

						if (pointsNb > 5)
						{
							VxData::Container& segmentDefinition4 = dynamic_cast<VxData::Container&>(fieldBaseSegments["5"]); 
							///*segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
							segmentDefinition4[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
							segmentDefinition4[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
							segmentDefinition4[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
							segmentDefinition4[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
						
							if (pointsNb > 6)
							{
								VxData::Container& segmentDefinition5 = dynamic_cast<VxData::Container&>(fieldBaseSegments["6"]); 
								///*segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
								segmentDefinition5[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
								segmentDefinition5[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
								segmentDefinition5[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
								segmentDefinition5[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
							}
						}
					}
				}
			}
		}
	}

	// Set properties of segments
	//int defSegmentIndex = 1;
	//for (int i=0; i<pointsNb; i++)
	//{
		//std::ostringstream indexSegmentSS;
		//indexPointSS << defSegmentIndex;
		//std::string indexSegmentString = indexSegmentSS.str();
		//std::cout << indexSegmentString << std::endl;

		/*VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    */

	//}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Modify properties of graphics extension
    static Vx::VxColor threadColor = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
	VxData::Container& parameters = graphicsExtension->getParameterContainer();
    (parameters)[CableSystems::GraphicsICD::kRadiusID].setValue(tetherRadius*60);
    (parameters)[CableSystems::GraphicsICD::kColorID].setValue(threadColor);
       // (parameters)[CableSystems::GraphicsICD::kNumberOfSidesID].setValue(10);
        //(parameters)[CableSystems::GraphicsICD::kRoundnessID].setValue(1.0);
       // (parameters)[CableSystems::GraphicsICD::kInterpolationID].setValue(false);
       // (parameters)[CableSystems::GraphicsICD::kSubdivisionPerSectionID].setValue(0);
}



void addTethersWith3RingsConfiguration2(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);

	// Find indexes for central part of net, and last part of the net (used in reading ring positions)
	int indexCenterPart, indexQuarterPart; 
	if(NetSize%2 != 0) // NetSize is odd
	{
		indexCenterPart = (int)(NetSize-1)/2 ;
		indexQuarterPart = (int)(NetSize-1)/4; 
	}
	else 
		std::cout << std::endl << "ERROR: NetSize in file netData has to be odd. Please exit simulation and correct its value. \n ";
	int indexLastPart = NetSize - 1;

	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	int tetherFlexible = readParameter<int>("tetherFlexible", tetherDataFileName);
	VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);
	VxReal tetherRadius = readParameter<VxReal>("tetherRadius", tetherDataFileName);
	VxReal tetherDensity = readParameter<VxReal>("tetherDensity", tetherDataFileName);
	VxReal tetherYoungModulus = readParameter<VxReal>("tetherYoungModulus", tetherDataFileName);
	VxReal ringsNb = readParameter<VxReal>("ringsNb", tetherDataFileName);
	VxReal tethersNb = readParameter<VxReal>("tethersNb", tetherDataFileName);
	VxReal EImultiplier = readParameter<VxReal>("EImultiplier", tetherDataFileName);
	std::cout  << std::endl << "Read " << std::endl;

	// read rings and put indexes of parts in arrays
	int ringPartX[MAXGRIDSIZE];
	int ringPartY[MAXGRIDSIZE];
	int sizeRingPartX = 0;
	int sizeRingPartY = 0;
	if (ringsNb > 0)
	{
		std::cout  << std::endl << "Read " << std::endl;
		sizeRingPartX = readArray("ringsPartX", tetherDataFileName, ringPartX, indexCenterPart, indexLastPart);
		sizeRingPartY = readArray("ringsPartY", tetherDataFileName, ringPartY, indexCenterPart, indexLastPart);
	}

	// read data for chaser
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Create generic cable extension, including dynamic cable, graphic cable, connection of these

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
			////Creates the Dynamic Cable Properties
			//Vx::VxSmartPtr<VxSim::VxExtension> createDynamicsProperties = VxSim::VxExtensionFactory::create(CableSystemsICD::kDynamicsPropertiesPluginKey);
			//if ( nullptr != createDynamicsProperties )
			//{
			//	std::cout << "createDynamicsProperties exists" << std::endl;
			//	createDynamicsProperties->setName(CableSystemsICD::kDynamicsPropertiesName);
			//	genericCable->add(createDynamicsProperties.get());
			//}
			// Creates connection between dynamicsExtension and graphicsExtension
			VxData::Container* cablesContainer = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
			auto connectionContainer = VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension>( connectionExtension );
			connectionContainer->create(&(*cablesContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID], graphicsExtension->getInput(VxGraphicsPlugins::SplineICD::kSplineControlPoints));
		}
	}
	// Adds the cable to the mechanism
	if (mechanism.valid())            
	{               
		mechanism->addExtension(genericCable);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Find the number of points to be specified for cable. It is: 1 for each ring + 2 attachments
	ringsNb = sizeRingPartY;
	int pointsNb = ringsNb + 2; 
	std::cout << "pointsNb is " << pointsNb << std::endl;

	// initialize containers to define cable properties
	VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
	VxData::FieldBase& defFieldBase = container[kDefinitionID];
	VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
	VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
	if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(pointsNb)) ) // We will set TOT items below    
	{
		Vx::LogError(0, "Cannot resize the List of PointDefinition\n");    
	}
	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set attachments.  
	// set first attachment 
	VxData::Container* attachmentPointChaser = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));    
	if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(chaserPart.getExtension());   
		(*attachmentPointChaser)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, 0.0, -chaserSideLength/2.0));
	}
	else  // tether attached to winch
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kWinch));    
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(winchPart.getExtension());
		(*attachmentPointChaser)[WinchDefinitionContainerID::kInverseWrappingID].setValue(true);
	}

	// create ring points
	int defPointIndex = 1;
	for (int i=0; i<ringsNb; i++)
	{
		std::ostringstream indexPointSS;
		indexPointSS << defPointIndex;
		std::string indexPointString = indexPointSS.str();
		std::cout << indexPointString << std::endl;

		VxData::Container& ringPointNet_1 = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]);  // ring in center of net
		ringPointNet_1[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
		
		if (tetherCounter == 0)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexCenterPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));		
			}
		}
		else if (tetherCounter == 1)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));		
			}
		}
		else if (tetherCounter == 2)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexCenterPart].getExtension());   
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, 0.0));		
			}
		}
		else if (tetherCounter == 3)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexLastPart].getExtension());
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));	
			}
		}
		defPointIndex = defPointIndex + 1;
	}
	
	// set last attachment point 
	std::ostringstream indexPointSS;
	indexPointSS << defPointIndex;
	std::string indexPointString = indexPointSS.str();
	std::cout << indexPointString << std::endl;
	VxData::Container& attachmentPointNet = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]); 
	attachmentPointNet[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
	if (ringsNb == 0)   // on central part of the net if there are no rings
		attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
	else				// on part [0][indexCenterPart] if there are rings
	{	
		if (tetherCounter == 0)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][0].getExtension());   
		else if (tetherCounter == 1)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexCenterPart].getExtension());   
		else if (tetherCounter == 2)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexLastPart].getExtension());   
		else if (tetherCounter == 3)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexCenterPart].getExtension());   
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set properties of cable. 
	int nSections = 20;
	VxReal EAtrue = tetherYoungModulus * PI * pow(tetherRadius,2.00);	 // assuming tether stiffness is defined as EA
	VxReal EItrue = tetherYoungModulus / 4.0 * PI * pow(tetherRadius,4.00);
	//std::cout << EItrue << std::endl;
	VxReal densityTrue = tetherDensity * PI * pow(tetherRadius,2.0);	// linear density

	VxReal EAused;
	VxReal EIused;
	VxReal densityUsed;
	if (tetherFlexible == 1)
	{
		EAused = EAtrue*(nSections-1)/nSections; // used EA
		EIused = EItrue*(nSections-1)/nSections; // used EI
		densityUsed = densityTrue;
	}
	else
	{
		EAused = EAtrue; // used EA
		EIused = EItrue; // used EI
		densityUsed = densityTrue;
	}
	EIused = EIused*EImultiplier;
	std::cout << " The value of EI used is " << EIused << std::endl << std::endl;

	VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
	params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(tetherRadius); 
	params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(densityUsed); // kg/m
	params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
	params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
	params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(EAused/10.0);
	params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(EIused/10.0);  
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(0); // 0: None, 1: spheres, 2: capsule
	params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.0);
	//params[CableSystemParamDefinitionContainerID::kInternalDampingMaxStableLoadID].setValue(1.0);
	  
	////wn1AxialCable = PI/2.0/cableLength * pow( cableYoungModulus/cableDensity, 0.5);
	////VxReal tetherDamping = 2 * csiAxial / wn1AxialCable * cableStiffness;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (tetherFlexible == 1)	// long segment of the cable is flexible
	{
		if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
		{
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(true);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(1.0);
		}
		else
		{
			VxReal initialCableLength = chaserDistanceFromNet;
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
			/*segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, 2.00, -1.0));
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, -2.00, -2.0));

			if (pointsNb > 2)
			{
				VxData::Container& segmentDefinition1 = dynamic_cast<VxData::Container&>(fieldBaseSegments["2"]); 
				/*segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
				segmentDefinition1[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
				segmentDefinition1[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
				segmentDefinition1[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
				segmentDefinition1[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
				segmentDefinition1[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
				segmentDefinition1[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			
				if (pointsNb > 3)
				{
					VxData::Container& segmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments["3"]); 
					/*segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
					segmentDefinition2[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
					segmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
					segmentDefinition2[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
					segmentDefinition2[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

					if (pointsNb > 4)
					{
						VxData::Container& segmentDefinition3 = dynamic_cast<VxData::Container&>(fieldBaseSegments["4"]); 
						/*segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
						segmentDefinition3[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
						segmentDefinition3[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
						segmentDefinition3[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
						segmentDefinition3[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
						segmentDefinition3[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
						segmentDefinition3[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

						if (pointsNb > 5)
						{
							VxData::Container& segmentDefinition4 = dynamic_cast<VxData::Container&>(fieldBaseSegments["5"]); 
							///*segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
							segmentDefinition4[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
							segmentDefinition4[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
							segmentDefinition4[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
							segmentDefinition4[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
						
							if (pointsNb > 6)
							{
								VxData::Container& segmentDefinition5 = dynamic_cast<VxData::Container&>(fieldBaseSegments["6"]); 
								///*segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
								segmentDefinition5[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
								segmentDefinition5[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
								segmentDefinition5[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
								segmentDefinition5[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
							}
						}
					}
				}
			}
		}
	}

	// Set properties of segments
	//int defSegmentIndex = 1;
	//for (int i=0; i<pointsNb; i++)
	//{
		//std::ostringstream indexSegmentSS;
		//indexPointSS << defSegmentIndex;
		//std::string indexSegmentString = indexSegmentSS.str();
		//std::cout << indexSegmentString << std::endl;

		/*VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    */

	//}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Modify properties of graphics extension
    static Vx::VxColor threadColor = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
	VxData::Container& parameters = graphicsExtension->getParameterContainer();
    (parameters)[CableSystems::GraphicsICD::kRadiusID].setValue(tetherRadius*60);
    (parameters)[CableSystems::GraphicsICD::kColorID].setValue(threadColor);
       // (parameters)[CableSystems::GraphicsICD::kNumberOfSidesID].setValue(10);
        //(parameters)[CableSystems::GraphicsICD::kRoundnessID].setValue(1.0);
       // (parameters)[CableSystems::GraphicsICD::kInterpolationID].setValue(false);
       // (parameters)[CableSystems::GraphicsICD::kSubdivisionPerSectionID].setValue(0);
}


void add2Tethers(std::string pathData, VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> chaserPart, VxSmartInterface<Part> **partMatrix, VxSmartInterface<Part> winchPart, int tetherCounter)
{
	// read constants
	std::stringstream constantsFileName;
	std::ifstream inFile;
	constantsFileName << pathData << "constants.txt" ;
	VxReal PI = readParameter<VxReal>("PI", constantsFileName);

	// read data for net
	std::stringstream netDataFileName;
	netDataFileName << pathData << "netData.txt";
	int NetSize = readParameter<int>("NetSize", netDataFileName);
	VxReal csiAxial = readParameter<VxReal>("csiAxial", netDataFileName);
	VxReal csiBending = readParameter<VxReal>("csiBending", netDataFileName);

	// Find indexes for central part of net, and last part of the net (used in reading ring positions)
	int indexCenterPart, indexQuarterPart; 
	if(NetSize%2 != 0) // NetSize is odd
	{
		indexCenterPart = (int)(NetSize-1)/2 ;
		indexQuarterPart = (int)(NetSize-1)/4; 
	}
	else 
		std::cout << std::endl << "ERROR: NetSize in file netData has to be odd. Please exit simulation and correct its value. \n ";
	int indexLastPart = NetSize - 1;

	// read data for tether
	std::stringstream tetherDataFileName;
	tetherDataFileName << pathData << "tetherData.txt";
	int tetherFlexible = readParameter<int>("tetherFlexible", tetherDataFileName);
	VxReal winchRadius = readParameter<VxReal>("winchRadius", tetherDataFileName);
	VxReal tetherRadius = readParameter<VxReal>("tetherRadius", tetherDataFileName);
	VxReal tetherDensity = readParameter<VxReal>("tetherDensity", tetherDataFileName);
	VxReal tetherYoungModulus = readParameter<VxReal>("tetherYoungModulus", tetherDataFileName);
	VxReal ringsNb = readParameter<VxReal>("ringsNb", tetherDataFileName);
	VxReal tethersNb = readParameter<VxReal>("tethersNb", tetherDataFileName);
	VxReal EImultiplier = readParameter<VxReal>("EImultiplier", tetherDataFileName);
	std::cout  << std::endl << "Read " << std::endl;

	// read data for chaser
	std::stringstream chaserDataFileName;
	chaserDataFileName << pathData << "chaserData.txt" ;
	VxReal chaserSideLength = readParameter<VxReal>("chaserSideLength", chaserDataFileName);
	VxReal chaserDistanceFromNet = readParameter<VxReal>("chaserDistanceFromNet", chaserDataFileName);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Create generic cable extension, including dynamic cable, graphic cable, connection of these

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
			////Creates the Dynamic Cable Properties
			//Vx::VxSmartPtr<VxSim::VxExtension> createDynamicsProperties = VxSim::VxExtensionFactory::create(CableSystemsICD::kDynamicsPropertiesPluginKey);
			//if ( nullptr != createDynamicsProperties )
			//{
			//	std::cout << "createDynamicsProperties exists" << std::endl;
			//	createDynamicsProperties->setName(CableSystemsICD::kDynamicsPropertiesName);
			//	genericCable->add(createDynamicsProperties.get());
			//}
			// Creates connection between dynamicsExtension and graphicsExtension
			VxData::Container* cablesContainer = dynamic_cast<VxData::Container*>(dynamicsExtension->getOutput(CableSystems::DynamicsICD::kCablesID));
			auto connectionContainer = VxSim::VxSmartInterface<VxContent::ConnectionContainerExtension>( connectionExtension );
			connectionContainer->create(&(*cablesContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID], graphicsExtension->getInput(VxGraphicsPlugins::SplineICD::kSplineControlPoints));
		}
	}
	// Adds the cable to the mechanism
	if (mechanism.valid())            
	{               
		mechanism->addExtension(genericCable);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Find the number of points to be specified for cable. It is: 1 for each ring + 2 attachments
	int pointsNb = ringsNb + 2; 
	std::cout << "pointsNb is " << pointsNb << std::endl;

	// initialize containers to define cable properties
	VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
	VxData::FieldBase& defFieldBase = container[kDefinitionID];
	VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
	VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
	if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(pointsNb)) ) // We will set TOT items below    
	{
		Vx::LogError(0, "Cannot resize the List of PointDefinition\n");    
	}
	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set attachments.  
	// set first attachment 
	VxData::Container* attachmentPointChaser = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));    
	if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(chaserPart.getExtension());   
		(*attachmentPointChaser)[PointDefinitionContainerID::kOffsetID].setValue(Vx::VxVector3(0.0, 0.0, -chaserSideLength/2.0));
	}
	else  // tether attached to winch
	{
		(*attachmentPointChaser)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kWinch));    
		(*attachmentPointChaser)[PointDefinitionContainerID::kPartID].setValue(winchPart.getExtension());
		(*attachmentPointChaser)[WinchDefinitionContainerID::kInverseWrappingID].setValue(true);
	}

	// create ring points
	int defPointIndex = 1;
	for (int i=0; i<ringsNb; i++)
	{
		std::ostringstream indexPointSS;
		indexPointSS << defPointIndex;
		std::string indexPointString = indexPointSS.str();
		std::cout << indexPointString << std::endl;

		VxData::Container& ringPointNet_1 = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]);  // ring in center of net
		ringPointNet_1[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kRing));
		
		if (tetherCounter == 0)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexCenterPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(1.0, 0.0, 0.0));		
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));		
			}
			else if (i==4)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][0].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));		
			}
		}
		else if (tetherCounter == 1)
		{
			if (i==0)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 0.0, -1.0));
			}
			else if (i==1)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexCenterPart].getExtension());   
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));
			}
			else if (i==2)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(-1.0, 0.0, 0.0));		
			}
			else if (i==3)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, -1.0, 0.0));		
			}
			else if (i==4)
			{
				ringPointNet_1[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexLastPart].getExtension()); 
				//ringPointNet_1[RingDefinitionContainerID::kRelativePrimaryAxisID].setValue(Vx::VxVector3(0.0, 1.0, 0.0));		
			}
		}
		defPointIndex = defPointIndex + 1;
	}
	
	// set last attachment point 
	std::ostringstream indexPointSS;
	indexPointSS << defPointIndex;
	std::string indexPointString = indexPointSS.str();
	std::cout << indexPointString << std::endl;
	VxData::Container& attachmentPointNet = dynamic_cast<VxData::Container&>(fieldBasePoints[indexPointString]); 
	attachmentPointNet[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
	if (ringsNb == 0)   // on central part of the net if there are no rings
		attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexCenterPart][indexCenterPart].getExtension());   
	else				// on part [0][indexCenterPart] if there are rings
	{	
		if (tetherCounter == 0)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[0][indexCenterPart].getExtension());   
		else if (tetherCounter == 1)
			attachmentPointNet[PointDefinitionContainerID::kPartID].setValue(partMatrix[indexLastPart][indexCenterPart].getExtension());   
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set properties of cable. 
	int nSections = 20;
	VxReal EAtrue = tetherYoungModulus * PI * pow(tetherRadius,2.00);	 // assuming tether stiffness is defined as EA
	VxReal EItrue = tetherYoungModulus / 4.0 * PI * pow(tetherRadius,4.00);
	//std::cout << EItrue << std::endl;
	VxReal densityTrue = tetherDensity * PI * pow(tetherRadius,2.0);	// linear density

	VxReal EAused;
	VxReal EIused;
	VxReal densityUsed;
	if (tetherFlexible == 1)
	{
		EAused = EAtrue*(nSections-1)/nSections; // used EA
		EIused = EItrue*(nSections-1)/nSections; // used EI
		densityUsed = densityTrue;
	}
	else
	{
		EAused = EAtrue; // used EA
		EIused = EItrue; // used EI
		densityUsed = densityTrue;
	}
	EIused = EIused*EImultiplier;
	std::cout << " The value of EI used is " << EIused << std::endl << std::endl;

	VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
	params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(tetherRadius); 
	params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(densityUsed); // kg/m
	params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
	params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
	params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(EAused/10.0);
	params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(EIused/10.0);  
	params[CableSystemParamDefinitionContainerID::kCollisionGeometryTypeID].setValue(0); // 0: None, 1: spheres, 2: capsule
	params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.0);
	//params[CableSystemParamDefinitionContainerID::kInternalDampingMaxStableLoadID].setValue(1.0);
	  
	////wn1AxialCable = PI/2.0/cableLength * pow( cableYoungModulus/cableDensity, 0.5);
	////VxReal tetherDamping = 2 * csiAxial / wn1AxialCable * cableStiffness;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (tetherFlexible == 1)	// long segment of the cable is flexible
	{
		if (winchRadius==0)	// tether attached to point on lower part of chaser, centered
		{
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(true);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(1.0);
		}
		else
		{
			VxReal initialCableLength = chaserDistanceFromNet;
			VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
			/*segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
			segmentDefinition0[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
			segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
			segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
			segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
			segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
			segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, 2.00, -1.0));
			//segmentDefinition0[SegmentDefinitionContainerID::kPointsID].setValue(Vx::VxVector3( 2.0, -2.00, -2.0));

			if (pointsNb > 2)
			{
				VxData::Container& segmentDefinition1 = dynamic_cast<VxData::Container&>(fieldBaseSegments["2"]); 
				/*segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
				segmentDefinition1[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
				segmentDefinition1[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
				segmentDefinition1[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
				segmentDefinition1[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
				segmentDefinition1[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
				segmentDefinition1[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
				segmentDefinition1[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
			
				if (pointsNb > 3)
				{
					VxData::Container& segmentDefinition2 = dynamic_cast<VxData::Container&>(fieldBaseSegments["3"]); 
					/*segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
					segmentDefinition2[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
					segmentDefinition2[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
					segmentDefinition2[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
					segmentDefinition2[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
					segmentDefinition2[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
					segmentDefinition2[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
					segmentDefinition2[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

					if (pointsNb > 4)
					{
						VxData::Container& segmentDefinition3 = dynamic_cast<VxData::Container&>(fieldBaseSegments["4"]); 
						/*segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
						segmentDefinition3[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
						segmentDefinition3[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
						segmentDefinition3[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
						segmentDefinition3[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
						segmentDefinition3[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
						segmentDefinition3[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
						segmentDefinition3[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);

						if (pointsNb > 5)
						{
							VxData::Container& segmentDefinition4 = dynamic_cast<VxData::Container&>(fieldBaseSegments["5"]); 
							///*segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
							//segmentDefinition4[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
							segmentDefinition4[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
							segmentDefinition4[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
							segmentDefinition4[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
							segmentDefinition4[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(2);
							segmentDefinition4[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
							segmentDefinition4[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
						
							if (pointsNb > 6)
							{
								VxData::Container& segmentDefinition5 = dynamic_cast<VxData::Container&>(fieldBaseSegments["6"]); 
								///*segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(1);
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionStartID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));
								//segmentDefinition5[SegmentDefinitionContainerID::kAttachmentDirectionEndID].setValue(Vx::VxVector3(0.0, 0.0, 1.0));*/
								segmentDefinition5[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
								segmentDefinition5[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
								segmentDefinition5[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
								segmentDefinition5[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(0);
								segmentDefinition5[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);
								segmentDefinition5[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(initialCableLength/nSections);
							}
						}
					}
				}
			}
		}
	}

	// Set properties of segments
	//int defSegmentIndex = 1;
	//for (int i=0; i<pointsNb; i++)
	//{
		//std::ostringstream indexSegmentSS;
		//indexPointSS << defSegmentIndex;
		//std::string indexSegmentString = indexSegmentSS.str();
		//std::cout << indexSegmentString << std::endl;

		/*VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["1"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    */

	//}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Modify properties of graphics extension
    static Vx::VxColor threadColor = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
	VxData::Container& parameters = graphicsExtension->getParameterContainer();
    (parameters)[CableSystems::GraphicsICD::kRadiusID].setValue(tetherRadius*60);
    (parameters)[CableSystems::GraphicsICD::kColorID].setValue(threadColor);
       // (parameters)[CableSystems::GraphicsICD::kNumberOfSidesID].setValue(10);
        //(parameters)[CableSystems::GraphicsICD::kRoundnessID].setValue(1.0);
       // (parameters)[CableSystems::GraphicsICD::kInterpolationID].setValue(false);
       // (parameters)[CableSystems::GraphicsICD::kSubdivisionPerSectionID].setValue(0);
}

