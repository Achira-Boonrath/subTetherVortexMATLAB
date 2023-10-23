// Create cable for one thread of the net

// Created by Eleonora Botta, February 27, 2017
// Modified by Eleonora Botta, March 3, 2017: way points in the cables to have length of threads = 4*distance of nodes
// Modified by Eleonora Botta, March 6, 2017: modified way points to have less parts in the simulation
// Modified by Eleonora Botta, March 14, 2017: added material for cable
// Modified by Eleonora Botta, March 16, 2017: pass some parameters for cables, previously read from netData.txt
// Modified by Eleonora Botta, March 17, 2017: make nSections dependent on wayPointsNb.They are related.
// Modified by Eleonora Botta, March 27, 2017: make threads attachments cantilevered, instead of ball and socket. 
// Modified by Eleonora Botta, April 17, 2017:	changed VxFatalError with LogError for Vortex Studio 2017a
//												added  #include <Vx/VxPart.h>  for Vortex Studio 2017a
// Modified by Eleonora Botta, April 17, 2017:	changed attachment of nodes, with ball and socket joints

// Problems:   check axial and bending stiffness: now bending stiffness is artificially low. 
//				Length is not exactly the same as that expected, but very close
//				instead of not having collision geom in threads, I should eliminate it from nodes with netModel=2
//				there are too many parts in the simulation

#include "MyLibraryNet/addCableForNetThread.h"

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

#include <iostream>
#include <fstream>

using namespace Vx;
using namespace VxDynamics;
using namespace VxSim;
using namespace CableSystems;
using namespace CableSystems::DynamicsICD;


void addCableForNetThread(VxSmartInterface<Mechanism> mechanism, VxSmartInterface<Part> part0, VxSmartInterface<Part> part1, VxReal threadStiffness, VxReal threadDamping, VxReal threadBendingStiffness, VxReal threadBendingDamping, VxReal MeshLength, VxReal netRadius, VxReal linearNetDensity, int wayPointsNb, int bendingPresentInNet, int netCGtype )
{
	int threadFlexible = 1;
	int nSections = 3;
	if ( wayPointsNb == 3 ) 
		nSections = 4;
	/*else if ( wayPointsNb == 0 ) 
		nSections = 1;*/

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
					//std::cout << "cableDynamicsExtension exists" << std::endl;
					dynamicsExtension->setName(CableSystemsICD::kDynamicsGenericCableName);
					genericCable->add(dynamicsExtension.get());
			}
			// Creates the Graphics Extension
			graphicsExtension = VxSim::VxExtensionFactory::create(VxGraphicsPlugins::SplineICD::kFactoryKey);
			if ( nullptr != graphicsExtension )
			{
				//std::cout << "graphicsExtension exists" << std::endl;
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

	// initialize containers to define cable properties
	VxData::Container& container = dynamicsExtension->getParameterContainer(); // Gets the VxData::Container holding all parameter fields. In Container, the custom data structure is created using VxData::FieldBase
	VxData::FieldBase& defFieldBase = container[kDefinitionID];
	VxData::Container& definition = dynamic_cast<VxData::Container&>(defFieldBase);
	VxData::FieldBase& fieldBasePoints = definition[CableSystemDefinitionContainerID::kPointDefinitionsID];
	if ( ! fieldBasePoints["size"].setValue(static_cast<unsigned int>(2)) ) // We will set TOT items below    
	{
		Vx::LogError(0, "Cannot resize the List of PointDefinition\n");      
	}
	VxData::FieldBase& fieldBaseSegments = definition[CableSystemDefinitionContainerID::kSegmentDefinitionsID];
	VxData::FieldBase& fieldBaseParams = definition[CableSystemDefinitionContainerID::kParamDefinitionID];

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set attachments.  

	// set first attachment 
	VxData::Container* attachment0 = dynamic_cast<VxData::Container*>(&(fieldBasePoints["0"]));    
	(*attachment0)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
	(*attachment0)[PointDefinitionContainerID::kPartID].setValue(part0.getExtension());   
	VxVector3 pos0 = part0->getVxPart()->getPosition();

	// set second attachment 
	VxData::Container* attachment1 = dynamic_cast<VxData::Container*>(&(fieldBasePoints["1"]));    
	(*attachment1)[PointDefinitionContainerID::kPointTypeID].setValue(VxEnum(PointDefinitionType::kAttachmentPoint));
	(*attachment1)[PointDefinitionContainerID::kPartID].setValue(part1.getExtension());   
	VxVector3 pos1 = part1->getVxPart()->getPosition();

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Set properties of cable. 
	VxReal EAtrue = threadStiffness * MeshLength;
	VxReal EItrue = threadBendingStiffness * MeshLength;
	VxReal densityTrue = linearNetDensity;	// linear density

	VxReal EAused;
	VxReal EIused;
	VxReal densityUsed;
	if ( (threadFlexible == 1) && (nSections != 1) )
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
	if (bendingPresentInNet == 0)
	{
		EIused = 0.00001;
		threadBendingDamping = 0.00001;
	}
	//EIused = EItrue/9;  // in net starting fully deployed, this value makes results more similar to LP with bending stiffness
	//EIused = EItrue/100;  // in net deploying, this value makes results more similar to LP with bending stiffness
	//EIused = 0.00001;
	//threadBendingDamping = 0.00001;
	std::cout << "The used EI is " << EIused << std::endl;

	VxData::Container& params = dynamic_cast<VxData::Container&>(fieldBaseParams);   
	params[CableSystemParamDefinitionContainerID::kRadiusID].setValue(netRadius); 
	params[CableSystemParamDefinitionContainerID::kLinearDensityID].setValue(densityUsed); // kg/m
	params[CableSystemParamDefinitionContainerID::kAxialStiffnessID].setValue(EAused);   
	params[CableSystemParamDefinitionContainerID::kBendingStiffnessID].setValue(EIused);    
	params[CableSystemParamDefinitionContainerID::kAxialDampingID].setValue(threadDamping*MeshLength); 
	params[CableSystemParamDefinitionContainerID::kBendingDampingID].setValue(threadBendingDamping*MeshLength);  	 
	params[CableSystemParamDefinitionContainerID::kCompressionMaxForceID].setValue(0.0);
	std::string MyCableMaterialName = "MyCableMaterial";
	params[CableSystemParamDefinitionContainerID::kMaterialNameID].setValue(MyCableMaterialName);
	  
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if (threadFlexible == 1)	
	{
		VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeStartID].setValue(0);	// 0: ball and socket; 1: cantilevered
		segmentDefinition0[SegmentDefinitionContainerID::kAttachmentTypeEndID].setValue(0);		// 0: ball and socket; 1: cantilevered
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(true);    
		segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(netCGtype);
		segmentDefinition0[SegmentDefinitionContainerID::kSelfCollisionID].setValue(false);

		if (wayPointsNb == 3)
		{
			VxData::FieldBase& fb = segmentDefinition0[SegmentDefinitionContainerID::kPointsID];
			VxData::FieldArrayBase& pointsFAB = dynamic_cast< VxData::FieldArrayBase& >(fb);
			VxData::FieldArray<VxData::Field<VxVector3> >& pointsFA = dynamic_cast< VxData::FieldArray<VxData::Field<VxVector3> >& >(fb);
			pointsFA.resize(wayPointsNb);
			
			VxVector3 fromP0toP1 = pos1 - pos0;
			VxReal distanceP0toP1 = fromP0toP1.normalize(); //make into unit vector and return original length
			VxVector3 perpendicularDirection = VxVector3( -fromP0toP1.y(), fromP0toP1.x(), 0.0 );
			//VxVector3 perpendicularDirection = VxVector3( -fromP0toP1.z(), 0.0, +fromP0toP1.x() );
			segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(MeshLength/(nSections-1)*2);
				
			VxReal stepAlongAxis = distanceP0toP1 / nSections;
			VxReal stepPerpToAxis = pow( pow(MeshLength/nSections, 2.0) - pow(stepAlongAxis, 2.0) , 0.5 );
			/*VxReal stepAlongAxis = distanceP0toP1 / 4.0;
			VxReal stepPerpToAxis = pow( pow(MeshLength/4.0, 2.0) - pow(stepAlongAxis, 2.0) , 0.5 );*/

			pointsFA[0].setValue( pos0 + (stepAlongAxis*fromP0toP1 + stepPerpToAxis*perpendicularDirection) );
			pointsFA[1].setValue( pos0 + (2.0*stepAlongAxis*fromP0toP1 ) );
			pointsFA[2].setValue( pos0 + (3.0*stepAlongAxis*fromP0toP1 - stepPerpToAxis*perpendicularDirection) );

			segmentDefinition0[SegmentDefinitionContainerID::kMaxNumberOfSectionsID].setValue(nSections);
		}

		//if (wayPointsNb == 0)
		//{
		//	//std::cout << "I am setting the section length = " << (MeshLength/4.0) <<  std::endl;
		//	//segmentDefinition0[SegmentDefinitionContainerID::kPreferredSectionLengthID].setValue(MeshLength/4.0);
		//	//segmentDefinition0[SegmentDefinitionContainerID::kMaxNumberOfSectionsID].setValue(4);
		//}
		
	}

	else if (threadFlexible == 0)	
	{
		VxData::Container& segmentDefinition0 = dynamic_cast<VxData::Container&>(fieldBaseSegments["0"]); 
		segmentDefinition0[SegmentDefinitionContainerID::kSupportTorsionID].setValue(false);    
		segmentDefinition0[SegmentDefinitionContainerID::kDragCoefficientID].setValue(0.0000001);
		segmentDefinition0[SegmentDefinitionContainerID::kFlexibleID].setValue(false);    
		//segmentDefinition0[SegmentDefinitionContainerID::kCollisionGeometryTypeID].setValue(netCGtype);
	}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Modify properties of graphics extension
    static Vx::VxColor threadColor = Vx::VxColor(1.0,0.0,0.0,1.0); // (red, green, blue, transparency)
	VxData::Container& parameters = graphicsExtension->getParameterContainer();
    (parameters)[CableSystems::GraphicsICD::kRadiusID].setValue(netRadius*5);
    (parameters)[CableSystems::GraphicsICD::kColorID].setValue(threadColor);
     
}
