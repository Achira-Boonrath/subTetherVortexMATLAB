// Created by Eleonora Botta, September 30, 2016
// Modified by Eleonora Botta, November 3, 2016: save on a separate file the positions of the cable points (for all cables)
// Modified by Eleonora Botta, November 5, 2016: rearranged the file with the positions of the cable points (for all cables)
// Modified by Eleonora Botta, December 28, 2016: added a version of function without outTetherPoints

// Save dynamics of the tether and winch linear velocity

#include "MyLibraryNet/saveDynamicsQuantitiesTether.h"

#include <VxSim/VxSmartInterface.h>
#include <VxSim/IExtension.h>
#include <CableSystems/DynamicsICD.h>
#include <VxDynamics/Hinge.h>
#include <Vx/VxHinge.h>

#include <VxSim/IMobile.h>	// this is needed by VxSmartInterface<IExtension> cableExtension

#include <VxData/FieldArray.h>
#include <VxData/Vector.h>
#include <Vx/VxSmartPtr.h>

#include <iostream>

using namespace VxSim;
using namespace Vx;
using namespace VxDynamics;
using namespace CableSystems::DynamicsICD::PointInfoContainerID;

void saveDynamicsQuantitiesTether(VxSmartInterface<IExtension> cableExtension, FILE* outTether)
{
	// save cable properties in time
	VxReal cableLength = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kTotalLengthID)->toReal();
	VxReal cableElongation = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kElongationID)->toReal();
	VxReal cableMaxTension = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kMaxTensionID)->toReal();
	VxReal cableBreakage = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kHasBrokenID)->toReal();
	fprintf(outTether, "%f \t %f \t %f \t %f", cableLength, cableElongation, cableMaxTension, cableBreakage);
	//fprintf( outTether, "%f \t %f \t %f \t %f ", cableLength, cableElongation, cableMaxTension, cableBreakage);

	// save hinge velocity in time
	//fprintf( outTether, "\t %f", currentWinchSpoolingVelocity );


	//// save points positions
	//VxData::Container* cableContainer = dynamic_cast<VxData::Container*>(cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kCablesID));
	//int numCables = (*cableContainer)[CableSystems::DynamicsICD::CablesContainerID::kNumCablesID].toInteger();
	////std::cout << "There are " << numCables << " cables. " << std::endl;

	//const VxData::FieldArrayBase& pointInfos = dynamic_cast<const VxData::FieldArrayBase&>((*cableContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID]);
	///*std:: cout << pointInfos.getSizeField() << std::endl;*/

	//for (int cableIndex = 0; cableIndex < numCables; ++cableIndex)
	//{
	//	// Get the Point Info container
	//	const VxData::Container& pointInfo = dynamic_cast<const VxData::Container&>(pointInfos.at(cableIndex));
	//	int numberOfPoints = pointInfo[CableSystems::DynamicsICD::PointInfoContainerID::kNumPointsID].toInteger();
	//	//std::cout << "Cable " << cableIndex << " has " << numberOfPoints << " points." << std::endl;

	//}

	// go one line below in files
	fprintf(outTether, "\n");

}

void saveDynamicsQuantitiesTether(VxSmartInterface<IExtension> cableExtension, FILE* outTether, VxSmartInterface<Hinge>* hingeForWinchPtr, float currentWinchSpoolingVelocity)
{
	// save cable properties in time
	VxReal cableLength = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kTotalLengthID)->toReal();
	VxReal cableElongation = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kElongationID)->toReal();
	VxReal cableMaxTension = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kMaxTensionID)->toReal();
	VxReal cableBreakage = cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kBreakNowID)->toReal();
	float winchvel = currentWinchSpoolingVelocity;
	fprintf( outTether, "%f \t %f \t %f \t %f", cableLength, cableElongation, cableMaxTension, winchvel);
	//fprintf( outTether, "%f \t %f \t %f \t %f ", cableLength, cableElongation, cableMaxTension, cableBreakage);
	
	// save hinge velocity in time
	//fprintf( outTether, "\t %f", currentWinchSpoolingVelocity );

	
	// save points positions
	VxData::Container* cableContainer = dynamic_cast<VxData::Container *>(cableExtension->getProxy()->getOutput(CableSystems::DynamicsICD::kCablesID));
	int numCables = (*cableContainer)[CableSystems::DynamicsICD::CablesContainerID::kNumCablesID].toInteger();
	//std::cout << "There are " << numCables << " cables. " << std::endl;
	
	const VxData::FieldArrayBase& pointInfos =  dynamic_cast<const VxData::FieldArrayBase&>((*cableContainer)[CableSystems::DynamicsICD::CablesContainerID::kPointInfosID]); 
	/*std:: cout << pointInfos.getSizeField() << std::endl;*/
	
	for (int cableIndex=0; cableIndex<numCables; ++cableIndex)
	{
		// Get the Point Info container
		const VxData::Container& pointInfo = dynamic_cast<const VxData::Container &>(pointInfos.at(cableIndex));
		int numberOfPoints = pointInfo[CableSystems::DynamicsICD::PointInfoContainerID::kNumPointsID].toInteger();                        
		//std::cout << "Cable " << cableIndex << " has " << numberOfPoints << " points." << std::endl;
		
	}

	// go one line below in files
	fprintf( outTether, "\n" );

}