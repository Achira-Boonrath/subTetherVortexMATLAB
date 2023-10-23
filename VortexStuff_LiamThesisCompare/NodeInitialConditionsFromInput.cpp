// Created by Liam Field 4/22/22
// 

#include "MyLibraryNet/NodeInitialConditionsFromInput.h"
#include "MyLibraryNet/QuatMat.h"
#include "MyLibraryNet/readTabDelimVector.h"


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

void NodeInitialConditionsFromInput(VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* tetheredTargetPartPtr, VxSmartInterface<Part>* partMatrix, int numNode, std::string pathData)
{
		
	
		


	VxVector3 angVel = (*chaserPartPtr)->getVxPart()->getAngularVelocity();
	VxQuaternion ChaserQ;
	(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
	VxVector3 chaserAngVel = ChaserQ.getInverse().rotate(angVel);

	VxVector3 angVelT = (*tetheredTargetPartPtr)->getVxPart()->getAngularVelocity();
	VxQuaternion targetQ;
	(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(targetQ);
	VxVector3 targetAngVel = targetQ.getInverse().rotate(angVelT);

	//Getting Attachment Positions
	// Getting the chaser attachmentment point from the constraints on the chaser
	const int count = (*chaserPartPtr)->getVxPart()->getConstraintCount();
	VxConstraint* v = (*chaserPartPtr)->getVxPart()->getConstraint(count - 1);
	VxReal3 chaserAtt;
	v->getPartAttachmentPositionRel(0, chaserAtt);


	const int countT = (*tetheredTargetPartPtr)->getVxPart()->getConstraintCount();
	VxConstraint* vT = (*tetheredTargetPartPtr)->getVxPart()->getConstraint(countT - 1);
	VxReal3 targetAtt;
	vT->getPartAttachmentPositionRel(0, targetAtt);
	//DistanceJoint* G = dynamic_cast<DistanceJoint*>(v);
	//auto chaserAtt = G->inputAttachment1.position;

	//Vx::VxQuaternion ChaserQ;
	//VxVector3 HHH = ChaserQ.rotate(chaserAtt);
	//(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
	VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
	VxVector3 targetAttPos = targetQ.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();


	VxVector3 L = targetAttPos - chaserAttPos;
	VxReal Lnorm = L.norm();
	VxVector3 positionInt = L / (numNode + 1);
	VxVector3 RelVel = -((*chaserPartPtr)->getVxPart()->getLinearVelocity() + angVel.cross(ChaserQ.rotate(chaserAtt)) - (*tetheredTargetPartPtr)->getVxPart()->getLinearVelocity()- angVelT.cross(targetQ.rotate(targetAtt)));
	VxVector3 velocityInt = RelVel / (numNode + 1);
	VxVector3 chsAttVel = (*chaserPartPtr)->getVxPart()->getLinearVelocity() + angVel.cross(ChaserQ.rotate(chaserAtt));


	for (int i = 0; i < numNode; i++)
	{
		VxSmartInterface<Part> p = partMatrix[i];
		// p->setLocalTransform(VxMath::Transformation::createTranslation(chaserAttPos + positionInt * (i + 1)));
		p->getVxPart()->setPosition((chaserAttPos + positionInt * (i + 1)));
		p->setLinearVelocity(chsAttVel + velocityInt * (i + 1));


	}


	/*std::stringstream nodePosFileName;
	nodePosFileName << pathData << "nodeInPos.txt";

	std::stringstream nodeVelFileName;
	nodeVelFileName << pathData << "nodeInVel.txt";


		for (int i = 0; i < numNode; i++)
		{

			VxVector3 nodePos = readTabDelimVector(nodePosFileName, i+1);
			VxVector3 nodeVel = readTabDelimVector(nodeVelFileName, i+1);





			VxSmartInterface<Part> p = partMatrix[i];
			p->setLocalTransform(VxMath::Transformation::createTranslation(nodePos));
			p->setLinearVelocity(nodeVel);


		}*/


}