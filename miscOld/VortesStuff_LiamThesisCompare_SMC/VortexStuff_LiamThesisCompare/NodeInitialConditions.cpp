// Created by Liam Field 4/22/22
// 

#include "MyLibraryNet/NodeInitialConditions.h"
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

void NodeInitialConditions(VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* tetheredTargetPartPtr, VxSmartInterface<Part>* partMatrix, int numNode)
{
	VxVector3 angVel = (*chaserPartPtr)->getVxPart()->getAngularVelocity();
	VxQuaternion ChaserQ;
		(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
		VxVector3 chaserAngVel = ChaserQ.getInverse().rotate(angVel);


		//Getting Attachment Positions
		VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
		VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[0];
		VxVector3 chaserAtt = v->inputAttachment1.position;
		VxVector3 targetAtt = v->inputAttachment2.position;

		//Vx::VxQuaternion ChaserQ;
		VxVector3 HHH = ChaserQ.rotate(chaserAtt);
		//(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
		VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();

		Vx::VxQuaternion TargetQ;
		(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);

		VxVector3 angVelT = (*tetheredTargetPartPtr)->getVxPart()->getAngularVelocity();
		VxVector3 targetAngVel = TargetQ.getInverse().rotate(angVelT);

		VxVector3 targetAttPos = TargetQ.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();

		VxVector3 L = targetAttPos - chaserAttPos;
		VxReal Lnorm = L.norm();
		VxVector3 positionInt = L / (numNode+1);
		VxVector3 RelVel = -((*chaserPartPtr)->getVxPart()->getLinearVelocity() + angVel.cross(ChaserQ.rotate(chaserAtt)) - (*tetheredTargetPartPtr)->getVxPart()->getLinearVelocity() - angVelT.cross(TargetQ.rotate(targetAtt)));
		VxVector3 velocityInt = RelVel / (numNode + 1);
		VxVector3 chsAttVel = (*chaserPartPtr)->getVxPart()->getLinearVelocity() + angVel.cross(ChaserQ.rotate(chaserAtt));
		
		
		for (int i = 0; i < numNode; i++)
		{
			VxSmartInterface<Part> p = partMatrix[i];
			p->setLocalTransform(VxMath::Transformation::createTranslation(chaserAttPos+positionInt*(i+1)));
			p->setLinearVelocity(chsAttVel + velocityInt * (i + 1));


		}

		



	



//		qe_vec[0] = qe_s;
//		VxMath::VxVector4 J = DesQ.operator double *();
//		VxMath::VxVector4 B = operator*(RotMat, VxMath::VxVector4(xc, 0));
//		VxQuaternion qe_vec = operator*(QMatD.transpose(), DesQ.operator double* ());

//		VxVector3 N = B.xyz();
		
		


		// Scalar Last
		//QMatD.operator()(0, 0) = DesQ.w();
		//QMatD.operator()(1, 1) = DesQ.w();
		//QMatD.operator()(2, 2) = DesQ.w();
		//QMatD.operator()(0, 1) = -DesQ.z();
		//QMatD.operator()(1, 0) = DesQ.z();
		//QMatD.operator()(0, 2) = DesQ.y();
		//QMatD.operator()(2, 0) = -DesQ.y();
		//QMatD.operator()(1, 2) = -DesQ.x();
		//QMatD.operator()(2, 1) = DesQ.x();

		//QMatD.operator()(3, 0) = -DesQ.x();
		//QMatD.operator()(3, 1) = -DesQ.y();
		//QMatD.operator()(3, 2) = -DesQ.z();
		//QMatD.operator()(3, 3) = 0;
		//////////////

}