// Created by Liam Field 3/27/22
// PID controller for tether length (distance joint)

#include "MyLibraryNet/TetherPIDThrustControl.h"
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

VxMath::VxVector4 TetherPIDThrustControl(VxSmartInterface<Part>* chaserPartPtr, VxSmartInterface<Part>* tetheredTargetPartPtr,VxVector3 K,double deltaL,double error,const int numNode,FILE* outPID)
{
	VxVector3 angVel = (*chaserPartPtr)->getVxPart()->getAngularVelocity();
	VxQuaternion ChaserQ;
		(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
		VxVector3 chaserAngVel = ChaserQ.getInverse().rotate(angVel);

		/*Vx::VxMassProperties MC;
		MC = (*tetheredTargetPartPtr)->getVxPart()->getMassProperties();
		Vx::VxInertiaTensor H;
		MC.getInertiaTensorLocal(H);*/
		//Getting Attachment Positions
		VxSmartInterface<Assembly> Assem = (*chaserPartPtr)->getParentAssembly();
		VxSim::VxSmartInterface<DistanceJoint> v = Assem->getConstraints()[0];
		VxSim::VxSmartInterface<DistanceJoint> v1 = Assem->getConstraints()[numNode];
		
		VxVector3 chaserAtt = v->inputAttachment1.position;
		VxVector3 targetAtt = v1->inputAttachment2.position;
		/*double StiffTest= v1->getVxConstraint()->getRelaxationStiffness(0);
		double DampTest= v1->getVxConstraint()->getRelaxationDamping(0);
		
		 VxReal TensionForce = v->getVxConstraint()->getConstraintEquationForce(0);*/
		//Vx::VxQuaternion ChaserQ;
		VxVector3 HHH = ChaserQ.rotate(chaserAtt);
		//(*chaserPartPtr)->getVxPart()->getOrientationQuaternion(ChaserQ);
		VxVector3 chaserAttPos = ChaserQ.rotate(chaserAtt) + (*chaserPartPtr)->getVxPart()->getPosition();
		VxVector3 WYZ = (*chaserPartPtr)->getVxPart()->getPosition();

		Vx::VxQuaternion TargetQ;
		(*tetheredTargetPartPtr)->getVxPart()->getOrientationQuaternion(TargetQ);

		VxVector3 angVelT = (*tetheredTargetPartPtr)->getVxPart()->getAngularVelocity();
		VxVector3 targetAngVel = TargetQ.getInverse().rotate(angVelT);

		VxVector3 targetAttPos = TargetQ.rotate(targetAtt) + (*tetheredTargetPartPtr)->getVxPart()->getPosition();

		VxVector3 L = targetAttPos - chaserAttPos;
		double Lnorm = L.norm();
		
		
		double Ldot = ((*chaserPartPtr)->getVxPart()->getLinearVelocity() + angVel.cross(ChaserQ.rotate(chaserAtt)) - (*tetheredTargetPartPtr)->getVxPart()->getLinearVelocity() - angVelT.cross(TargetQ.rotate(targetAtt))).dot(L/Lnorm);

		double processVar = (30 + deltaL - Lnorm);
		double Fc = K[0] * (30 + deltaL - Lnorm) + K[1] * (Ldot) + K[2] * (error);
		VxVector3 U = -Fc * L / Lnorm;
		VxMath::VxVector4 UVecOut = VxMath::VxVector4(U.x(), U.y(), U.z(), 30 + deltaL - Lnorm);





		fprintf(outPID, "%f \t %f \t %f \t %f \t %f \t %f", Fc,U.x(),U.y(),U.z(),Ldot,error);
		
		fprintf(outPID, "\n");
		
		return UVecOut;



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